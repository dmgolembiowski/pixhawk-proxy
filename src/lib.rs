#![feature(duration_as_u128)]
extern crate prost;
#[macro_use]
extern crate prost_derive;

#[macro_use]
extern crate serde_derive;
extern crate serde;
extern crate serde_json;

extern crate lmcp_sentinelizer;
extern crate uxas_attribute_message;

extern crate lmcp;
extern crate range_check;

//
// Standard includes
//
use std::collections::VecDeque;
use std::f32::NAN;
use std::time::{Duration, Instant};

// To check ranges for sending bytes to the autopilot
// (u32/i32 -> u8/i8 conversion)
use range_check::Within;

//
// LMCP includes
//
use lmcp::afrl::cmasi::air_vehicle_state::AirVehicleState;
use lmcp::afrl::cmasi::air_vehicle_state::AirVehicleStateT;
use lmcp::afrl::cmasi::entity_state::EntityStateT;

use lmcp::afrl::cmasi::air_vehicle_configuration::AirVehicleConfiguration;
use lmcp::afrl::cmasi::air_vehicle_configuration::AirVehicleConfigurationT;
use lmcp::afrl::cmasi::entity_configuration::EntityConfigurationT;

use lmcp::afrl::cmasi::flight_profile::FlightProfile;
use lmcp::afrl::cmasi::flight_profile::FlightProfileT;

use lmcp::afrl::cmasi::waypoint::WaypointT;

use lmcp::afrl::cmasi::session_status::SessionStatus;

use lmcp::afrl::cmasi::altitude_type::AltitudeType;
pub use lmcp::Message as LmcpMessage;

//
// Our own includes
//
use lmcp_sentinelizer::LmcpSentinelizer;
use uxas_attribute_message::AddressedAttributedMessage;

//
// Pixhawk proxy includes
//
mod battery;

// Include the `items` module, which is generated from items.proto.
pub mod mavlink_common {
    include!(concat!(env!("OUT_DIR"), "/mavlink.common.rs"));
}

// export modules
use battery::BatteryStatus;
pub use mavlink_common::*;

/// TODO: send a message when the proxy times out
const DEFAULT_KEEP_ALIVE_TIMEOUT: Duration = Duration::from_millis(10_000);

pub enum ProxyStatus {
    SendConfiguration,
    UploadInitMission,
    Arm,
    TakeOff,
    MissionReady,
}

/// Structure containing all necessary proxy data
pub struct PixhawkProxy {
    /// debug print on/off
    debug: bool,
    /// When did we receive the last HEARTBEAT message
    last_heartbeat: Instant,
    /// Timestamp of the last received ATTITUDE message
    last_attitude_since_boot: Duration,
    /// Timestamp of the last HIGHRES_IMU message
    last_highres_imu_since_boot: Duration,
    /// BATTERY_STATUS message for book-keeping (and deriving `energy_rate`)
    battery_status: BatteryStatus,
    /// The up-to-date AirVehicleState LMCP struct
    current_air_vehicle_state: AirVehicleState,
    /// AirVehicleConfiguration Message
    air_vehicle_configuration: Option<AirVehicleConfiguration>,
    /// List of current waypoints
    waypoints: Vec<Box<WaypointT>>,
    /// Start time
    timer: Instant,
    /// State machine for PixhawkProxy
    proxy_status: ProxyStatus,
}

impl Default for PixhawkProxy {
    fn default() -> PixhawkProxy {
        PixhawkProxy {
            debug: false,
            last_heartbeat: Instant::now() - DEFAULT_KEEP_ALIVE_TIMEOUT,
            last_attitude_since_boot: Duration::new(0, 0),
            last_highres_imu_since_boot: Duration::new(0, 0),
            current_air_vehicle_state: AirVehicleState::default(),
            battery_status: BatteryStatus::new(),
            air_vehicle_configuration: None,
            waypoints: vec![],
            timer: Instant::now(),
            proxy_status: ProxyStatus::SendConfiguration,
        }
    }
}

impl PixhawkProxy {
    /// For serialization of LMCP messages
    const DEFAULT_SER_BUFFER_SIZE: usize = 8200;

    /// TODO: these should be configurable
    const AC_ID: i64 = 400;
    const SYSTEM_ID: u32 = 1;
    const COMPONENT_ID: u32 = 0;

    pub fn set_debug(&mut self, debug: bool) {
        self.debug = debug;
    }

    /// handle incoming messages, does nothing for now
    pub fn handle_lmcp_msg(&mut self, lmcp_msg: LmcpMessage) -> (VecDeque<LmcpMessage>, VecDeque<mavlink_common::MavlinkMessage>) {
        let mut lmcp_queue = VecDeque::new();
        let mut mavlink_queue = VecDeque::new();

        match lmcp_msg {
            LmcpMessage::AfrlCmasiMissionCommand(data) => {
                println!("Got mission command");
                if data.vehicle_id == Self::AC_ID {
                    self.waypoints = data.waypoint_list
                }
                let mut msg = mavlink_common::MavlinkMessage::default();
                let mut mav_msg = mavlink_common::MissionCount::default();
                mav_msg.target_system = Self::SYSTEM_ID;
                mav_msg.target_component = Self::COMPONENT_ID;
                mav_msg.count = self.waypoints.len() as u32;
                msg.msg_set = Some(mavlink_common::mavlink_message::MsgSet::MissionCount(mav_msg));
                mavlink_queue.push_back(msg);
            },
            LmcpMessage::AfrlCmasiLineSearchTask(_) => {
                //self.proxy_status = ProxyStatus::MissionReady;
            },
            _ => {},
        }

        (lmcp_queue, mavlink_queue)
    }

    /// Parse an incoming Mavlink proto message and if it contains relevant data,
    /// update the AirVehicleState struct
    pub fn handle_mavlink_msg(
        &mut self,
        proto_msg: mavlink_common::MavlinkMessage,
    ) -> (VecDeque<LmcpMessage>, VecDeque<mavlink_common::MavlinkMessage>) {
        let mut lmcp_queue = VecDeque::new();
        let mut mavlink_queue = VecDeque::new();

        if let Some(msg) = proto_msg.msg_set {
            use mavlink_common::mavlink_message::MsgSet::*;
            match msg {
                Heartbeat(_data) => {
                    /*
                    match self.proxy_status {
                        ProxyStatus::SendConfiguration => {
                            // Create a new air vehicle configuration
                            *self.current_air_vehicle_state.id_mut() = Self::AC_ID;

                            self.air_vehicle_configuration = Some(PixhawkProxy::create_config_message());
                            let msg = PixhawkProxy::create_config_message();
                            if self.debug {
                                println!("Sending a configuration message {:?}", msg);
                            }
                            lmcp_queue.push_front(LmcpMessage::AfrlCmasiAirVehicleConfiguration(msg));
                        },
                        _ => {},
                    }*/

                    // TODO: check if the heartbeat data are sane
                    self.last_heartbeat = Instant::now();
                    *self.current_air_vehicle_state.time_mut() = self.timer.elapsed().as_millis() as i64;
                    let msg = self.current_air_vehicle_state.clone();
                    if self.debug {
                        println!("Sending air vehicle state: {:?}", msg);
                    }
                    lmcp_queue.push_front(LmcpMessage::AfrlCmasiAirVehicleState(msg));
                },
                VfrHud(data) => {
                    // vehicle true airspeed [m/s]
                    *self.current_air_vehicle_state.airspeed_mut() = data.airspeed;
                    assert!(data.heading.is_within(-180..180));
                    *self.current_air_vehicle_state.course_mut() = data.heading as f32;
                    *self.current_air_vehicle_state.groundspeed_mut() = data.groundspeed;
                },
                GlobalPositionInt(data) => {
                    // Course/Groundtrack angle of the entity referenced to true North [deg]
                    // The perceived entity location. A valid EntityState must include Location (null not allowed)
                    {
                        *self.current_air_vehicle_state.location.latitude_mut() = data.lat as f64 / 1e7;
                        *self.current_air_vehicle_state.location.longitude_mut() = data.lon as f64 / 1e7;
                        // TODO: fix WGS84 to AMSL projection
                        *self.current_air_vehicle_state.location.altitude_mut() = data.alt as f32 / 1000.0;
                        // TODO: fix the altitude type (maybe use relative altitude?)
                        *self.current_air_vehicle_state.location.altitude_type_mut() = AltitudeType::MSL;
                    }
                    // Vertical speed (positive upwards) of the vehicle in the inertial frame (rate of change of altitude) [m/s]
                    *self.current_air_vehicle_state.vertical_speed_mut() = data.vz as f32 / -100.0;
                },
                HighresImu(data) => {
                    // Timestamp (microseconds, synced to UNIX time or since system boot) [us]
                    // TODO: check for staleness
                    self.last_highres_imu_since_boot = Duration::from_micros(data.time_usec);
                    // Acceleration in the body x-direction (postive out nose) [m/s^2]
                    *self.current_air_vehicle_state.udot_mut() = data.xacc;
                    // Acceleration in the body y-direction (postive out nose) [m/s^2]
                    *self.current_air_vehicle_state.vdot_mut() = data.yacc;
                    // Acceleration in the body z-direction (postive out nose) [m/s^2]
                    *self.current_air_vehicle_state.wdot_mut() = data.zacc;
                },
                Attitude(data) => {
                    // Timestamp (milliseconds since system boot) [ms]
                    // TODO: check that it is monotonically increasing & check for staleness
                    self.last_attitude_since_boot = Duration::from_millis(data.time_boot_ms.into());
                    // Angle between true North and the projection of the body x-axis in the North-East plane. [deg]
                    // TODO: Does pixhawk compensate for True vs Magnetic north?
                    *self.current_air_vehicle_state.heading_mut() = data.yaw.to_degrees();
                    // Pitch of vehicle around body y-axis (positive upwards) [deg]
                    *self.current_air_vehicle_state.pitch_mut() = data.pitch.to_degrees();
                    // Roll angle of the vehicle around body x-axis (positive right wing down) [deg]
                    *self.current_air_vehicle_state.roll_mut() = data.roll.to_degrees();
                    // Roll-rate of vehicle (angular velocity around body x-axis).  Positive right-wing down. [deg/s]
                    *self.current_air_vehicle_state.p_mut() = data.rollspeed.to_degrees();
                    // pitch rate of the vehicle (angular velocity around body y-axis).  Positive nose-up. [deg/s]
                    *self.current_air_vehicle_state.q_mut() = data.pitchspeed.to_degrees();
                    // yaw rate of the vehicle (angular velocity around body z-axis).  Positive nose right. [deg/s]
                    *self.current_air_vehicle_state.r_mut() = data.yawspeed.to_degrees();
                },
                BatteryStatus(data) => {
                    // TODO: check for -1
                    self.battery_status.update(Instant::now(), data.battery_remaining as f32);
                    if let Some(energy_available) = self.battery_status.energy_available {
                        *self.current_air_vehicle_state.energy_available_mut() = energy_available;
                    }
                    if let Some(energy_rate) = self.battery_status.energy_rate {
                        *self.current_air_vehicle_state.actual_energy_rate_mut() = energy_rate;
                    }
                },
                MissionRequest(data) => {
                    println!("Got MissionRequest: {:#?}", data);
                    let seq = data.seq; // requested element

                    // respond with MISSION_ITEM
                    let mut mav_msg = mavlink_common::MissionItem::default();
                    mav_msg.target_system = Self::SYSTEM_ID;
                    mav_msg.target_component = Self::COMPONENT_ID;
                    mav_msg.seq = seq;
                    mav_msg.frame = 0; // Global TODO: use enums
                    mav_msg.autocontinue = 1; // Autocontinue to the next waypoint
                    mav_msg.command = 16; // MAV_CMD_NAV_WAYPOINT The scheduled action for the waypoint.
                    mav_msg.current = 1; // false:0, true:1
                    mav_msg.param1 = 0.0; // 	Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
                    mav_msg.param2 = 5.0; // Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
                    mav_msg.param3 = 0.0; // 0 to pass through the WP
                    mav_msg.param4 = NAN; // Desired yaw angle at waypoint (rotary wing). NaN for unchanged.
                    mav_msg.x = self.waypoints[seq as usize].latitude() as f32; // latitude
                    mav_msg.y = self.waypoints[seq as usize].longitude() as f32; // longitude
                    println!("altitude = {}", self.waypoints[seq as usize].altitude() as f32);
                    mav_msg.z = self.waypoints[seq as usize].altitude() as f32; // altitude (relative or absolute, depending on frame).

                    let mut msg = mavlink_common::MavlinkMessage::default();
                    msg.msg_set = Some(mavlink_common::mavlink_message::MsgSet::MissionItem(mav_msg));
                    mavlink_queue.push_back(msg);
                },
                MissionAck(data) => {
                    println!("Got mission ack: {:#?}", data);

                    // send to start
                    let mut mav_msg = mavlink_common::CommandInt::default();
                    mav_msg.target_system = Self::SYSTEM_ID;
                    mav_msg.target_component = Self::COMPONENT_ID;
                    mav_msg.frame = 0; // MAV_MISSION_FRAME ?
                    mav_msg.current = 0;
                    mav_msg.command = 300; // MAV_CMD_MISSION_START
                    mav_msg.autocontinue = 1;
                    mav_msg.param1 = 1.0; // first_item: the first mission item to run
                    mav_msg.param2 = self.waypoints.len() as f32; // last_item: the last mission item to run (after this item is run, the mission ends)
                    mav_msg.param3 = NAN;
                    mav_msg.param4 = NAN;
                    mav_msg.x = 0;
                    mav_msg.y = 0;
                    mav_msg.z = 0.0;

                    let mut msg = mavlink_common::MavlinkMessage::default();
                    msg.msg_set = Some(mavlink_common::mavlink_message::MsgSet::CommandInt(mav_msg));

                    println!("Sending {:#?}", msg);
                    mavlink_queue.push_back(msg);
                },
                MissionItemReached(data) => {
                    println!("Vehicle reached mission seq {}", data.seq);
                }
                MissionCurrent(data) => {
                    println!("Vehicle current mission seq {}", data.seq);
                }
                _ => {},
            }
        }
        (lmcp_queue, mavlink_queue)
    }

    /// Sample configuration message
    /// No payload or gimbal for now
    /// TODO: load this from an XML file
    pub fn create_config_message() -> AirVehicleConfiguration {
        let mut flight_profile = FlightProfile::default();
        *flight_profile.name_mut() = "Cruise".as_bytes().to_vec();
        *flight_profile.airspeed_mut() = 27.5;
        *flight_profile.pitch_angle_mut() = 0.0;
        *flight_profile.vertical_speed_mut() = 0.0;
        *flight_profile.max_bank_angle_mut() = 30.0;
        *flight_profile.energy_rate_mut() = 0.02;

        let mut air_veh_conf = AirVehicleConfiguration::default();
        *air_veh_conf.id_mut() = PixhawkProxy::AC_ID;
        *air_veh_conf.label_mut() = "UAV 400".as_bytes().to_vec();
        *air_veh_conf.minimum_speed_mut() = 15.0;
        *air_veh_conf.maximum_speed_mut() = 35.0;
        *air_veh_conf.nominal_speed_mut() = 27.5;
        *air_veh_conf.minimum_altitude_mut() = 0.0;
        *air_veh_conf.maximum_altitude_mut() = 10000.0;
        *air_veh_conf.nominal_altitude_mut() = 100.0; // should be 700
        *air_veh_conf.nominal_flight_profile_mut() = Box::new(flight_profile);

        air_veh_conf
    }

    /// Attempt to decode a received strem
    /// Expected format is
    /// LmcpSentinelizer:
    /// 	AddressedAttributedMessage:
    ///			LmcpMessage
    ///
    /// TODO: make Sentinel configurable
    pub fn decode_stream(stream: Vec<u8>) -> (Option<LmcpMessage>, Vec<u8>) {
        let (data, rem) = LmcpSentinelizer::parse_stream(stream);
        if let Some(payload) = data {
            if let Some(attr_msg) = AddressedAttributedMessage::deserialize(payload) {
                match LmcpMessage::deser(attr_msg.get_payload()) {
                    Ok(msg) => {
                        // msg is an option alread
                        return (msg, rem);
                    },
                    Err(e) => {
                        println!("PixhawkProxy::decode_message: Error decoding stream: {:?}", e);
                        return (None, rem);
                    },
                }
            }
        }
        (None, rem)
    }

    /// Attemp to encode an LMCP message
    /// Wrap the message with AddressedAttributedMessage
    /// and then LmcpSentinelizer so the resulting format is:
    /// LmcpSentinelizer:
    /// 	AddressedAttributedMessage:
    ///			LmcpMessage
    ///
    /// TODO: make Sentinel/AttributedMessage configurable
    /// Attributed message has extra header in the form of:
    /// `afrl.cmasi.SessionStatus$lmcp|afrl.cmasi.SessionStatus||0|0$`
    pub fn encode_message(msg: LmcpMessage) -> Option<Vec<u8>> {
        let mut buf = vec![0; Self::DEFAULT_SER_BUFFER_SIZE];
        let lmcp_type_name = msg.subscription();
        match msg.ser(buf.as_mut_slice()) {
            Ok(msg_len) => {
                let v: Vec<_> = buf.drain(0..msg_len).collect();
                let mut msg = AddressedAttributedMessage::default();
                msg.set_address(lmcp_type_name); // FULL_LMCP_TYPE_NAME e.g. afrl.cmasi.SessionStatus
                msg.set_content_type("lmcp"); // type of the payload message
                msg.set_descriptor(lmcp_type_name); // FULL_LMCP_TYPE_NAME e.g. afrl.cmasi.SessionStatus
                msg.set_sender_group("TcpBridge"); // TODO: not needed?
                msg.set_sender_entity_id("100"); // TODO: pick some other number?
                msg.set_sender_service_id("64"); // TODO: pick some other number?
                msg.set_payload(v); // serialized LMCP message
                let stream = LmcpSentinelizer::create_sentinelized_stream(&msg.serialize());
                Some(stream)
            },
            Err(e) => {
                println!("PixhawkProxy::encode_message: Error encoding message: {:?}", e);
                None
            },
        }
    }
    
    /// Return afrl::cmasi::session_status::SessionStatus message
    pub fn get_session_status(&self) -> LmcpMessage {
        let msg = SessionStatus::default();
        LmcpMessage::AfrlCmasiSessionStatus(msg)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use lmcp_sentinelizer::LmcpSentinelizer;
    use uxas_attribute_message::AddressedAttributedMessage;

    #[test]
    fn test_deserialize_1() {
        let msg = get_msg_1();
        let (msg1, _) = LmcpSentinelizer::parse_stream(msg);
        if let Some(data) = msg1 {
            let msg2 = AddressedAttributedMessage::deserialize(data).unwrap();
            println!("msg2={}", msg2);
            match LmcpMessage::deser(msg2.get_payload()).unwrap() {
                Some(msg) => {
                    println!("msg3 = {:?}", msg);
                },
                None => panic!("LMCP msg deserialization error"),
            }
        } else {
            panic!("Parsing failed");
        }
    }

    #[test]
    fn test_deserialize_2() {
        let msg = get_msg_2();
        let (msg1, _) = LmcpSentinelizer::parse_stream(msg);
        if let Some(data) = msg1 {
            let msg2 = AddressedAttributedMessage::deserialize(data).unwrap();
            println!("msg2={}", msg2);
            match LmcpMessage::deser(msg2.get_payload()).unwrap() {
                Some(msg) => {
                    println!("msg3 = {:?}", msg);
                },
                None => panic!("LMCP msg deserialization error"),
            }
        } else {
            panic!("Parsing failed");
        }
    }

    #[test]
    fn test_deserialize_3() {
        let msg = get_msg_3();
        let (msg1, _) = LmcpSentinelizer::parse_stream(msg);
        if let Some(data) = msg1 {
            let msg2 = AddressedAttributedMessage::deserialize(data).unwrap();
            println!("msg2={}", msg2);
            match LmcpMessage::deser(msg2.get_payload()).unwrap() {
                Some(msg) => {
                    println!("msg3 = {:?}", msg);
                },
                None => panic!("LMCP msg deserialization error"),
            }
        } else {
            panic!("Parsing failed");
        }
    }

    fn get_msg_1() -> Vec<u8> {
        let v = vec![
            43, 61, 43, 61, 43, 61, 43, 61, 49, 57, 57, 35, 64, 35, 64, 35, 64, 35, 64, 97, 102, 114, 108, 46, 99, 109, 97, 115, 105, 46,
            83, 101, 115, 115, 105, 111, 110, 83, 116, 97, 116, 117, 115, 36, 108, 109, 99, 112, 124, 97, 102, 114, 108, 46, 99, 109, 97,
            115, 105, 46, 83, 101, 115, 115, 105, 111, 110, 83, 116, 97, 116, 117, 115, 124, 124, 48, 124, 48, 36, 76, 77, 67, 80, 0, 0, 0,
            127, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 46, 0, 3, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 115, 63, 128,
            0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 2, 0, 3, 0, 6, 115, 111, 117, 114, 99, 101, 0, 61, 46, 46, 47, 46, 46, 47,
            79, 112, 101, 110, 85, 120, 65, 83, 47, 101, 120, 97, 109, 112, 108, 101, 115, 47, 48, 55, 95, 80, 121, 116, 104, 111, 110, 87,
            105, 116, 104, 65, 109, 97, 115, 101, 47, 83, 99, 101, 110, 97, 114, 105, 111, 95, 69, 109, 112, 116, 121, 46, 120, 109, 108,
            0, 0, 30, 176, 33, 37, 33, 37, 33, 37, 33, 37, 49, 51, 57, 57, 56, 63, 94, 63, 94, 63, 94, 63, 94,
        ];
        v
    }

    fn get_msg_2() -> Vec<u8> {
        let v = vec![
            43, 61, 43, 61, 43, 61, 43, 61, 53, 51, 57, 35, 64, 35, 64, 35, 64, 35, 64, 97, 102, 114, 108, 46, 99, 109, 97, 115, 105, 46,
            65, 105, 114, 86, 101, 104, 105, 99, 108, 101, 83, 116, 97, 116, 101, 36, 108, 109, 99, 112, 124, 97, 102, 114, 108, 46, 99,
            109, 97, 115, 105, 46, 65, 105, 114, 86, 101, 104, 105, 99, 108, 101, 83, 116, 97, 116, 101, 124, 124, 48, 124, 48, 36, 76, 77,
            67, 80, 0, 0, 1, 207, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 15, 0, 3, 0, 0, 0, 0, 0, 0, 0, 3, 65, 160, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 67, 3, 28, 91, 64, 160, 0, 0, 187, 192, 14, 226, 56, 126, 235, 183, 51, 160, 89,
            165, 186, 63, 88, 146, 67, 3, 28, 91, 65, 159, 100, 35, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 63, 248, 11, 15, 140,
            196, 39, 144, 192, 96, 144, 243, 43, 133, 37, 138, 67, 200, 1, 64, 0, 0, 0, 1, 66, 198, 122, 221, 60, 204, 204, 205, 0, 2, 1,
            67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 21, 0, 3, 0, 0, 0, 0, 0, 0, 39, 19, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 194, 52, 0, 0, 0, 0, 0,
            0, 65, 176, 0, 0, 65, 132, 0, 0, 0, 4, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 63, 247, 255, 121, 156, 123, 70, 153,
            192, 96, 144, 200, 62, 40, 33, 57, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 63, 247, 247, 93,
            178, 206, 18, 23, 192, 96, 144, 214, 101, 129, 126, 229, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0,
            3, 63, 247, 255, 143, 243, 234, 84, 221, 192, 96, 144, 228, 92, 174, 13, 168, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0,
            0, 0, 0, 0, 0, 3, 0, 3, 63, 248, 5, 72, 13, 249, 221, 81, 192, 96, 144, 218, 96, 210, 4, 169, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67,
            77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 63, 247, 255, 134, 181, 158, 109, 234, 192, 96, 144, 216, 187, 65, 25, 242, 0, 0, 0,
            0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 27, 0, 3, 0, 0, 0, 0, 0, 0, 3, 235, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 194,
            52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 31, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 174, 165, 0, 0, 65,
            160, 0, 0, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 94, 204, 33, 37, 33, 37, 33, 37, 33, 37, 51, 48, 55, 51, 54, 63, 94, 63,
            94, 63, 94, 63, 94,
        ];
        v
    }

    fn get_msg_3() -> Vec<u8> {
        let v = vec![
            43, 61, 43, 61, 43, 61, 43, 61, 49, 49, 51, 35, 64, 35, 64, 35, 64, 35, 64, 97, 102, 114, 108, 46, 99, 109, 97, 115, 105, 46,
            83, 101, 115, 115, 105, 111, 110, 83, 116, 97, 116, 117, 115, 36, 108, 109, 99, 112, 124, 97, 102, 114, 108, 46, 99, 109, 97,
            115, 105, 46, 83, 101, 115, 115, 105, 111, 110, 83, 116, 97, 116, 117, 115, 124, 124, 48, 124, 48, 36, 76, 77, 67, 80, 0, 0, 0,
            41, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 46, 0, 3, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 107, 18, 63, 128,
            0, 0, 0, 0, 0, 0, 4, 49, 33, 37, 33, 37, 33, 37, 33, 37, 55, 48, 54, 50, 63, 94, 63, 94, 63, 94, 63, 94, 43, 61, 43, 61, 43,
            61, 43, 61, 53, 51, 57, 35, 64, 35, 64, 35, 64, 35, 64, 97, 102, 114, 108, 46, 99, 109, 97, 115, 105, 46, 65, 105, 114, 86,
            101, 104, 105, 99, 108, 101, 83, 116, 97, 116, 101, 36, 108, 109, 99, 112, 124, 97, 102, 114, 108, 46, 99, 109, 97, 115, 105,
            46, 65, 105, 114, 86, 101, 104, 105, 99, 108, 101, 83, 116, 97, 116, 101, 124, 124, 48, 124, 48, 36, 76, 77, 67, 80, 0, 0, 1,
            207, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 15, 0, 3, 0, 0, 0, 0, 0, 0, 1, 144, 65, 176, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 194, 210, 216, 235, 0, 0, 0, 0, 190, 208, 96, 185, 58, 115, 0, 206, 58, 15, 143, 120, 189, 157,
            228, 53, 194, 210, 216, 235, 65, 176, 0, 0, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 168, 57, 200, 123, 188,
            150, 192, 94, 63, 153, 178, 178, 55, 211, 68, 47, 0, 0, 0, 0, 0, 1, 66, 199, 252, 77, 57, 145, 192, 135, 0, 2, 1, 67, 77, 65,
            83, 73, 0, 0, 0, 0, 0, 0, 27, 0, 3, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 194, 52, 0, 0, 0, 0, 0, 0, 1, 67, 77,
            65, 83, 73, 0, 0, 0, 0, 0, 0, 21, 0, 3, 0, 0, 0, 0, 0, 0, 39, 17, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 194, 52, 0, 0, 0, 0, 0, 0, 66,
            52, 0, 0, 66, 7, 0, 0, 0, 4, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 167, 47, 61, 23, 146, 170, 192, 94, 64,
            128, 16, 187, 110, 64, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 168, 124, 21, 109,
            199, 178, 192, 94, 64, 196, 193, 8, 18, 157, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70,
            168, 119, 95, 118, 125, 117, 192, 94, 63, 247, 70, 226, 16, 91, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0,
            0, 3, 0, 3, 64, 70, 167, 197, 0, 27, 82, 77, 192, 94, 63, 211, 196, 163, 25, 3, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73,
            0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 168, 4, 97, 178, 139, 102, 192, 94, 64, 39, 52, 89, 2, 66, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 107, 18, 0, 0, 65, 176, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 82, 202, 33, 37, 33, 37, 33, 37, 33, 37, 50, 55, 54, 52, 56, 63, 94, 63, 94, 63, 94, 63, 94, 43, 61, 43,
            61, 43, 61, 43, 61, 53, 51, 57, 35, 64, 35, 64, 35, 64, 35, 64, 97, 102, 114, 108, 46, 99, 109, 97, 115, 105, 46, 65, 105, 114,
            86, 101, 104, 105, 99, 108, 101, 83, 116, 97, 116, 101, 36, 108, 109, 99, 112, 124, 97, 102, 114, 108, 46, 99, 109, 97, 115,
            105, 46, 65, 105, 114, 86, 101, 104, 105, 99, 108, 101, 83, 116, 97, 116, 101, 124, 124, 48, 124, 48, 36, 76, 77, 67, 80, 0, 0,
            1, 207, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 15, 0, 3, 0, 0, 0, 0, 0, 0, 1, 244, 65, 176, 248, 9, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 194, 17, 155, 241, 0, 0, 0, 0, 65, 159, 248, 110, 184, 45, 25, 63, 65, 48, 94, 16, 65, 242,
            84, 216, 194, 17, 155, 241, 65, 176, 248, 9, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 167, 217, 220, 38, 183,
            77, 192, 94, 60, 65, 148, 145, 240, 162, 68, 122, 0, 0, 0, 0, 0, 1, 66, 199, 252, 80, 57, 145, 192, 135, 0, 2, 1, 67, 77, 65,
            83, 73, 0, 0, 0, 0, 0, 0, 27, 0, 3, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 194, 52, 0, 0, 0, 0, 0, 0, 1, 67, 77,
            65, 83, 73, 0, 0, 0, 0, 0, 0, 21, 0, 3, 0, 0, 0, 0, 0, 0, 39, 17, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 194, 52, 0, 0, 0, 0, 0, 0, 66,
            52, 0, 0, 66, 7, 0, 0, 0, 4, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 169, 82, 66, 255, 236, 45, 192, 94, 62,
            202, 71, 35, 60, 121, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 169, 130, 212, 84, 216,
            223, 192, 94, 60, 193, 101, 123, 83, 70, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 168,
            90, 133, 215, 154, 231, 192, 94, 60, 113, 233, 5, 124, 184, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3,
            0, 3, 64, 70, 167, 209, 27, 172, 141, 152, 192, 94, 61, 57, 213, 71, 213, 242, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73,
            0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 168, 150, 100, 102, 99, 241, 192, 94, 61, 3, 16, 160, 45, 77, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 107, 18, 0, 0, 65, 176, 248, 9, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 89, 71, 33, 37, 33, 37, 33, 37, 33, 37, 50, 57, 49, 56, 53, 63, 94, 63, 94, 63, 94, 63, 94,
        ];
        v
    }
}
