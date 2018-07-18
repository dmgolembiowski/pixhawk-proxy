// Pixhawk proxy module
use std::time::{Duration, Instant};

use lmcp::afrl::cmasi::air_vehicle_state::AirVehicleState;
use lmcp::afrl::cmasi::air_vehicle_state::AirVehicleStateT;
use lmcp::afrl::cmasi::altitude_type::AltitudeType;
use lmcp::afrl::cmasi::entity_state::EntityStateT;
use lmcp::Message as LmcpMessage;

use std::collections::VecDeque;

// To check ranges for sending bytes to the autopilot (u32/i32 -> u8/i8 conversion)
use range_check::Within;

use super::mavlink_common;

const DEFAULT_KEEP_ALIVE_TIMEOUT: Duration = Duration::from_millis(10_000);

/// Helper struct to keep energy data together
struct BatteryStatus {
    timestamp: Option<Instant>,
    energy_available: Option<f32>,
    energy_rate: Option<f32>,
}

impl BatteryStatus {
    pub fn new() -> BatteryStatus {
        BatteryStatus {
            timestamp: None,
            energy_available: None,
            energy_rate: None,
        }
    }

    /// Update internal values with new data
    pub fn update(&mut self, timestamp: Instant, energy_available: f32) {
        if let Some(last_ts) = self.timestamp {
            let dt = timestamp.duration_since(last_ts);
            if let Some(last_energy) = self.energy_available {
                if last_energy == energy_available {
                    // TODO: set to 0 if no change for a given time
                    return; // no change
                }
                let dt = dt.as_secs() as f32 + dt.subsec_nanos() as f32 * 1e-9;
                let e_rate = (energy_available - last_energy) / dt;
                self.energy_rate = Some(e_rate);
            }
        }
        self.timestamp = Some(timestamp);
        self.energy_available = Some(energy_available);
    }
}

pub struct PixhawkProxy {
    last_heartbeat: Instant,
    last_attitude_since_boot: Duration,
    last_highres_imu_since_boot: Duration,
    pub current_air_vehicle_state: AirVehicleState,
    battery_status: BatteryStatus,
}

impl Default for PixhawkProxy {
    fn default() -> PixhawkProxy {
        PixhawkProxy {
            last_heartbeat: Instant::now() - DEFAULT_KEEP_ALIVE_TIMEOUT,
            last_attitude_since_boot: Duration::new(0, 0),
            last_highres_imu_since_boot: Duration::new(0, 0),
            current_air_vehicle_state: AirVehicleState::default(),
            battery_status: BatteryStatus::new(),
        }
    }
}

impl PixhawkProxy {
    /// handle incoming messages, does nothing for now
    pub fn handle_lmcp_msg(&mut self, _lmcp_msg: LmcpMessage) -> (VecDeque<LmcpMessage>, VecDeque<mavlink_common::MavlinkMessage>) {
        
        (VecDeque::new(), VecDeque::new())
    }
    
    /// Parse an incoming Mavlink proto message and if it contains relevant data,
    /// update the AirVehicleState struct
    pub fn handle_mavlink_msg(&mut self, proto_msg: mavlink_common::MavlinkMessage) -> (VecDeque<LmcpMessage>, VecDeque<mavlink_common::MavlinkMessage>) {
        if let Some(msg) = proto_msg.msg_set {
            use mavlink_common::mavlink_message::MsgSet::*;
            match msg {
                Heartbeat(_data) => {
                    // TODO: check if the heartbeat data are sane
                    self.last_heartbeat = Instant::now();
                    println!("{:#?}", self.current_air_vehicle_state);
                }
                VfrHud(data) => {
                    // vehicle true airspeed [m/s]
                    *self.current_air_vehicle_state.airspeed_mut() = data.airspeed;
                    assert!(data.heading.is_within(-180..180));
                    *self.current_air_vehicle_state.course_mut() = data.heading as f32;
                    *self.current_air_vehicle_state.groundspeed_mut() = data.groundspeed;
                }
                GlobalPositionInt(data) => {
                    // Course/Groundtrack angle of the entity referenced to true North [deg]
                    // The perceived entity location. A valid EntityState must include Location (null not allowed)
                    {
                        *self.current_air_vehicle_state.location.latitude_mut() =
                            data.lat as f64 / 1e7;
                        *self.current_air_vehicle_state.location.longitude_mut() =
                            data.lon as f64 / 1e7;
                        // TODO: fix WGS84 to AMSL projection
                        *self.current_air_vehicle_state.location.altitude_mut() =
                            data.alt as f32 / 1000.0;
                        // TODO: fix the altitude type (maybe use relative altitude?)
                        *self.current_air_vehicle_state.location.altitude_type_mut() =
                            AltitudeType::MSL;
                    }
                    // Vertical speed (positive upwards) of the vehicle in the inertial frame (rate of change of altitude) [m/s]
                    *self.current_air_vehicle_state.vertical_speed_mut() = data.vz as f32 / -100.0;
                }
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
                }
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
                }
                BatteryStatus(data) => {
                    // TODO: check for -1
                    self.battery_status
                        .update(Instant::now(), data.battery_remaining as f32);
                    if let Some(energy_available) = self.battery_status.energy_available {
                        *self.current_air_vehicle_state.energy_available_mut() = energy_available;
                    }
                    if let Some(energy_rate) = self.battery_status.energy_rate {
                        *self.current_air_vehicle_state.actual_energy_rate_mut() = energy_rate;
                    }
                }
                _ => {}
            }
        }
        (VecDeque::new(), VecDeque::new())
    }
}
