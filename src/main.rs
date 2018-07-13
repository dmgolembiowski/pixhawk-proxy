extern crate prost;
#[macro_use]
extern crate prost_derive;

#[macro_use]
extern crate serde_derive;
extern crate serde;
extern crate serde_json;

extern crate zmq;
extern crate range_check;
extern crate lmcp;

// To check ranges for sending bytes to the autopilot (u32/i32 -> u8/i8 conversion)
use range_check::Within;

use prost::Message;
use std::io::Cursor;

use lmcp::afrl::cmasi::air_vehicle_state::AirVehicleState;
use lmcp::afrl::cmasi::air_vehicle_state::AirVehicleStateT;
use lmcp::afrl::cmasi::entity_state::EntityStateT;

// Include the `items` module, which is generated from items.proto.
pub mod mavlink_common {
    include!(concat!(env!("OUT_DIR"), "/mavlink.common.rs"));
}

pub struct PixhawkProxy {
    current_air_vehicle_state: AirVehicleState,
}

impl Default for PixhawkProxy {
    fn default() -> PixhawkProxy {
        PixhawkProxy {
            current_air_vehicle_state: AirVehicleState::default(),
        }
    }
}

impl PixhawkProxy {
    /// Parse an incoming Mavlink proto message and if it contains relevant data,
    /// update the AirVehicleState struct
    pub fn handle_proto_msg(&mut self, proto_msg: mavlink_common::MavlinkMessage) {
        if let Some(msg) = proto_msg.msg_set {
            use mavlink_common::mavlink_message::MsgSet::*;
            match msg {
                Heartbeat(data) => {
                    // debug only
                    println!("{:#?}", data);
                    println!("{:#?}", self.current_air_vehicle_state);
                }
                VfrHud(data) => {
                    *self.current_air_vehicle_state.airspeed_mut() = data.airspeed;
                    // TODO: Check range?
                    // [dependencies]
                    // range_check = "0.1"
                    // rust-range-check
                    // use range_check::Within;
                    // assert!(4123.is_within(3000..5000));
                    //println!("Data heading = {}", data.heading);
                    assert!(data.heading.is_within(0..360)); // heading is -1, so probably do some check around this
                    *self.current_air_vehicle_state.course_mut() = data.heading as f32;
                    *self.current_air_vehicle_state.groundspeed_mut() = data.groundspeed; // TODO: fix with GPS data
                }
                GlobalPositionInt(data) => {
                    *self.current_air_vehicle_state.location.latitude_mut() = data.lat as f64 / 1e7;
                    *self.current_air_vehicle_state.location.longitude_mut() =
                        data.lon as f64 / 1e7;
                    *self.current_air_vehicle_state.location.altitude_mut() =
                        data.alt as f32 / 1000.0; // TODO: fix WGS84 to AMSL projection
                                                  // *proxy.current_air_vehicle_state.location.altitude_type_mut() = AltitudeType::MSL ?
                    *self.current_air_vehicle_state.vertical_speed_mut() = data.vz as f32 / -100.0; // TODO: check values
                     /*
                     Ground speed from NED: 
                        let v_north_m_s: f32 = gps_data.v_north as f32 / 100.0;
                        let v_east_m_s = gps_data.v_east as f32 / 100.0;
                        let v_down_m_s = gps_data.v_down as f32 / 100.0;
                        self.current_air_vehicle_state.course = v_east_m_s.atan2(v_north_m_s).to_degrees();
                     */                }
                HighresImu(data) => {
                    *self.current_air_vehicle_state.udot_mut() = data.xacc;
                    *self.current_air_vehicle_state.vdot_mut() = data.yacc;
                    *self.current_air_vehicle_state.wdot_mut() = data.zacc;
                }
                Attitude(data) => {
                    // TODO: yaw? Heading?
                    *self.current_air_vehicle_state.heading_mut() = data.yaw.to_degrees() + 180.0;
                    *self.current_air_vehicle_state.pitch_mut() = data.pitch.to_degrees();
                    *self.current_air_vehicle_state.roll_mut() = data.roll.to_degrees();
                    *self.current_air_vehicle_state.p_mut() = data.rollspeed.to_degrees();
                    *self.current_air_vehicle_state.q_mut() = data.pitchspeed.to_degrees();
                    *self.current_air_vehicle_state.r_mut() = data.yawspeed.to_degrees();
                }
                BatteryStatus(data) => {
                    *self.current_air_vehicle_state.energy_available_mut() =
                        data.battery_remaining as f32; // TODO: check for -1
                    //*proxy.current_air_vehicle_state.actual_energy_rate_mut() = TODO: estimate the current energy rate from msg timestamp
                }
                _ => {}
            }
        }
    }
}

fn main() {
    let context = zmq::Context::new();
    let subscriber = context.socket(zmq::SUB).unwrap();
    let filter = "";
    assert!(subscriber.connect("tcp://localhost:5556").is_ok());
    assert!(subscriber.set_subscribe(filter.as_bytes()).is_ok());

    let mut proxy = PixhawkProxy::default();

    loop {
        let stream = subscriber.recv_bytes(0).unwrap();
        let msg = mavlink_common::MavlinkMessage::decode(&mut Cursor::new(stream)).unwrap();
        proxy.handle_proto_msg(msg);
    }
}
