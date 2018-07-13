extern crate prost;
#[macro_use]
extern crate prost_derive;

#[macro_use]
extern crate serde_derive;
extern crate serde;
extern crate serde_json;

use std::io::Cursor;

use prost::Message;

// Include the `items` module, which is generated from items.proto.
pub mod mavlink_common {
    include!(concat!(env!("OUT_DIR"), "/mavlink.common.rs"));
}

fn alt_test() { 
    let mut alt = mavlink_common::Altitude::default();
    alt.time_usec = 53700512; 
    alt.altitude_monotonic=488.23987;
    alt.altitude_amsl = 488.01703;
    alt.altitude_local=0.03985728;
    alt.altitude_relative=0.012936121;
    alt.altitude_terrain= std::f32::NAN;
    alt.bottom_clearance = std::f32::NAN;

    println!("original msg={:?}",alt);

    let stream = serde_json::to_string(&alt).unwrap();
    println!("json msg={}",stream);

    /*
    // Not supported right now
    let new_alt: mavlink_common::Altitude = serde_json::from_str(&stream).unwrap();
    */

    let mut buf = Vec::new();
    buf.reserve(alt.encoded_len());
    alt.encode(&mut buf).unwrap();
    println!("serialized len = {}", buf.len());

    let new_msg = mavlink_common::Altitude::decode(&mut Cursor::new(buf));
    println!("decoded msg = {:?}", new_msg);
}

fn main() {
    //alt_test();
    
    let mut msg = mavlink_common::SysStatus::default();
    msg.onboard_control_sensors_present = 32;
    msg.onboard_control_sensors_enabled = 0;
    msg.onboard_control_sensors_health = 2097184;
    msg.load = 0;
    msg.voltage_battery = 12150;
    msg.current_battery = -100;
    msg.drop_rate_comm = 0;
    msg.errors_comm = 0;
    msg.errors_count1 = 0;
    msg.errors_count2 = 0;
    msg.errors_count3 = 0;
    msg.errors_count4 = 0;
    msg.battery_remaining = 100;
    println!("original msg={:?}\n",msg);

    /*
    let mut mavmsg = mavlink_common::MavlinkMessage::default();
    mavmsg.msg_set = Some(mavlink_common::mavlink_message::MsgSet::Sysstatus(msg));
    
    let stream = serde_json::to_string(&mavmsg).unwrap();
    println!("json msg={}\n",stream);
    
    let mut buf = Vec::new();
    buf.reserve(mavmsg.encoded_len());
    mavmsg.encode(&mut buf).unwrap();
    println!("serialized len = {}\n", buf.len());
    
    let msg2: mavlink_common::MavlinkMessage = serde_json::from_str(&stream).unwrap();
    println!("msg2={:?}\n",msg2);
    */

    let v = serialize_msg(&msg);
    println!("serialized len = {}", v.len());

    let new_msg = deserialize_msg(&v).unwrap();
    println!("decoded msg={:?}",new_msg);

}

pub fn serialize_msg(msg: &mavlink_common::SysStatus) -> Vec<u8> {
    let mut buf = Vec::new();
    buf.reserve(msg.encoded_len());
    // Unwrap is safe, since we have reserved sufficient capacity in the vector.
    msg.encode(&mut buf).unwrap();
    buf
}

pub fn deserialize_msg(buf: &[u8]) -> Result<mavlink_common::SysStatus, prost::DecodeError> {
    mavlink_common::SysStatus::decode(&mut Cursor::new(buf))
}