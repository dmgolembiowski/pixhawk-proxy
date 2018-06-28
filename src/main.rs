extern crate prost;
#[macro_use]
extern crate prost_derive;

use std::io::Cursor;

use prost::Message;

// Include the `items` module, which is generated from items.proto.
pub mod mavlink_common {
    include!(concat!(env!("OUT_DIR"), "/mavlink.common.rs"));
}

fn main() {
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
    println!("original msg={:?}",msg);
    
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