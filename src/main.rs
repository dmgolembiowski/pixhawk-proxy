/// Main file for Pixhawk proxy
///
/// We have 4 threads (all use ZMQ for external message passing, and `sync::mpsc::channel` for internal message passing):
/// 	LMCP TX: sends LMCP messages (mainly air_vehicle_state) to `LmcpObjectNetworkPublishPullBridge` when a new message is available
///		LMCP RX: receives LMCP messages and processes them when appropriate (mostly waypoints)
///		MAVLINK RX: receives Mavlink_protobuf messages and updates `air_vehicle_state` struct
///		MAVLINK TX: sends Mavlink_protobuf messages that were created from incoming LMCP messages (such as mission and waypoint updates)
///
extern crate prost;
#[macro_use]
extern crate prost_derive;

#[macro_use]
extern crate serde_derive;
extern crate serde;
extern crate serde_json;

extern crate lmcp;
extern crate range_check;
extern crate zmq;

// our internal crates
extern crate lmcp_sentinelizer;
extern crate uxas_attribute_message;

use lmcp_sentinelizer::LmcpSentinelizer;
use uxas_attribute_message::AddressedAttributedMessage;

use prost::Message;
use std::collections::VecDeque;
use std::io::Cursor;
use std::process::exit;
use std::sync::mpsc::{channel, Sender};
use std::sync::{Arc, Mutex};
use std::thread;

// Include the `items` module, which is generated from items.proto.
pub mod mavlink_common {
    include!(concat!(env!("OUT_DIR"), "/mavlink.common.rs"));
}

use lmcp::Message as LmcpMessage;

mod pixhawk_proxy;
use pixhawk_proxy::PixhawkProxy;

fn main() {
    // Default ZMQ context
    let context = zmq::Context::new();

    // Define channels
    let (lmcp_tx, lmcp_rx) = channel();
    let (mavlink_tx, mavlink_rx) = channel();

    // create proxy
    let proxy = Arc::new(Mutex::new(PixhawkProxy::default()));

    // keep thread handles
    let mut handles = vec![];

    // RX thread: receive and process Mavlink messages
    handles.push(
        thread::Builder::new()
            .name("th_mavlink_rx".to_string())
            .spawn({
                let proxy = proxy.clone();
                let send_to_lmcp = Sender::clone(&lmcp_tx);
                let send_to_mavlink = Sender::clone(&mavlink_tx);
                let subscriber = context.socket(zmq::SUB).unwrap();
                let filter = "";
                match subscriber.connect("tcp://127.0.0.1:44440") {
                    Ok(_) => {}
                    Err(e) => {
                        println!("Error: {}", e);
                        exit(1);
                    }
                }
                assert!(subscriber.set_subscribe(filter.as_bytes()).is_ok());

                move || loop {
                    let stream = subscriber.recv_bytes(0).unwrap();
                    let msg =
                        mavlink_common::MavlinkMessage::decode(&mut Cursor::new(stream)).unwrap();
                    let mut lmcp_msgs_to_send = VecDeque::new();
                    let mut mavlink_msgs_to_send = VecDeque::new();
                    match proxy.lock() {
                        Ok(mut mav_proxy) => {
                            let (mut lmcp_msgs, mut mavlink_msgs) =
                                mav_proxy.handle_mavlink_msg(msg);
                            lmcp_msgs_to_send.append(&mut lmcp_msgs);
                            mavlink_msgs_to_send.append(&mut mavlink_msgs);
                        }
                        Err(e) => {
                            println!("Error locking proxy: {}", e);
                        }
                    }
                    for mavlink_msg in mavlink_msgs_to_send {
                        send_to_mavlink.send(mavlink_msg).unwrap();
                    }
                    for lmcp_msg in lmcp_msgs_to_send {
                        send_to_lmcp.send(lmcp_msg).unwrap();
                    }
                }
            }),
    );

    // RX thread: receive LMCP messages
    // 55560 is used by the UxAS bridge
    // 5555 is used by the python example
    handles.push(
        thread::Builder::new()
            .name("th_lmcp_rx".to_string())
            .spawn({
                let _proxy = proxy.clone();
                let send_to_lmcp = Sender::clone(&lmcp_tx);
                let send_to_mavlink = Sender::clone(&mavlink_tx);
                //let subscriber = context.socket(zmq::SUB).unwrap();
                let subscriber = context.socket(zmq::STREAM).unwrap();
                let filter = "";
                match subscriber.connect("tcp://127.0.0.1:5555") {
                    Ok(_) => {}
                    Err(e) => {
                        println!("Error: {}", e);
                        exit(1);
                    }
                } /*
                match subscriber.set_subscribe(filter.as_bytes()) {
                    Ok(_) => {}
                    Err(e) => {
                        println!("Error: {}", e);
                        exit(1);
                    }
                }
                */

                move || loop {
                    let mut mpart = subscriber.recv_multipart(0).unwrap();
                    println!("Received {} msgs", mpart.len());
                    println!("mpart = {:?}", mpart);
                    assert_eq!(mpart.len(), 2); // assert we have only 2 messages
                    let stream = mpart.pop().unwrap(); // ~= mpart[1]
                    println!("stream len = {}", stream.len());
                    if stream.len() == 0 {
                        continue;
                    }

                    // denetinelize
                    let msg1 = LmcpSentinelizer::parse_sentinelized_stream(stream).unwrap();
                    let msg2 = AddressedAttributedMessage::deserialize(msg1).unwrap();
                    println!("attribute msg = {}", msg2);
                    let msg3 = LmcpMessage::deser(msg2.get_payload()).unwrap().unwrap();
                    println!("Received msg: {:#?}", msg3);

                    /*
                    let stream = subscriber.recv_bytes(0).unwrap();
                    println!("Received {} bytes", stream.len());
                    if stream.len() <= 5 {
                        continue;
                    }
                    if let Some(msg) = LmcpMessage::deser(&stream).unwrap() {
                        let mut lmcp_msgs_to_send = VecDeque::new();
                        let mut mavlink_msgs_to_send = VecDeque::new();
                        match proxy.lock() {
                            Ok(mut mav_proxy) => {
                                let (mut lmcp_msgs, mut mavlink_msgs) =
                                    mav_proxy.handle_lmcp_msg(msg);
                                lmcp_msgs_to_send.append(&mut lmcp_msgs);
                                mavlink_msgs_to_send.append(&mut mavlink_msgs);
                            }
                            Err(e) => {
                                println!("Error locking proxy: {}", e);
                            }
                        }
                        for mavlink_msg in mavlink_msgs_to_send {
                            send_to_mavlink.send(mavlink_msg).unwrap();
                        }
                        for lmcp_msg in lmcp_msgs_to_send {
                            send_to_lmcp.send(lmcp_msg).unwrap();
                        }
                    }
                    */
                }
            }),
    );

    // TX thread: Sending AirVehicle and other LMCP messages
    handles.push(
        thread::Builder::new()
            .name("th_lmcp_tx".to_string())
            .spawn({
                let publisher = context.socket(zmq::PUB).unwrap();
                //let publisher = context.socket(zmq::STREAM).unwrap();
                match publisher.bind("tcp://127.0.0.1:55561") {
                    //match publisher.bind("tcp://127.0.0.1:5555") {
                    Ok(_) => {}
                    Err(e) => {
                        println!("Error: {}", e);
                        exit(1);
                    }
                }

                // allocate buffer
                let mut buf = vec![0; 1024];

                move || loop {
                    match lmcp_rx.recv() {
                        Ok(msg) => {
                            // serialize message and send data
                            let msg_len = msg.ser(buf.as_mut_slice()).unwrap();
                            publisher.send(&buf[0..msg_len], 0).unwrap(); // send buffer with 0 flags
                        }
                        Err(e) => {
                            println!("Error occured: {}", e);
                        }
                    }
                }
            }),
    );

    // TX thread: Sending Mavlink proto messages
    handles.push(
        thread::Builder::new()
            .name("th_mavlink_tx".to_string())
            .spawn({
                let publisher = context.socket(zmq::PUB).unwrap();
                match publisher.bind("tcp://127.0.0.1:44441") {
                    Ok(_) => {}
                    Err(e) => {
                        println!("Error: {}", e);
                        exit(1);
                    }
                }

                move || loop {
                    match mavlink_rx.recv() {
                        Ok(msg) => {
                            // encode message into a buffer and send data
                            let mut buf = Vec::new();
                            buf.reserve(msg.encoded_len());
                            msg.encode(&mut buf).unwrap();
                            publisher.send(&buf, 0).unwrap(); // send buffer with 0 flags
                        }
                        Err(e) => {
                            println!("Error occured: {}", e);
                        }
                    }
                }
            }),
    );

    // wait for threads to finish
    for handle in handles {
        handle.unwrap().join().unwrap();
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_deserialize_1() {
        let msg = get_msg_1();
        let msg1 = LmcpSentinelizer::parse_sentinelized_stream(msg).unwrap();
        let msg2 = AddressedAttributedMessage::deserialize(msg1).unwrap();
        println!("msg2={}", msg2);
        match LmcpMessage::deser(msg2.get_payload()).unwrap() {
            Some(msg) => {
                println!("msg3 = {:?}", msg);
            }
            None => panic!("LMCP msg deserialization error"),
        }
    }

    #[test]
    fn test_deserialize_2() {
        let msg = get_msg_2();
        let msg1 = LmcpSentinelizer::parse_sentinelized_stream(msg).unwrap();
        let msg2 = AddressedAttributedMessage::deserialize(msg1).unwrap();
        println!("msg2={}", msg2);
        match LmcpMessage::deser(msg2.get_payload()).unwrap() {
            Some(msg) => {
                println!("msg3 = {:?}", msg);
            }
            None => panic!("LMCP msg deserialization error"),
        }
    }

    #[test]
    fn test_deserialize_3() {
        let msg = get_msg_3();
        let msg1 = LmcpSentinelizer::parse_sentinelized_stream(msg).unwrap();
        let msg2 = AddressedAttributedMessage::deserialize(msg1).unwrap();
        println!("msg2={}", msg2);
        match LmcpMessage::deser(msg2.get_payload()).unwrap() {
            Some(msg) => {
                println!("msg3 = {:?}", msg);
            }
            None => panic!("LMCP msg deserialization error"),
        }
    }
    
    #[test]
    fn test_serialize_1() {
        panic!("TODO: not implemented");
    }

    fn get_msg_1() -> Vec<u8> {
        let v = vec![
            43, 61, 43, 61, 43, 61, 43, 61, 49, 57, 57, 35, 64, 35, 64, 35, 64, 35, 64, 97, 102,
            114, 108, 46, 99, 109, 97, 115, 105, 46, 83, 101, 115, 115, 105, 111, 110, 83, 116, 97,
            116, 117, 115, 36, 108, 109, 99, 112, 124, 97, 102, 114, 108, 46, 99, 109, 97, 115,
            105, 46, 83, 101, 115, 115, 105, 111, 110, 83, 116, 97, 116, 117, 115, 124, 124, 48,
            124, 48, 36, 76, 77, 67, 80, 0, 0, 0, 127, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 46,
            0, 3, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 115, 63, 128, 0, 0, 0,
            1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 2, 0, 3, 0, 6, 115, 111, 117, 114, 99, 101,
            0, 61, 46, 46, 47, 46, 46, 47, 79, 112, 101, 110, 85, 120, 65, 83, 47, 101, 120, 97,
            109, 112, 108, 101, 115, 47, 48, 55, 95, 80, 121, 116, 104, 111, 110, 87, 105, 116,
            104, 65, 109, 97, 115, 101, 47, 83, 99, 101, 110, 97, 114, 105, 111, 95, 69, 109, 112,
            116, 121, 46, 120, 109, 108, 0, 0, 30, 176, 33, 37, 33, 37, 33, 37, 33, 37, 49, 51, 57,
            57, 56, 63, 94, 63, 94, 63, 94, 63, 94,
        ];
        v
    }

    fn get_msg_2() -> Vec<u8> {
        let v = vec![
            43, 61, 43, 61, 43, 61, 43, 61, 53, 51, 57, 35, 64, 35, 64, 35, 64, 35, 64, 97, 102,
            114, 108, 46, 99, 109, 97, 115, 105, 46, 65, 105, 114, 86, 101, 104, 105, 99, 108, 101,
            83, 116, 97, 116, 101, 36, 108, 109, 99, 112, 124, 97, 102, 114, 108, 46, 99, 109, 97,
            115, 105, 46, 65, 105, 114, 86, 101, 104, 105, 99, 108, 101, 83, 116, 97, 116, 101,
            124, 124, 48, 124, 48, 36, 76, 77, 67, 80, 0, 0, 1, 207, 1, 67, 77, 65, 83, 73, 0, 0,
            0, 0, 0, 0, 15, 0, 3, 0, 0, 0, 0, 0, 0, 0, 3, 65, 160, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 67, 3, 28, 91, 64, 160, 0, 0, 187, 192, 14, 226, 56,
            126, 235, 183, 51, 160, 89, 165, 186, 63, 88, 146, 67, 3, 28, 91, 65, 159, 100, 35, 1,
            67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 63, 248, 11, 15, 140, 196, 39, 144, 192,
            96, 144, 243, 43, 133, 37, 138, 67, 200, 1, 64, 0, 0, 0, 1, 66, 198, 122, 221, 60, 204,
            204, 205, 0, 2, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 21, 0, 3, 0, 0, 0, 0, 0, 0,
            39, 19, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 194, 52, 0, 0, 0, 0, 0, 0, 65, 176, 0, 0, 65,
            132, 0, 0, 0, 4, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 63, 247, 255, 121,
            156, 123, 70, 153, 192, 96, 144, 200, 62, 40, 33, 57, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67,
            77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 63, 247, 247, 93, 178, 206, 18, 23, 192, 96,
            144, 214, 101, 129, 126, 229, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0,
            0, 0, 0, 3, 0, 3, 63, 247, 255, 143, 243, 234, 84, 221, 192, 96, 144, 228, 92, 174, 13,
            168, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 63, 248,
            5, 72, 13, 249, 221, 81, 192, 96, 144, 218, 96, 210, 4, 169, 0, 0, 0, 0, 0, 0, 0, 1, 1,
            67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 63, 247, 255, 134, 181, 158, 109, 234,
            192, 96, 144, 216, 187, 65, 25, 242, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0,
            0, 0, 0, 0, 0, 27, 0, 3, 0, 0, 0, 0, 0, 0, 3, 235, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 194,
            52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 31, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 174, 165, 0, 0, 65, 160, 0, 0, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 94, 204, 33, 37, 33, 37, 33, 37, 33, 37, 51, 48, 55, 51, 54, 63, 94, 63, 94,
            63, 94, 63, 94,
        ];
        v
    }

    fn get_msg_3() -> Vec<u8> {
        let v = vec![
            43, 61, 43, 61, 43, 61, 43, 61, 49, 49, 51, 35, 64, 35, 64, 35, 64, 35, 64, 97, 102,
            114, 108, 46, 99, 109, 97, 115, 105, 46, 83, 101, 115, 115, 105, 111, 110, 83, 116, 97,
            116, 117, 115, 36, 108, 109, 99, 112, 124, 97, 102, 114, 108, 46, 99, 109, 97, 115,
            105, 46, 83, 101, 115, 115, 105, 111, 110, 83, 116, 97, 116, 117, 115, 124, 124, 48,
            124, 48, 36, 76, 77, 67, 80, 0, 0, 0, 41, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 46,
            0, 3, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 107, 18, 63, 128, 0, 0, 0,
            0, 0, 0, 4, 49, 33, 37, 33, 37, 33, 37, 33, 37, 55, 48, 54, 50, 63, 94, 63, 94, 63, 94,
            63, 94, 43, 61, 43, 61, 43, 61, 43, 61, 53, 51, 57, 35, 64, 35, 64, 35, 64, 35, 64, 97,
            102, 114, 108, 46, 99, 109, 97, 115, 105, 46, 65, 105, 114, 86, 101, 104, 105, 99, 108,
            101, 83, 116, 97, 116, 101, 36, 108, 109, 99, 112, 124, 97, 102, 114, 108, 46, 99, 109,
            97, 115, 105, 46, 65, 105, 114, 86, 101, 104, 105, 99, 108, 101, 83, 116, 97, 116, 101,
            124, 124, 48, 124, 48, 36, 76, 77, 67, 80, 0, 0, 1, 207, 1, 67, 77, 65, 83, 73, 0, 0,
            0, 0, 0, 0, 15, 0, 3, 0, 0, 0, 0, 0, 0, 1, 144, 65, 176, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 194, 210, 216, 235, 0, 0, 0, 0, 190, 208, 96, 185,
            58, 115, 0, 206, 58, 15, 143, 120, 189, 157, 228, 53, 194, 210, 216, 235, 65, 176, 0,
            0, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 168, 57, 200, 123, 188,
            150, 192, 94, 63, 153, 178, 178, 55, 211, 68, 47, 0, 0, 0, 0, 0, 1, 66, 199, 252, 77,
            57, 145, 192, 135, 0, 2, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 27, 0, 3, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 194, 52, 0, 0, 0, 0, 0, 0, 1, 67, 77, 65, 83,
            73, 0, 0, 0, 0, 0, 0, 21, 0, 3, 0, 0, 0, 0, 0, 0, 39, 17, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            194, 52, 0, 0, 0, 0, 0, 0, 66, 52, 0, 0, 66, 7, 0, 0, 0, 4, 1, 67, 77, 65, 83, 73, 0,
            0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 167, 47, 61, 23, 146, 170, 192, 94, 64, 128, 16, 187,
            110, 64, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64,
            70, 168, 124, 21, 109, 199, 178, 192, 94, 64, 196, 193, 8, 18, 157, 0, 0, 0, 0, 0, 0,
            0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 168, 119, 95, 118, 125,
            117, 192, 94, 63, 247, 70, 226, 16, 91, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73,
            0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 167, 197, 0, 27, 82, 77, 192, 94, 63, 211, 196, 163,
            25, 3, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64,
            70, 168, 4, 97, 178, 139, 102, 192, 94, 64, 39, 52, 89, 2, 66, 0, 0, 0, 0, 0, 0, 0, 1,
            0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            107, 18, 0, 0, 65, 176, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 82, 202, 33,
            37, 33, 37, 33, 37, 33, 37, 50, 55, 54, 52, 56, 63, 94, 63, 94, 63, 94, 63, 94, 43, 61,
            43, 61, 43, 61, 43, 61, 53, 51, 57, 35, 64, 35, 64, 35, 64, 35, 64, 97, 102, 114, 108,
            46, 99, 109, 97, 115, 105, 46, 65, 105, 114, 86, 101, 104, 105, 99, 108, 101, 83, 116,
            97, 116, 101, 36, 108, 109, 99, 112, 124, 97, 102, 114, 108, 46, 99, 109, 97, 115, 105,
            46, 65, 105, 114, 86, 101, 104, 105, 99, 108, 101, 83, 116, 97, 116, 101, 124, 124, 48,
            124, 48, 36, 76, 77, 67, 80, 0, 0, 1, 207, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 15,
            0, 3, 0, 0, 0, 0, 0, 0, 1, 244, 65, 176, 248, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 194, 17, 155, 241, 0, 0, 0, 0, 65, 159, 248, 110, 184, 45, 25, 63,
            65, 48, 94, 16, 65, 242, 84, 216, 194, 17, 155, 241, 65, 176, 248, 9, 1, 67, 77, 65,
            83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 167, 217, 220, 38, 183, 77, 192, 94, 60, 65,
            148, 145, 240, 162, 68, 122, 0, 0, 0, 0, 0, 1, 66, 199, 252, 80, 57, 145, 192, 135, 0,
            2, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 27, 0, 3, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 194, 52, 0, 0, 0, 0, 0, 0, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0,
            0, 21, 0, 3, 0, 0, 0, 0, 0, 0, 39, 17, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 194, 52, 0, 0, 0,
            0, 0, 0, 66, 52, 0, 0, 66, 7, 0, 0, 0, 4, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3,
            0, 3, 64, 70, 169, 82, 66, 255, 236, 45, 192, 94, 62, 202, 71, 35, 60, 121, 0, 0, 0, 0,
            0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 169, 130, 212,
            84, 216, 223, 192, 94, 60, 193, 101, 123, 83, 70, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77,
            65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 168, 90, 133, 215, 154, 231, 192, 94,
            60, 113, 233, 5, 124, 184, 0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0,
            0, 0, 3, 0, 3, 64, 70, 167, 209, 27, 172, 141, 152, 192, 94, 61, 57, 213, 71, 213, 242,
            0, 0, 0, 0, 0, 0, 0, 1, 1, 67, 77, 65, 83, 73, 0, 0, 0, 0, 0, 0, 3, 0, 3, 64, 70, 168,
            150, 100, 102, 99, 241, 192, 94, 61, 3, 16, 160, 45, 77, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 107,
            18, 0, 0, 65, 176, 248, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 89, 71, 33, 37,
            33, 37, 33, 37, 33, 37, 50, 57, 49, 56, 53, 63, 94, 63, 94, 63, 94, 63, 94,
        ];
        v
    }
}
