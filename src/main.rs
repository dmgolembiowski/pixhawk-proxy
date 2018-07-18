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

use prost::Message;
use std::collections::VecDeque;
use std::io::Cursor;
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
    handles.push(
        thread::Builder::new()
            .name("th_lmcp_rx".to_string())
            .spawn({
                let _proxy = proxy.clone();
                let send_to_lmcp = Sender::clone(&lmcp_tx);
                let send_to_mavlink = Sender::clone(&mavlink_tx);
                let subscriber = context.socket(zmq::SUB).unwrap();
                let filter = "";
                match subscriber.connect("tcp://127.0.0.1:55560") {
                    Ok(_) => {}
                    Err(e) => {
                        println!("Error: {}", e);
                    }
                }
                assert!(subscriber.set_subscribe(filter.as_bytes()).is_ok());

                move || loop {
                    let stream = subscriber.recv_bytes(0).unwrap();
                    println!("Received {} bytes", stream.len());
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
                }
            }),
    );

    // TX thread: Sending AirVehicle and other LMCP messages
    handles.push(
        thread::Builder::new()
            .name("th_lmcp_tx".to_string())
            .spawn({
                let publisher = context.socket(zmq::PUB).unwrap();
                match publisher.bind("tcp://127.0.0.1:55561") {
                    Ok(_) => {}
                    Err(e) => {
                        println!("Error: {}", e);
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
