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

#[macro_use]
extern crate clap;

// our internal crates
extern crate lmcp_sentinelizer;
extern crate uxas_attribute_message;

use prost::Message;

use std::collections::VecDeque;
use std::io::Cursor;
use std::process::exit;
use std::sync::mpsc::{channel, Sender};
use std::sync::{Arc, Mutex};
use std::thread;

use clap::App;

// Include the `items` module, which is generated from items.proto.
pub mod mavlink_common {
    include!(concat!(env!("OUT_DIR"), "/mavlink.common.rs"));
}

mod pixhawk_proxy;
use pixhawk_proxy::PixhawkProxy;

/// Run for example with `cargo run -- -s tcp://127.0.0.1:4440 tcp://127.0.0.1:44440 tcp://127.0.0.1:5555 tcp://127.0.0.1:5555`
fn main() {
    let yaml = load_yaml!("../cli.yml");
    let matches = App::from_yaml(yaml).get_matches();

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
                let addr = matches.value_of("MAVLINK_SUB").unwrap().clone();
                match subscriber.bind(addr) {
                    Ok(_) => {
                        println!("Mavlink Subscriber: bound to {}", addr);
                    }
                    Err(e) => {
                        println!("Mavlink Subscriber error: {} connecting to {}", e, addr);
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

    // TX thread: Sending Mavlink proto messages
    handles.push(
        thread::Builder::new()
            .name("th_mavlink_tx".to_string())
            .spawn({
                let publisher = context.socket(zmq::PUB).unwrap();
                let addr = matches.value_of("MAVLINK_PUB").unwrap().clone();
                match publisher.bind(addr) {
                    Ok(_) => {
                        println!("Mavlink Publisher: bound to {}", addr);
                    }
                    Err(e) => {
                        println!("Mavlink Publisher error: {} connecting to {}", e, addr);
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

    // TX thread: Sending AirVehicle and other LMCP messages
    // spawn only if not using ZMQ_STREAM
    if !matches.is_present("stream") {
        handles.push(
            thread::Builder::new()
                .name("th_lmcp_tx".to_string())
                .spawn({
                    let publisher = context.socket(zmq::PUB).unwrap();
                    let addr = matches.value_of("LMCP_PUB").unwrap().clone();
                    match publisher.bind(addr) {
                        Ok(_) => {
                            println!("LMCP Publisher: bound to {}", addr);
                        }
                        Err(e) => {
                            println!("LMCP Publisher error: {} connecting to {}", e, addr);
                            exit(1);
                        }
                    }

                    move || loop {
                        match lmcp_rx.recv() {
                            Ok(msg) => {
                                // serialize message and send data
                                if let Some(stream) = PixhawkProxy::encode_message(msg) {
                                    publisher.send(&stream, 0).unwrap(); // send buffer with 0 flags
                                }
                            }
                            Err(e) => {
                                println!("Error occured: {}", e);
                            }
                        }
                    }
                }),
        );
    } else {
        println!("With ZMQ::STREAM Rx and Tx is handled with a single socket");
    }

    // RX thread: receive LMCP messages
    // 55560 is used by the UxAS bridge
    // 5555 is used by the python example
    if matches.is_present("stream") {
        // Use streamed version
        handles.push(
            thread::Builder::new()
                .name("th_lmcp_rx".to_string())
                .spawn({
                    let debug = matches.is_present("debug");
                    let send_to_mavlink = Sender::clone(&mavlink_tx);
                    println!("Connecting as zmq::STREAM");
                    let socket = context.socket(zmq::STREAM).unwrap();

                    let addr = matches.value_of("LMCP_SUB").unwrap().clone();
                    match socket.connect(addr) {
                        Ok(_) => {
                            println!("LMCP Subscriber: connected to {}", addr);
                        }
                        Err(e) => {
                            println!("LMCP Subscriber error: {} connecting to {}", e, addr);
                            exit(1);
                        }
                    }

                    let mut client_id = None;
                    let mut lmcp_msgs_to_send = VecDeque::new();
                    let mut mavlink_msgs_to_send = VecDeque::new();
                    move || loop {
                        let mut mpart = socket.recv_multipart(0).unwrap();
                        assert_eq!(mpart.len(), 2); // assert we have only 2 messages
                        let stream = mpart.pop().unwrap(); // ~= mpart[1]
                        if stream.len() == 0 {
                            continue;
                        }
                        if client_id == None {
                            client_id = Some(mpart.pop().unwrap());
                        }

                        if debug {
                            println!("Received {} bytes", stream.len());
                        }

                        if let Some(msg) = PixhawkProxy::decode_stream(stream) {
                            if debug {
                                println!("Received msg: {:#?}", msg);
                            }
                            match proxy.lock() {
                                Ok(mut mav_proxy) => {
                                    let (mut lmcp_msgs, mut mavlink_msgs) =
                                        mav_proxy.handle_lmcp_msg(msg);
                                    lmcp_msgs_to_send.append(&mut lmcp_msgs);
                                    mavlink_msgs_to_send.append(&mut mavlink_msgs);
                                }
                                Err(e) => {
                                    println!("Error locking PixhawkProxy: {}", e);
                                }
                            }

                            while !mavlink_msgs_to_send.is_empty() {
                                let mavlink_msg = mavlink_msgs_to_send.pop_front().unwrap();
                                send_to_mavlink.send(mavlink_msg).unwrap();
                            }

                            while !lmcp_msgs_to_send.is_empty() {
                                let lmcp_msg = lmcp_msgs_to_send.pop_front().unwrap();
                                if let Some(stream) = PixhawkProxy::encode_message(lmcp_msg) {
                                    if let Some(ref client) = client_id {
                                        socket.send_multipart(&[&client, &stream], 0).unwrap(); // send buffer with 0 flags
                                    }
                                }
                            }
                        }
                    }
                }),
        );
    } else {
        // Use SUBscriber
        handles.push(
            thread::Builder::new()
                .name("th_lmcp_rx".to_string())
                .spawn({
                    let debug = matches.is_present("debug");
                    let send_to_lmcp = Sender::clone(&lmcp_tx);
                    let send_to_mavlink = Sender::clone(&mavlink_tx);
                    println!("Connecting as zmq::SUB");
                    let subscriber = context.socket(zmq::SUB).unwrap();

                    let filter = "";
                    let addr = matches.value_of("LMCP_SUB").unwrap().clone();

                    match subscriber.bind(addr) {
                        Ok(_) => {
                            println!("LMCP Subscriber: bound to {}", addr);
                        }
                        Err(e) => {
                            println!("LMCP Subscriber error: {} binding to {}", e, addr);
                            exit(1);
                        }
                    }

                    assert!(subscriber.set_subscribe(filter.as_bytes()).is_ok());

                    let mut lmcp_msgs_to_send = VecDeque::new();
                    let mut mavlink_msgs_to_send = VecDeque::new();
                    move || loop {
                        let stream = subscriber.recv_bytes(0).unwrap();

                        if debug {
                            println!("Received {} bytes", stream.len());
                        }

                        if let Some(msg) = PixhawkProxy::decode_stream(stream) {
                            if debug {
                                println!("Received msg: {:#?}", msg);
                            }
                            match proxy.lock() {
                                Ok(mut mav_proxy) => {
                                    let (mut lmcp_msgs, mut mavlink_msgs) =
                                        mav_proxy.handle_lmcp_msg(msg);
                                    lmcp_msgs_to_send.append(&mut lmcp_msgs);
                                    mavlink_msgs_to_send.append(&mut mavlink_msgs);
                                }
                                Err(e) => {
                                    println!("Error locking PixhawkProxy: {}", e);
                                }
                            }

                            while !mavlink_msgs_to_send.is_empty() {
                                let mavlink_msg = mavlink_msgs_to_send.pop_front().unwrap();
                                send_to_mavlink.send(mavlink_msg).unwrap();
                            }

                            while !lmcp_msgs_to_send.is_empty() {
                                let lmcp_msg = lmcp_msgs_to_send.pop_front().unwrap();
                                // use the channel
                                send_to_lmcp.send(lmcp_msg).unwrap();
                            }
                        }
                    }
                }),
        );
    }

    // wait for threads to finish
    for handle in handles {
        handle.unwrap().join().unwrap();
    }
}
