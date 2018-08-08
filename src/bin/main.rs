/// Main file for Pixhawk proxy
///
/// We have 4 threads (all use ZMQ for external message passing, and `sync::mpsc::channel` for internal message passing):
/// 	LMCP TX: sends LMCP messages (mainly air_vehicle_state) to `LmcpObjectNetworkPublishPullBridge` when a new message is available
///		LMCP RX: receives LMCP messages and processes them when appropriate (mostly waypoints)
///		MAVLINK RX: receives Mavlink_protobuf messages and updates `air_vehicle_state` struct
///		MAVLINK TX: sends Mavlink_protobuf messages that were created from incoming LMCP messages (such as mission and waypoint updates)
///
/// TODO: add log!() crate for better logging
#[macro_use]
extern crate clap;

extern crate pixhawk_proxy;
extern crate prost;
extern crate zmq;

use std::collections::VecDeque;
use std::io::Cursor;
use std::process::exit;
use std::sync::mpsc::{channel, Sender};
use std::sync::{Arc, Mutex};
use std::thread;

use clap::App;
use prost::Message;

use pixhawk_proxy::mavlink_common::MavlinkMessage;
use pixhawk_proxy::PixhawkProxy;

/// pixhawk-proxy [FLAGS] <MAVLINK_SUB> <MAVLINK_PUB> <LMCP_SUB> <LMCP_PUB>
/// Run for example with `cargo run -- -s tcp://127.0.0.1:4440 tcp://127.0.0.1:44440 tcp://127.0.0.1:5555 tcp://127.0.0.1:5555`
/// Or with a secondary bridge: `cargo run -- -s -d tcp://127.0.0.1:4440 tcp://127.0.0.1:44440 tcp://127.0.0.1:9999 none`
fn main() {
    let yaml = load_yaml!("../../cli.yml");
    let matches = App::from_yaml(yaml).get_matches();

    // Default ZMQ context
    let context = zmq::Context::new();

    // Define channels
    let (lmcp_tx, lmcp_rx) = channel();
    let (mavlink_tx, mavlink_rx) = channel();

    // create proxy
    let mut proxy = PixhawkProxy::default();
    proxy.set_debug(matches.is_present("debug"));
    let proxy = Arc::new(Mutex::new(proxy));

    // keep thread handles
    let mut handles = vec![];

    // MAVLINK RX thread:
    // receive and process Mavlink messages
    // calls `handle_mavlink_msg` and sends resulting LMCP and MAVLINK messages
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
                match subscriber.connect(addr) {
                    Ok(_) => {
                        println!("Mavlink Subscriber: connected to {}", addr);
                    }
                    Err(e) => {
                        println!("Mavlink Subscriber error: {} connecting to {}", e, addr);
                        exit(1);
                    }
                }
                assert!(subscriber.set_subscribe(filter.as_bytes()).is_ok());

                move || loop {
                    let stream = subscriber.recv_bytes(0).unwrap();
                    let msg = MavlinkMessage::decode(&mut Cursor::new(stream)).unwrap();
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

    // MAVLINK TX thread:
    // Sending Mavlink proto messages
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
                            println!("Got a message to send: {:#?}", msg);
                            let mut buf = Vec::with_capacity(msg.encoded_len());
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

    if !matches.is_present("stream") {
        // LMCP TX thread:
        // Sending AirVehicle and other LMCP messages to UxAS or OpenAMASE
        // spawn only if *not* using ZMQ_STREAM
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
        // LMCP RX & TX thread:
        // receive and send LMCP messages using ZMQ_STREAM
        handles.push(
            thread::Builder::new()
                .name("th_lmcp_rx".to_string())
                .spawn({
                    let proxy = proxy.clone();
                    let debug = matches.is_present("debug");
                    let send_to_mavlink = Sender::clone(&mavlink_tx);
                    println!("Connecting as zmq::STREAM");
                    let socket = context.socket(zmq::STREAM).unwrap();
                    // TODO: make the timeout configurable
                    socket.set_rcvtimeo(1000).unwrap();

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
                    let mut stream = vec![];
                    move || loop {
                        match socket.recv_multipart(0) {
                            Ok(mut mpart) => {
                                assert_eq!(mpart.len(), 2); // assert we have only 2 messages
                                stream.append(&mut mpart.pop().unwrap()); // ~= mpart[1]
                                if client_id == None {
                                    client_id = Some(mpart.pop().unwrap());
                                    if debug {
                                        println!("th_lmcp_rx:cliend_id: {:?}", client_id);
                                    }
                                }

                                if debug {
                                    println!("th_lmcp_rx:Received {} bytes", stream.len());
                                }

                                let (res, rem) = PixhawkProxy::decode_stream(stream.clone());

                                match res {
                                    Some(msg) => {
                                        // process msg
                                        if debug {
                                            println!("th_lmcp_rx:Received msg: {:?}", msg);
                                        }
                                        match proxy.lock() {
                                            Ok(mut mav_proxy) => {
                                                if debug {
                                                    println!("th_lmcp_rx:processing message");
                                                }
                                                let (mut lmcp_msgs, mut mavlink_msgs) =
                                                    mav_proxy.handle_lmcp_msg(msg);
                                                lmcp_msgs_to_send.append(&mut lmcp_msgs);
                                                mavlink_msgs_to_send.append(&mut mavlink_msgs);
                                            }
                                            Err(e) => {
                                                println!(
                                                    "th_lmcp_rx:Error locking PixhawkProxy: {}",
                                                    e
                                                );
                                            }
                                        }
                                        stream.clear();
                                    }
                                    None => {
                                        // keep the stream data
                                        println!("Keeping {} of data", rem.len());
                                        stream = rem;
                                    }
                                }
                            }
                            Err(_e) => {
                                //println!("th_lmcp_rx:Error recv_multipart: {}", e);
                            }
                        }

                        while !mavlink_msgs_to_send.is_empty() {
                            let mavlink_msg = mavlink_msgs_to_send.pop_front().unwrap();
                            send_to_mavlink.send(mavlink_msg).unwrap();
                        }

                        match lmcp_rx.try_recv() {
                            Ok(msg) => {
                                lmcp_msgs_to_send.push_back(msg);
                            }
                            Err(_e) => {
                                //println!("th_lmcp_rx: error at lmcprx.try_recv(): {:?}", e);
                            }
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
                }),
        );
    }

    if !matches.is_present("stream") {
        // LMCP RX thread:
        // Use SUBscriber and process incoming LMCP messages
        handles.push(
            thread::Builder::new()
                .name("th_lmcp_rx".to_string())
                .spawn({
                    let proxy = proxy.clone();
                    let debug = matches.is_present("debug");
                    let send_to_lmcp = Sender::clone(&lmcp_tx);
                    let send_to_mavlink = Sender::clone(&mavlink_tx);
                    println!("Connecting as zmq::SUB");
                    let subscriber = context.socket(zmq::SUB).unwrap();

                    let filter = "";
                    let addr = matches.value_of("LMCP_SUB").unwrap().clone();

                    match subscriber.connect(addr) {
                        Ok(_) => {
                            println!("LMCP Subscriber: connected to {}", addr);
                        }
                        Err(e) => {
                            println!("LMCP Subscriber error: {} connecting to {}", e, addr);
                            exit(1);
                        }
                    }

                    assert!(subscriber.set_subscribe(filter.as_bytes()).is_ok());

                    let mut lmcp_msgs_to_send = VecDeque::new();
                    let mut mavlink_msgs_to_send = VecDeque::new();
                    let mut stream = vec![];
                    move || loop {
                        //let stream = subscriber.recv_bytes(0).unwrap();
                        stream.append(&mut subscriber.recv_bytes(0).unwrap());

                        if debug {
                            println!("Received {} bytes", stream.len());
                        }

                        let (res, rem) = PixhawkProxy::decode_stream(stream.clone());
                        match res {
                            Some(msg) => {
                                // process msg
                                if debug {
                                    println!("th_lmcp_rx:Received msg: {:#?}", msg);
                                }
                                match proxy.lock() {
                                    Ok(mut mav_proxy) => {
                                        if debug {
                                            println!("th_lmcp_rx:processing message");
                                        }
                                        let (mut lmcp_msgs, mut mavlink_msgs) =
                                            mav_proxy.handle_lmcp_msg(msg);
                                        lmcp_msgs_to_send.append(&mut lmcp_msgs);
                                        mavlink_msgs_to_send.append(&mut mavlink_msgs);
                                    }
                                    Err(e) => {
                                        println!("th_lmcp_rx:Error locking PixhawkProxy: {}", e);
                                    }
                                }
                                stream.clear();
                            }
                            None => {
                                // keep the stream data
                                println!("Keeping {} of data", rem.len());
                                stream = rem;
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
                }),
        );
    }

    // wait for threads to finish
    for handle in handles {
        handle.unwrap().join().unwrap();
    }
}
