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
use std::sync::mpsc::{channel, Sender, Receiver};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use clap::App;
use prost::Message;

use pixhawk_proxy::mavlink_common::MavlinkMessage;
use pixhawk_proxy::LmcpMessage;
use pixhawk_proxy::PixhawkProxy;

/// Periodic thread
/// Sends periodically an LMCP message
fn thread_periodic_lmcp<F>(proxy: Arc<Mutex<PixhawkProxy>>, send_to_lmcp: Sender<LmcpMessage>, fn_periodic: F, debug: bool, period: u64) 
where F: Fn(&PixhawkProxy) -> LmcpMessage {
    loop {
        let msg;
        match proxy.lock() {
            Ok(p) => {
                msg = Some(fn_periodic(&p));
            },
            Err(e) => {
                msg = None;
                println!("Error locking proxy: {}", e);
            },
        }
        if let Some(m) = msg {
            if debug {
                println!("thread_periodic_lmcp: Sending {:?}", m); 
            }
            send_to_lmcp.send(m).unwrap();
        }
        thread::sleep(Duration::from_millis(period));
    }
}

/// MAVLINK RX thread:
/// receive and process Mavlink messages
/// calls `handle_mavlink_msg` and sends resulting LMCP and MAVLINK messages
fn thread_mavlink_rx(
    proxy: Arc<Mutex<PixhawkProxy>>,
    send_to_lmcp: Sender<LmcpMessage>,
    send_to_mavlink: Sender<MavlinkMessage>,
    subscriber: zmq::Socket,
    addr: String,
    filter: String,
    _debug: bool,
) {
    match subscriber.connect(&addr) {
        Ok(_) => {
            println!("Mavlink Subscriber: connected to {}", addr);
        },
        Err(e) => {
            println!("Mavlink Subscriber error: {} connecting to {}", e, addr);
            exit(1);
        },
    }
    assert!(subscriber.set_subscribe(filter.as_bytes()).is_ok());

    loop {
        let stream = subscriber.recv_bytes(0).unwrap();
        let msg = MavlinkMessage::decode(&mut Cursor::new(stream)).unwrap();
        let mut lmcp_msgs_to_send = VecDeque::new();
        let mut mavlink_msgs_to_send = VecDeque::new();
        match proxy.lock() {
            Ok(mut mav_proxy) => {
                let (mut lmcp_msgs, mut mavlink_msgs) = mav_proxy.handle_mavlink_msg(msg);
                lmcp_msgs_to_send.append(&mut lmcp_msgs);
                mavlink_msgs_to_send.append(&mut mavlink_msgs);
            },
            Err(e) => {
                println!("Error locking proxy: {}", e);
            },
        }
        for mavlink_msg in mavlink_msgs_to_send {
            send_to_mavlink.send(mavlink_msg).unwrap();
        }
        for lmcp_msg in lmcp_msgs_to_send {
            send_to_lmcp.send(lmcp_msg).unwrap();
        }
    }
}

/// MAVLINK TX thread:
/// Sending Mavlink proto messages
fn thread_mavlink_tx(mavlink_rx: Receiver<MavlinkMessage>, publisher: zmq::Socket, addr: String, _debug: bool) {
    match publisher.bind(&addr) {
        Ok(_) => {
            println!("Mavlink Publisher: bound to {}", addr);
        },
        Err(e) => {
            println!("Mavlink Publisher error: {} connecting to {}", e, addr);
            exit(1);
        },
    }

    loop {
        match mavlink_rx.recv() {
            Ok(msg) => {
                // encode message into a buffer and send data
                println!("Got a message to send: {:#?}", msg);
                let mut buf = Vec::with_capacity(msg.encoded_len());
                msg.encode(&mut buf).unwrap();
                publisher.send(&buf, 0).unwrap(); // send buffer with 0 flags
            },
            Err(e) => {
                println!("Error occured: {}", e);
            },
        }
    }
}

/// LMCP TX thread:
/// Sending AirVehicle and other LMCP messages to UxAS or OpenAMASE
/// spawn only if *not* using ZMQ_STREAM
fn thread_lmcp_tx(lmcp_rx: Receiver<LmcpMessage>, publisher: zmq::Socket, addr: String, _debug: bool) {
    match publisher.bind(&addr) {
        Ok(_) => {
            println!("LMCP Publisher: bound to {}", addr);
        },
        Err(e) => {
            println!("LMCP Publisher error: {} connecting to {}", e, addr);
            exit(1);
        },
    }

    loop {
        match lmcp_rx.recv() {
            Ok(msg) => {
                // serialize message and send data
                if let Some(stream) = PixhawkProxy::encode_message(msg) {
                    publisher.send(&stream, 0).unwrap(); // send buffer with 0 flags
                }
            },
            Err(e) => {
                println!("Error occured: {}", e);
            },
        }
    }
}

/// LMCP RX & TX thread:
/// receive and send LMCP messages using ZMQ_STREAM
fn thread_lmcp_stream(
    proxy: Arc<Mutex<PixhawkProxy>>,
    lmcp_rx: Receiver<LmcpMessage>,
    send_to_mavlink: Sender<MavlinkMessage>,
    socket: zmq::Socket,
    addr: String,
    debug: bool,
    timeout: i32,
) {
    socket.set_rcvtimeo(timeout).unwrap();
    match socket.connect(&addr) {
        Ok(_) => {
            println!("LMCP Subscriber: connected to {}", addr);
        },
        Err(e) => {
            println!("LMCP Subscriber error: {} connecting to {}", e, addr);
            exit(1);
        },
    }

    let mut client_id = None;
    let mut lmcp_msgs_to_send = VecDeque::new();
    let mut mavlink_msgs_to_send = VecDeque::new();
    let mut stream = vec![];
    loop {
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
                        if debug {
                            println!("th_lmcp_rx:Received msg: {:?}", msg);
                        }
                        match proxy.lock() {
                            Ok(mut mav_proxy) => {
                                if debug {
                                    println!("th_lmcp_rx:processing message");
                                }
                                let (mut lmcp_msgs, mut mavlink_msgs) = mav_proxy.handle_lmcp_msg(msg);
                                lmcp_msgs_to_send.append(&mut lmcp_msgs);
                                mavlink_msgs_to_send.append(&mut mavlink_msgs);
                            },
                            Err(e) => {
                                println!("th_lmcp_rx:Error locking PixhawkProxy: {}", e);
                            },
                        }
                        stream.clear();
                    },
                    None => {
                        // keep the stream data
                        println!("Keeping {} of data", rem.len());
                        stream = rem;
                    },
                }
            },
            Err(_e) => {
                //println!("th_lmcp_rx:Error recv_multipart: {}", e);
            },
        }

        while !mavlink_msgs_to_send.is_empty() {
            let mavlink_msg = mavlink_msgs_to_send.pop_front().unwrap();
            send_to_mavlink.send(mavlink_msg).unwrap();
        }

        match lmcp_rx.try_recv() {
            Ok(msg) => {
                lmcp_msgs_to_send.push_back(msg);
            },
            Err(_e) => {
                //println!("th_lmcp_rx: error at lmcprx.try_recv(): {:?}", e);
            },
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

/// LMCP RX thread:
/// Use SUBscriber and process incoming LMCP messages
fn thread_lmcp_rx(
    proxy: Arc<Mutex<PixhawkProxy>>,
    send_to_lmcp: Sender<LmcpMessage>,
    send_to_mavlink: Sender<MavlinkMessage>,
    subscriber: zmq::Socket,
    addr: String,
    filter: String,
    debug: bool,
) {
    match subscriber.connect(&addr) {
        Ok(_) => {
            println!("LMCP Subscriber: connected to {}", addr);
        },
        Err(e) => {
            println!("LMCP Subscriber error: {} connecting to {}", e, addr);
            exit(1);
        },
    }
    assert!(subscriber.set_subscribe(filter.as_bytes()).is_ok());

    let mut lmcp_msgs_to_send = VecDeque::new();
    let mut mavlink_msgs_to_send = VecDeque::new();
    let mut stream = vec![];
    loop {
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
                        let (mut lmcp_msgs, mut mavlink_msgs) = mav_proxy.handle_lmcp_msg(msg);
                        lmcp_msgs_to_send.append(&mut lmcp_msgs);
                        mavlink_msgs_to_send.append(&mut mavlink_msgs);
                    },
                    Err(e) => {
                        println!("th_lmcp_rx:Error locking PixhawkProxy: {}", e);
                    },
                }
                stream.clear();
            },
            None => {
                // keep the stream data
                println!("Keeping {} of data", rem.len());
                stream = rem;
            },
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

/// pixhawk-proxy [FLAGS] <MAVLINK_SUB> <MAVLINK_PUB> <LMCP_SUB> <LMCP_PUB>
/// Run for example with `cargo run -- -s tcp://127.0.0.1:4440 tcp://127.0.0.1:44440 tcp://127.0.0.1:5555 tcp://127.0.0.1:5555`
/// Or with a secondary bridge: `cargo run -- -s -d tcp://127.0.0.1:4440 tcp://127.0.0.1:44440 tcp://127.0.0.1:9999 none`
fn main() {
    let yaml = load_yaml!("../../cli.yml");
    let matches = App::from_yaml(yaml).get_matches();

    // Default ZMQ contextzmq::Socket
    let context = zmq::Context::new();

    // Define channels
    let (lmcp_tx, lmcp_rx) = channel();
    let (mavlink_tx, mavlink_rx) = channel();

    // create proxy
    let mut proxy = PixhawkProxy::default();
    proxy.set_debug(matches.is_present("debug"));
    proxy.set_autostart(matches.is_present("autostart"));
    let proxy = Arc::new(Mutex::new(proxy));

    // keep thread handles
    let mut handles = vec![];

    // MAVLINK RX thread:
    // receive and process Mavlink messages
    // calls `handle_mavlink_msg` and sends resulting LMCP and MAVLINK messages
    let p = proxy.clone();
    let sender_lmcp = Sender::clone(&lmcp_tx);
    let sender_mavlink = Sender::clone(&mavlink_tx);
    let socket = context.socket(zmq::SUB).unwrap();
    let addr = matches.value_of("MAVLINK_SUB").unwrap().to_string();
    let filter = String::from("");
    let debug = matches.is_present("debug");
    handles.push(
        thread::Builder::new()
            .name("th_mavlink_rx".to_string())
            .spawn({ move || thread_mavlink_rx(p, sender_lmcp, sender_mavlink, socket, addr, filter, debug) }),
    );

    // MAVLINK TX thread:
    // Sending Mavlink proto messages
    let socket = context.socket(zmq::PUB).unwrap();
    let addr = matches.value_of("MAVLINK_PUB").unwrap().to_string();
    let debug = matches.is_present("debug");
    handles.push(
        thread::Builder::new()
            .name("th_mavlink_tx".to_string())
            .spawn({ move || thread_mavlink_tx(mavlink_rx, socket, addr, debug) }),
    );

    if !matches.is_present("stream") {
        // LMCP TX thread:
        // Sending AirVehicle and other LMCP messages to UxAS or OpenAMASE
        // spawn only if *not* using ZMQ_STREAM
        let socket = context.socket(zmq::PUB).unwrap();
        let addr = matches.value_of("LMCP_PUB").unwrap().to_string();
        let debug = matches.is_present("debug");
        handles.push(
            thread::Builder::new()
                .name("th_lmcp_tx".to_string())
                .spawn({ move || thread_lmcp_tx(lmcp_rx, socket, addr, debug) }),
        );

        // LMCP RX thread:
        // Use SUBscriber and process incoming LMCP messages
        let p = proxy.clone();
        let sender_lmcp = Sender::clone(&lmcp_tx);
        let sender_mavlink = Sender::clone(&mavlink_tx);
        let socket = context.socket(zmq::SUB).unwrap();
        let addr = matches.value_of("LMCP_SUB").unwrap().to_string();
        let filter = String::from("");
        let debug = matches.is_present("debug");
        handles.push(
            thread::Builder::new()
                .name("th_lmcp_rx".to_string())
                .spawn({ move || thread_lmcp_rx(p, sender_lmcp, sender_mavlink, socket, addr, filter, debug) }),
        );
    } else {
        println!("With ZMQ::STREAM Rx and Tx is handled with a single socket");
        // LMCP RX & TX thread:
        // receive and send LMCP messages using ZMQ_STREAM
        let p = proxy.clone();
        let sender_mavlink = Sender::clone(&mavlink_tx);
        let socket = context.socket(zmq::STREAM).unwrap();
        let addr = matches.value_of("LMCP_SUB").unwrap().to_string();
        let debug = matches.is_present("debug");
        let timeout = 1000; // TODO: make it configurable (maybe from console?)
        handles.push(
            thread::Builder::new()
                .name("th_lmcp_stream".to_string())
                .spawn({ move || thread_lmcp_stream(p, lmcp_rx, sender_mavlink, socket, addr, debug, timeout) }),
        );
    }

    // Spawn periodic threads
    let p = proxy.clone();
    let sender_lmcp = Sender::clone(&lmcp_tx);
    let debug = matches.is_present("debug");
    handles.push(
        thread::Builder::new()
            .name("thread_periodic_lmcp_session_status".to_string())
            .spawn({ move || thread_periodic_lmcp(p, sender_lmcp, PixhawkProxy::get_session_status, debug, 4000) }),
    );
    
    let p = proxy.clone();
    let sender_lmcp = Sender::clone(&lmcp_tx);
    let debug = matches.is_present("debug");
    handles.push(
        thread::Builder::new()
            .name("thread_periodic_lmcp_entity_configuration".to_string())
            .spawn({ move || thread_periodic_lmcp(p, sender_lmcp, PixhawkProxy::get_air_vehicle_configuration, debug, 2000) }),
    );

    // wait for threads to finish
    for handle in handles {
        handle.unwrap().join().unwrap();
    }
}
