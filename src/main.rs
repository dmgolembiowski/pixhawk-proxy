extern crate lmcp;
extern crate mavlink;

use std::sync::Arc;
use std::thread;
use std::env;
use std::time::Duration;

use lmcp::afrl::cmasi::air_vehicle_state::AirVehicleState;
use lmcp::afrl::cmasi::air_vehicle_state::AirVehicleStateT;
use lmcp::afrl::cmasi::entity_state::EntityStateT;
//use lmcp::afrl::cmasi::altitude_type::AltitudeType;
use mavlink::common::*;


/*
 Entity state
 ID - fixed
 u (dx) [m/s]: 0
 v (dy): 0 
 w (dz): 0
 udot [m/s^2]: <message id="105" name="HIGHRES_IMU">
 vdot:
 wdot:
 heading [0-360 deg]:           <message id="74" name="VFR_HUD">
 pitch [deg]: <message id="30" name="ATTITUDE">
 roll [deg]:
 p [deg/sec]: <message id="30" name="ATTITUDE">
 q:
 r:
 course [deg]:     <message id="74" name="VFR_HUD">
 groundspeed [deg]:     <message id="74" name="VFR_HUD">
 location:  <message id="33" name="GLOBAL_POSITION_INT">
 energyAvailable [%]:         <message id="147" name="BATTERY_STATUS"> battery_remaining
 actualEnergyrate [%/s]: // calculate diff between last two battery messages
 payloadStatelist: N/A
 currentWaypoint [wp id]: N/A
 currentCommand [command id]: N/A
 mode: ?
 associated tasks: N?A
 time [ms since 1/1/1970]: N/A
 info [keyval pairs]: N/A

 AirVehicle state
 Airspeed [m/s]:           <message id="74" name="VFR_HUD">
 VerticalSpeed [m/s]           <message id="74" name="VFR_HUD">
 WindSpeed [m/s]:      <message id="231" name="WIND_COV"> // computes from the message
 WindDirection [deg]     <message id="231" name="WIND_COV">
*/

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

fn main() {
    let args: Vec<_> = env::args().collect();

    if args.len() < 2 {
        println!("Usage: pixhawk-proxy (tcp|udpin|udpout|serial):(ip|dev):(port|baud)");
        return;
    }

    let vehicle = Arc::new(mavlink::connect(&args[1]).unwrap());
    let mut proxy = PixhawkProxy::default();

    thread::spawn({
        let vehicle = vehicle.clone();
        move || loop {
            vehicle.send(&mavlink::heartbeat_message()).ok();
            thread::sleep(Duration::from_secs(1));
        }
    });

    loop {
        if let Ok(msg) = vehicle.recv() {
            match msg {
                MavMessage::HEARTBEAT(data) => {
                    println!("Got {:?}", data);
                    // TODO: Check for the correct autopilot etc?
                }
                MavMessage::VFR_HUD(data) => {
                    *proxy.current_air_vehicle_state.airspeed_mut() = data.airspeed;
                    // TODO: Check range?
                    // [dependencies]
                    // range_check = "0.1"
                    // rust-range-check
                    // use range_check::Within;
                    // assert!(4123.is_within(3000..5000));
                    *proxy.current_air_vehicle_state.course_mut() = data.heading as f32;
                    *proxy.current_air_vehicle_state.groundspeed_mut() = data.groundspeed; // TODO: fix with GPS data
                }
                MavMessage::GLOBAL_POSITION_INT(data) => {
                     *proxy.current_air_vehicle_state.location.latitude_mut() = data.lat as f64 / 1e7;
                     *proxy.current_air_vehicle_state.location.longitude_mut() = data.lon as f64 / 1e7;
                     *proxy.current_air_vehicle_state.location.altitude_mut() = data.alt as f32 / 1000.0; // TODO: fix WGS84 to AMSL projection
                     // *proxy.current_air_vehicle_state.location.altitude_type_mut() = AltitudeType::MSL ?
                     *proxy.current_air_vehicle_state.vertical_speed_mut() = data.vz as f32 / -100.0; // TODO: check values
                     /*
                     Ground speed from NED: 
                        let v_north_m_s: f32 = gps_data.v_north as f32 / 100.0;
                        let v_east_m_s = gps_data.v_east as f32 / 100.0;
                        let v_down_m_s = gps_data.v_down as f32 / 100.0;
                        self.current_air_vehicle_state.course = v_east_m_s.atan2(v_north_m_s).to_degrees();
                     */
                }
                MavMessage::HIGHRES_IMU(data) => {
                    *proxy.current_air_vehicle_state.udot_mut() = data.xacc;
                    *proxy.current_air_vehicle_state.vdot_mut() = data.yacc;
                    *proxy.current_air_vehicle_state.wdot_mut() = data.zacc;
                }
                MavMessage::ATTITUDE(data) => {
                    // TODO: yaw? Heading?
                    *proxy.current_air_vehicle_state.heading_mut() = data.yaw.to_degrees() + 180.0;
                    *proxy.current_air_vehicle_state.pitch_mut() = data.pitch.to_degrees();
                    *proxy.current_air_vehicle_state.roll_mut() = data.roll.to_degrees();
                    *proxy.current_air_vehicle_state.p_mut() = data.rollspeed.to_degrees();
                    *proxy.current_air_vehicle_state.q_mut() = data.pitchspeed.to_degrees();
                    *proxy.current_air_vehicle_state.r_mut() = data.yawspeed.to_degrees();
                    
                }
                MavMessage::BATTERY_STATUS(data) => {
                    *proxy.current_air_vehicle_state.energy_available_mut() = data.battery_remaining as f32; // TODO: check for -1
                    //*proxy.current_air_vehicle_state.actual_energy_rate_mut() = TODO: estimate the current energy rate from msg timestamp 
                }
                /* TODO: Fix parser first
                MavMessage::WIND_COV(msg) => {
                    println!("Got {:?}", msg);
                }
                SYSTEM_TIME has time - should we use it? Or should we use localtime?
                */
                _ => {}
            }
        } else {
            break;
        }
    }
}
