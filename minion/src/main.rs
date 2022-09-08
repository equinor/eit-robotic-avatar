use std::{net::UdpSocket, sync::{Mutex, Arc}, thread, f64::consts::{PI, FRAC_PI_2}, time::Duration};

use rust_gpiozero::{Motor, OutputDevice};
use rustdds::{DomainParticipant, QosPolicyBuilder, policy, ros2::RosParticipant};
use serde::Deserialize;

fn main() {
    let network = network_start();
    let mut drive = drive_start();
    let arm = arm_start();

    let tracking = Arc::new(Mutex::new(Tracking::default()));

    let tracking_net = tracking.clone();
    thread::spawn( move || {
        loop {
            let data = network_get(&network); 
            *tracking_net.lock().unwrap() = data;
        }
    });

    let tracking_drive = tracking.clone();
    thread::spawn( move || {
        loop {
            let lock = tracking_drive.lock().unwrap();
            let data = *lock;
            drop(lock);
            drive_run(&mut drive, &data);
        }
    });

    loop {
        let lock = tracking.lock().unwrap();
        let data = *lock;
        drop(lock);
        arm_run(arm, &data);
    }
}


#[derive(Deserialize, Debug, Default, Clone, Copy)]
struct Tracking {
    rx: f64,
    ry: f64,
    rz: f64,
    l: Controller,
    r: Controller,
}

#[derive(Deserialize, Debug, Default, Clone, Copy)]
struct Controller {
    x: f64,  // Thumb Sticks X
    y: f64,  // Thumb Sticks X
    a: bool, // A or X button
    b: bool, // B or Y button
    c: f64,  // Trigger
    d: f64,  // Grip
}

fn network_start() -> UdpSocket {
    const ADDRESS: &str = "0.0.0.0:6666";
    println!("Binding to UDP:{}", ADDRESS);
    UdpSocket::bind(ADDRESS).expect("UDP failed to bind")
}

fn network_get(network: &UdpSocket) -> Tracking {
    let mut buf = [0; 512];
    let (amt, _src) = network
        .recv_from(&mut buf)
        .expect("Read from network error");

    serde_json::from_slice(&buf[..amt]).expect("Faild to parse tracking message")
}

struct Drive {
    front_left: Wheel,
    front_right: Wheel, 
    back_left: Wheel,
    back_right: Wheel,
}

impl Drive {
    fn new() -> Self {
        // Setting up the GPIO pins for the wheels.
        Self {
            front_left: Wheel::new(20,16,21),
            front_right: Wheel::new(13,19,26),
            back_left: Wheel::new(23,24,25),
            back_right: Wheel::new(17,27,22)
        }
    }

    fn set_speed(&mut self, left: f64, right: f64) {
        self.front_left.set_speed(left);
        self.front_right.set_speed(right);
        self.back_left.set_speed(left);
        self.back_right.set_speed(right);
    }
}

struct Wheel {
    motor: Motor,
    _enable: OutputDevice,
}

impl Wheel {
    fn new(forward_pin: u8, backward_pin:u8, enable_pin: u8) -> Self{
        let mut enable = OutputDevice::new(enable_pin);
        enable.on();
        Self {
            motor: Motor::new(forward_pin, backward_pin),
            _enable: enable
        }
    }

    fn set_speed(&mut self, speed: f64) {
        self.motor.set_speed(f64::abs(speed));
        if speed.signum() == 1.0 {
            self.motor.forward();
        }else {
            self.motor.backward();
        }
    }
}

fn drive_start() -> Drive {
    println!("Starting drive");
    Drive::new()
}

fn drive_run(drive: &mut Drive, data: &Tracking) {
    let controller = &data.l;
    let y = controller.y;
    let x = controller.x;

    let w = (1.0 - f64::abs(y)) * (x) + x;
    let v = (1.0 - f64::abs(x)) * (y) + y;

    let left = -(v - w) / 2.0;
    let right = -(v + w) / 2.0;

    drive.set_speed(left, right);
}

fn arm_start() {
    let mut ros_participant = RosParticipant::new().unwrap();

    println!("Looking for topics");
    let nodes = ros_participant.handle_node_read();
    println!("{:?}", nodes);
    thread::sleep(Duration::from_millis(1000));
    let nodes = ros_participant.discovered_topics();
    println!("{:?}", nodes);
}

fn arm_tranlation(rx: f64, ry: f64, rz: f64) -> (f64, f64, f64) {
    let rx = rx * PI;
    let ry = ry * PI;

    // We dont want it to look backwards so we clamp it to +- half a PI
    let rx = rx.clamp(-FRAC_PI_2, FRAC_PI_2);
    let ry = ry.clamp(-FRAC_PI_2, FRAC_PI_2);

    // RZ need to be handel spesialy as robot have strange gripper API.
    let rz = rz.clamp(-0.25, 0.25);
    let rz = rz * 0.02;

    (rx,ry,rz)
}

fn arm_run(_arm: (), data: &Tracking) {
    let (rx, ry, rz) = arm_tranlation(data.rx, data.ry, data.rz);

    //println!("rx:{}, ry:{} rz:{}", rx, ry, rz);
    thread::sleep(Duration::from_millis(500));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arm_tranlation_posetive_limits() {
        let out = arm_tranlation(1.0,1.0,1.0);
        assert_eq!(out, (FRAC_PI_2,FRAC_PI_2,0.005));
    }
}
