use std::{net::UdpSocket, sync::{Mutex, Arc}, thread};

use rust_gpiozero::{Motor, OutputDevice};
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

    loop {
        let lock = tracking.lock().unwrap();
        let data = *lock;
        drop(lock);
        drive_run(&mut drive, &data);
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
    enable: OutputDevice,
}

impl Wheel {
    fn new(forward_pin: u8, backward_pin:u8, enable_pin: u8) -> Self{
        let mut enable = OutputDevice::new(enable_pin);
        enable.on();
        Self {
            motor: Motor::new(forward_pin, backward_pin),
            enable
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
    println!("Arm controll not implmeted");
}

fn arm_run(_arm: (), _data: &Tracking) {}
