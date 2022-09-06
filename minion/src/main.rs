use std::net::UdpSocket;

use serde::Deserialize;

fn main() {
    let network = network_start();
    let drive = drive_start();
    let arm = arm_start();

    loop{
        let data = network_get(&network);
        println!("{:?}", &data);
        drive_run(drive, &data);
        arm_run(arm, &data); 
    }
}

#[derive(Deserialize, Debug)]
struct Tracking{
    rx: f64,
    ry: f64,
    rz: f64,
    l: Controller,
    r: Controller,
}

#[derive(Deserialize, Debug)]
struct Controller {
    x: f64, // Thumb Sticks X
    y: f64, // Thumb Sticks X
    a: bool, // A or X button
    b: bool, // B or Y button
    c: f64, // Trigger
    d: f64, // Grip
}

fn network_start() -> UdpSocket {
    const ADDRESS: &str = "0.0.0.0:6666";
    println!("Binding to UDP:{}", ADDRESS);
    UdpSocket::bind(ADDRESS).expect("UDP failed to bind")
}

fn network_get(network: &UdpSocket) -> Tracking {
    let mut buf = [0; 512];
    let (amt, _src) = network.recv_from(&mut buf).expect("Read from network error");

    serde_json::from_slice(&buf[..amt]).expect("Faild to parse tracking message")
}

fn drive_start() {
    println!("Drive not implmeted");
}

fn drive_run(_drive: (), _data: &Tracking) {

}

fn arm_start() {
    println!("Arm controll not implmeted");
}

fn arm_run(_arm: (), _data: &Tracking) {

}