
mod config {
    use std::{net::SocketAddr, env::{self, VarError}};
    use anyhow::{Result, Context, bail};
    use dotenvy::dotenv;

    /// Local robot config from enviroment.
    /// 
    /// Minium needed to contact centrall controll server.
    /// All the other settings shuld be gotten from server.
    pub struct LocalConfig{
        pub udp_address: SocketAddr, // Adress to be used by UDP server.
    }

    impl Default for LocalConfig {
        fn default() -> Self {
            Self { 
                udp_address: "0.0.0.0:6666".parse().unwrap(), 
            }
        }
    }

    impl LocalConfig {
        pub fn from_env() -> Result<LocalConfig> {
            dotenv().ok();

            let mut config = LocalConfig::default();

            match env::var("MINION_UDP_ADDRESS") {
                Ok(value) => config.udp_address = value.parse().context("MINION_UDP_ADDRESS must be a valid socket adress 0.0.0.0:6666")?,
                Err(VarError::NotPresent) => (),
                Err(err) => bail!(err)
            }

            Ok(config)
        }
    }

}

mod server {

}

mod drive {

}

mod arm {

}

use config::LocalConfig;
use pyo3::{
    types::{PyDict, PyModule},
    Py, PyAny, PyResult, Python,
};
use rust_gpiozero::{Motor, OutputDevice};
use serde::Deserialize;
use std::{
    net::UdpSocket,
    sync::{Arc, Mutex},
    thread,
};

use anyhow::Result;

fn main() -> Result<()> {
    let config = LocalConfig::from_env()?;

    let network = network_start(&config);
    let mut drive = drive_start();
    let arm = arm_start();

    let tracking = Arc::new(Mutex::new(Tracking::default()));

    let tracking_net = tracking.clone();
    thread::spawn(move || loop {
        let data = network_get(&network);
        *tracking_net.lock().unwrap() = data;
    });

    let tracking_drive = tracking.clone();
    thread::spawn(move || loop {
        let lock = tracking_drive.lock().unwrap();
        let data = *lock;
        drop(lock);
        drive_run(&mut drive, &data);
    });

    loop {
        let lock = tracking.lock().expect("Tracking mutex is poisonius");
        let data = *lock;
        drop(lock);
        arm_run(&arm, &data);
    }
}

#[allow(dead_code)]
#[derive(Deserialize, Debug, Default, Clone, Copy)]
struct Tracking {
    rx: f64,
    ry: f64,
    rz: f64,
    l: Controller,
    r: Controller,
}

#[allow(dead_code)]
#[derive(Deserialize, Debug, Default, Clone, Copy)]
struct Controller {
    x: f64,  // Thumb Sticks X
    y: f64,  // Thumb Sticks X
    a: bool, // A or X button
    b: bool, // B or Y button
    c: f64,  // Trigger
    d: f64,  // Grip
}

fn network_start(config: &LocalConfig) -> UdpSocket {
    println!("Binding to UDP:{}", config.udp_address);
    UdpSocket::bind(config.udp_address).expect("UDP failed to bind")
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
            front_left: Wheel::new(20, 16, 21),
            front_right: Wheel::new(13, 19, 26),
            back_left: Wheel::new(23, 24, 25),
            back_right: Wheel::new(17, 27, 22),
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
    fn new(forward_pin: u8, backward_pin: u8, enable_pin: u8) -> Self {
        let mut enable = OutputDevice::new(enable_pin);
        enable.on();
        Self {
            motor: Motor::new(forward_pin, backward_pin),
            _enable: enable,
        }
    }

    fn set_speed(&mut self, speed: f64) {
        self.motor.set_speed(f64::abs(speed));
        if speed.signum() == 1.0 {
            self.motor.forward();
        } else {
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

/// The python code to controll the arm compiled in.
const ARM_PY: &str = include_str!("./arm.py");

struct Arm {
    module: Py<PyModule>,
    object: Py<PyAny>,
}

fn arm_start() -> Arm {
    Python::with_gil(|py| -> PyResult<Arm> {
        // compile and run python code.
        let arm_module = PyModule::from_code(py, ARM_PY, "arm.py", "arm")?;

        // run the arm_start function.
        let arm_object = arm_module.getattr("arm_start")?.call0()?;

        Ok(Arm {
            module: arm_module.into(),
            object: arm_object.into(),
        })
    })
    .unwrap()
}

fn arm_run(arm: &Arm, data: &Tracking) {
    Python::with_gil(|py| -> PyResult<()> {
        // Copy data into PyDict
        let py_data = PyDict::new(py);
        py_data.set_item("rx", data.rx)?;
        py_data.set_item("ry", data.ry)?;
        py_data.set_item("rz", data.rz)?;

        // Call the run function
        arm.module
            .getattr(py, "arm_run")?
            .call1(py, (&arm.object, py_data))?;
        Ok(())
    })
    .unwrap()
}
