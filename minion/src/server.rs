use std::{net::UdpSocket, sync::Arc, thread};

use anyhow::{Context, Result};
use parking_lot::Mutex;
use serde::Deserialize;

use crate::config::LocalConfig;

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
    y: f64,  // Thumb Sticks Y
    a: bool, // A or X button
    b: bool, // B or Y button
    c: f64,  // Trigger
    d: f64,  // Grip
}

pub struct Head {
    pub rx: f64,
    pub ry: f64,
    pub rz: f64,
}

pub struct Drive {
    pub speed: f64,
    pub turn: f64,
}

#[derive(Clone)]
pub struct Server {
    socket: Arc<UdpSocket>,
    tracking: Arc<Mutex<Tracking>>,
}

impl Server {
    pub fn new(config: &LocalConfig) -> Result<Self> {
        Ok(Server {
            socket: Arc::new(UdpSocket::bind(config.udp_address).context("UDP failed to bind")?),
            tracking: Arc::default(),
        })
    }

    /// Starts sever in a diffrent tread.
    pub fn start(&self) {
        let server = self.clone();
        thread::spawn(move || loop {
            let mut buf = [0; 512];
            let (amt, _src) = server
                .socket
                .recv_from(&mut buf)
                .expect("Read from network error");

            let mut tracking = server.tracking.lock();
            *tracking =
                serde_json::from_slice(&buf[..amt]).expect("Faild to parse tracking message")
        });
    }

    pub fn head(&self) -> Head {
        let tracking = self.tracking.lock();
        Head {
            rx: tracking.rx,
            ry: tracking.ry,
            rz: tracking.rz,
        }
    }

    pub fn drive(&self) -> Drive {
        let tracking = self.tracking.lock();
        Drive {
            speed: tracking.l.y,
            turn: tracking.l.x,
        }
    }
}
