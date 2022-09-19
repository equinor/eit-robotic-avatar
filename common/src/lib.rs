use serde::{Serialize, Deserialize};

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct Tracking { 
    pub head: Head,
    pub drive: Drive,
}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct Head {
    pub rx: f64,
    pub ry: f64,
    pub rz: f64,
}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct Drive { 
    pub speed: f64,
    pub turn: f64,
}