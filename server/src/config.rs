use std::net::SocketAddr;
use dotenvy::dotenv;
use figment::{Figment, providers::{Toml, Env, Format, Serialized}};
use serde::{Deserialize, Serialize};
use anyhow::Result;    

#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
    pub minion_udp_address: SocketAddr,
    pub extra_cert_names: Vec<String>,
    pub bind_address: SocketAddr,
    pub https: bool,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            minion_udp_address: "192.168.41.58:6666".parse().unwrap(),
            extra_cert_names: vec!["192.168.41.33".to_string()],
            bind_address: "0.0.0.0:3000".parse().unwrap(),
            https: true,
        }
    }
}

impl Config {
    pub fn load() -> Result<Self> {
        dotenv().ok();
        Ok(Figment::from(Serialized::defaults(Config::default()))
            .merge(Toml::file("avatar.toml"))
            .merge(Env::prefixed("AVATAR_"))
            .extract()?)
    }
}