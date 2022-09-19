use anyhow::{bail, Context, Result};
use dotenvy::dotenv;
use std::{
    env::{self, VarError},
    net::SocketAddr,
};

/// Local robot config from enviroment.
///
/// Minium needed to contact centrall controll server.
/// All the other settings shuld be gotten from server.
pub struct LocalConfig {
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
            Ok(value) => {
                config.udp_address = value
                    .parse()
                    .context("MINION_UDP_ADDRESS must be a valid socket adress 0.0.0.0:6666")?
            }
            Err(VarError::NotPresent) => (),
            Err(err) => bail!(err),
        }

        Ok(config)
    }
}
