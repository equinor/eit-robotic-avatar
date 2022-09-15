mod config;
mod legacy;
mod server;

use anyhow::Result;

use crate::config::Config;

#[tokio::main]
async fn main() -> Result<()> {
    // Load config
    let config = Config::load()?;

    // Load modules
    let app = legacy::route(&config).await?;

    // Start the server based on config
    server::serve(app, &config).await?;
    Ok(())
}