mod config;
mod legacy;
mod server;
mod minion;

use anyhow::Result;
use axum::Router;

pub use crate::config::Config;

#[tokio::main]
async fn main() -> Result<()> {
    // Load config
    let config = Config::load()?;

    // Load modules
    let legacy = legacy::route().await?;
    let minion = minion::setup().await?;

    // Create app router
    let app = Router::new()
        .merge(legacy)
        .nest("/minion", minion); 

    // Start the server based on config
    server::serve(app, &config).await?;
    Ok(())
}