use std::{sync::Arc, net::SocketAddr};

use crate::config::Config;
use anyhow::Result;
use axum::{routing::{post, get}, Router};
use parking_lot::{Mutex, const_mutex};
use tokio::net::UdpSocket;

pub async fn route(config: &Config) -> Result<Router> {
    let sock = Arc::new(UdpSocket::bind("0.0.0.0:0".parse::<SocketAddr>()?).await?);
    println!("Connecting to minion robot at: {}", config.minion_udp_address);
    sock.connect(config.minion_udp_address).await?;

    let router = Router::new()
        .route("/post_offer", post(post_offer))
        .route("/get_offer", get(get_offer))
        .route("/post_answer", post(post_answer))
        .route("/get_answer", get(get_answer))
        .route(
            "/post_tracking",
            post({
                let sock = Arc::clone(&sock);
                move |body| post_tracking(body, Arc::clone(&sock))
            }),
        );
    Ok(router)
}

static OFFER: Mutex<String> = const_mutex(String::new());

async fn post_offer(body: String) {
    ANSWER.lock().clear();
    let mut offer = OFFER.lock();
    offer.clear();
    offer.push_str(&body);
}

async fn get_offer() -> String {
    let offer = OFFER.lock();
    if offer.is_empty() {
        "{}".to_string()
    } else {
        offer.clone()
    }
}

static ANSWER: Mutex<String> = const_mutex(String::new());

async fn post_answer(body: String) {
    OFFER.lock().clear();
    let mut answer = ANSWER.lock();
    answer.clear();
    answer.push_str(&body);
}

async fn get_answer() -> String {
    let answer = ANSWER.lock();
    if answer.is_empty() {
        "{}".to_string()
    } else {
        answer.clone()
    }
}

async fn post_tracking(body: String, sock: Arc<UdpSocket>) {
    sock.send(body.as_bytes()).await.unwrap();
    //println!("{}", body)
}