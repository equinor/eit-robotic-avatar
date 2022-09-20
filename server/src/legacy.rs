use anyhow::Result;
use axum::{routing::{post, get}, Router};
use parking_lot::{Mutex, const_mutex};

pub async fn route() -> Result<Router> {
    let router = Router::new()
        .route("/post_offer", post(post_offer))
        .route("/get_offer", get(get_offer))
        .route("/post_answer", post(post_answer))
        .route("/get_answer", get(get_answer));
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