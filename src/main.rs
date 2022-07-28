use axum::{
    routing::{get, get_service, post},
    Router,
};
use axum_server::tls_rustls::RustlsConfig;
use hyper::StatusCode;
use parking_lot::{Mutex, const_mutex};
use rcgen::generate_simple_self_signed;
use tower_http::services::ServeDir;

#[tokio::main]
async fn main() {
    // build our application with a single route
    let app = Router::new()
        .route("/post_offer", post(post_offer))
        .route("/get_offer", get(get_offer))
        .route("/post_answer", post(post_answer))
        .route("/get_answer", get(get_answer))
        .route("/post_tracking", post(post_tracking))
        .fallback(get_service(ServeDir::new("./dist")).handle_error(
            |error: std::io::Error| async move {
                (
                    StatusCode::INTERNAL_SERVER_ERROR,
                    format!("Unhandled internal error: {}", error),
                )
            },
        ));

    let subject_alt_names:&[_] = &["127.0.0.1".to_string(),
	"10.52.120.59".to_string(), "10.52.115.15".to_string()];
    let cert = generate_simple_self_signed(subject_alt_names).unwrap();
    let config = RustlsConfig::from_der(vec![cert.serialize_der().unwrap()], cert.serialize_private_key_der()).await.unwrap();

    // run it with hyper on localhost:3000
    axum_server::bind_rustls("0.0.0.0:3000".parse().unwrap(), config)
        .serve(app.into_make_service())
        .await
        .unwrap();
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

async fn get_answer() -> String{
    let answer = ANSWER.lock();
    if answer.is_empty() {
        "{}".to_string()
    } else {
        answer.clone()
    }
}

async fn post_tracking(body: String){
    println!("{}", body)
}
