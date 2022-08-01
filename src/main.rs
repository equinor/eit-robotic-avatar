use std::{net::SocketAddr, sync::Arc};

use axum::{
    routing::{get, post},
    Router, response::{IntoResponse, Response}, body::{boxed, Full},
};
use axum_server::tls_rustls::RustlsConfig;
use hyper::{StatusCode, Uri, header};
use parking_lot::{const_mutex, Mutex};
use rcgen::generate_simple_self_signed;
use rust_embed::RustEmbed;
use tokio::net::UdpSocket;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Setup udp
    let sock = Arc::new(UdpSocket::bind("0.0.0.0:0".parse::<SocketAddr>().unwrap()).await?);
    let remote_addr = "10.52.118.113:6666".parse::<SocketAddr>().unwrap();
    sock.connect(remote_addr).await?;

    // build our application with a single route
    let app = Router::new()
        .route("/", get(index_handler))
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
        )
        .fallback(get(static_handler));

    let subject_alt_names: &[_] = &[
        "127.0.0.1".to_string(),
        "10.52.120.59".to_string(),
        "10.52.115.15".to_string(),
    ];
    let cert = generate_simple_self_signed(subject_alt_names).unwrap();
    let config = RustlsConfig::from_der(
        vec![cert.serialize_der().unwrap()],
        cert.serialize_private_key_der(),
    )
    .await
    .unwrap();

    // run it with hyper on localhost:3000
    axum_server::bind_rustls("0.0.0.0:3000".parse().unwrap(), config)
        .serve(app.into_make_service())
        .await
        .unwrap();
    Ok(())
}

async fn index_handler() -> impl IntoResponse {
    static_handler("/index.html".parse::<Uri>().unwrap()).await
}

async fn static_handler(uri: Uri) -> impl IntoResponse {
    let path = uri.path().trim_start_matches('/').to_string();

    StaticFile(path)
}

#[derive(RustEmbed)]
#[folder = "dist/"]
struct Asset;

pub struct StaticFile<T>(pub T);

impl<T> IntoResponse for StaticFile<T>
where
    T: Into<String>,
{
    fn into_response(self) -> Response {
        let path = self.0.into();

        match Asset::get(path.as_str()) {
            Some(content) => {
                let body = boxed(Full::from(content.data));
                let mime = mime_guess::from_path(path).first_or_octet_stream();
                Response::builder()
                    .header(header::CONTENT_TYPE, mime.as_ref())
                    .body(body)
                    .unwrap()
            }
            None => Response::builder()
                .status(StatusCode::NOT_FOUND)
                .header(header::CONTENT_TYPE, "text/html")
                .body(boxed(Full::from("<h1>404</h1><p>Not Found</p>")))
                .unwrap(),
        }
    }
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
