use std::{net::SocketAddr, sync::Arc};

use axum::{
    routing::{get, post},
    Router, response::{IntoResponse, Response}, body::{boxed, Full},
};
use axum_server::tls_rustls::RustlsConfig;
use hyper::{StatusCode, Uri, header};
use network_interface::{NetworkInterface, NetworkInterfaceConfig, Addr};
use parking_lot::{const_mutex, Mutex};
use rcgen::generate_simple_self_signed;
use rust_embed::RustEmbed;
use tokio::{net::UdpSocket, io};

struct Config {
    minion_udp_address: SocketAddr,
    extra_cert_names: Vec<String>,
    bind_address: SocketAddr,
    https: bool,
}

impl Default for Config {
    fn default() -> Self {
        Config { 
            minion_udp_address: "10.52.119.109:6666".parse().unwrap(),
            extra_cert_names: vec!["10.52.115.13".to_string()],
            bind_address: "0.0.0.0:3000".parse().unwrap(),
            https: true,
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Load config
    let config = Config::default();

    // Setup udp
    let sock = Arc::new(UdpSocket::bind("0.0.0.0:0".parse::<SocketAddr>().unwrap()).await?);
    sock.connect(config.minion_udp_address).await?;

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

    // Start the server based on config
    serve(app, &config).await.unwrap();
    Ok(())
}

async fn serve(routes: Router, config: &Config ) -> io::Result<()>{
    if config.https {
        // Build a development cert based on networks found.      
        let network_interfaces = NetworkInterface::show().unwrap();
        let mut subject_alt_names: Vec<_> = network_interfaces.iter().filter_map(| n | {
            match n.addr? {
                Addr::V4(addr) => Some(addr.ip.to_string()),
                Addr::V6(_) =>None
            }
        }).collect();

        // Add names from override.
        let mut names = config.extra_cert_names.clone();
        subject_alt_names.append(&mut names);
        println!("Creating cert for: {}", subject_alt_names.join(" "));


        let cert = generate_simple_self_signed(subject_alt_names).unwrap();
        let tls_config = RustlsConfig::from_der(
            vec![cert.serialize_der().unwrap()],
            cert.serialize_private_key_der(),
        )
        .await
        .unwrap();

        println!("Binding server to {} using HTTPS", config.bind_address);
        axum_server::bind_rustls("0.0.0.0:3000".parse().unwrap(), tls_config)
            .serve(routes.into_make_service()).await

    } else {
        println!("Binding server to {} using HTTP", config.bind_address);
        axum_server::bind(config.bind_address)
        .serve(routes.into_make_service()).await
    }
}

async fn index_handler() -> impl IntoResponse {
    static_handler("/index.html".parse::<Uri>().unwrap()).await
}

async fn static_handler(uri: Uri) -> impl IntoResponse {
    let path = uri.path().trim_start_matches('/').to_string();

    StaticFile(path)
}

#[derive(RustEmbed)]
#[folder = "../dist/"]
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
