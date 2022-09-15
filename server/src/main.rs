use std::{net::SocketAddr, sync::Arc};


use axum::{
    body::{boxed, Full},
    response::{IntoResponse, Response},
    routing::{get, post},
    Router,
};
use axum_server::tls_rustls::RustlsConfig;
use hyper::{header, StatusCode, Uri};
use network_interface::{Addr, NetworkInterface, NetworkInterfaceConfig};
use parking_lot::{const_mutex, Mutex};
use rcgen::generate_simple_self_signed;
use rust_embed::RustEmbed;
use tokio::net::UdpSocket;
use anyhow::Result;

use crate::config::Config;

#[tokio::main]
async fn main() -> Result<()> {
    // Load config
    let config = Config::load()?;

    // Setup udp
    let sock = Arc::new(UdpSocket::bind("0.0.0.0:0".parse::<SocketAddr>()?).await?);
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
    serve(app, &config).await?;
    Ok(())
}

mod config {
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
}

async fn serve(routes: Router, config: &Config) -> Result<()> {
    if config.https {
        // Build a development cert based on networks found.
        let network_interfaces = NetworkInterface::show()?;
        let mut subject_alt_names: Vec<_> = network_interfaces
            .iter()
            .filter_map(|n| match n.addr? {
                Addr::V4(addr) => Some(addr.ip.to_string()),
                Addr::V6(_) => None,
            })
            .collect();

        // Add names from override.
        let mut names = config.extra_cert_names.clone();
        subject_alt_names.append(&mut names);
        println!("Creating cert for: {}", subject_alt_names.join(" "));

        let cert = generate_simple_self_signed(subject_alt_names)?;
        let tls_config = RustlsConfig::from_der(
            vec![cert.serialize_der()?],
            cert.serialize_private_key_der(),
        )
        .await?;

        println!("Binding server to {} using HTTPS", config.bind_address);
        Ok(axum_server::bind_rustls(config.bind_address, tls_config)
            .serve(routes.into_make_service())
            .await?)
    } else {
        println!("Binding server to {} using HTTP", config.bind_address);
        Ok(axum_server::bind(config.bind_address)
            .serve(routes.into_make_service())
            .await?)
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
