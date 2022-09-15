use axum::{
    body::{boxed, Full},
    response::{IntoResponse, Response},
    routing::get,
    Router,
};
use axum_server::tls_rustls::RustlsConfig;
use hyper::{header, StatusCode, Uri};
use network_interface::{Addr, NetworkInterface, NetworkInterfaceConfig};
use rcgen::generate_simple_self_signed;
use rust_embed::RustEmbed;
use anyhow::Result;

use crate::config::Config;

pub async fn serve(app: Router, config: &Config) -> Result<()> {
    let routes = Router::new()
        .route("/", get(index_handler))
        .fallback(get(static_handler));
    
    let routes = routes.merge(app);

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