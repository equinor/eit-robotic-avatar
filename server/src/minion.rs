mod model {
    use std::sync::Arc;

    use time::OffsetDateTime;

    #[allow(dead_code)]
    enum Status {
        Offline, // Minion robot is offline have made no connection to the server.
        NotReady, // Minion robot have connected but is not ready. (Its staring or stopping)
        Running, // Ready and willing.
        Timeout, // The minion robot have missed a few check ins. (It should have stopped moving.)  
    }

    #[allow(dead_code)]
    struct Head {
        rx: f64,
        ry: f64,
        rz: f64,
    }

    #[allow(dead_code)]
    struct Drive { 
        speed: f64,
        turn: f64,
    }

    #[allow(dead_code)]
    struct LogEntry {
        timestamp: OffsetDateTime,
        msg: String,
    }

    #[allow(dead_code)]
    enum Stream{
        Disconnected,
        MinionOffer(String),
        HeadSetAnswer(String),
        Connecting(),
        Connected
    }

    #[allow(dead_code)]
    #[derive(Clone)]
    pub struct Minion {
        status: Arc<Status>,
        movement: Arc<(Head, Drive)>,
        log: Arc<Vec<LogEntry>>,
        stream: Arc<Stream>,
    }

    impl Default for Minion {
        fn default() -> Minion {
            Minion { 
                status: Arc::new(Status::Offline),
                movement: Arc::new((Head { rx: 0.0, ry: 0.0, rz: 0.0 }, Drive { speed: 0.0, turn: 0.0 })),
                log: Arc::new(Vec::new()),
                stream: Arc::new(Stream::Disconnected),
            }
        }
    }

}

mod front{
    use anyhow::{Result, Ok};
    use axum::{Router, Extension, routing::get, Json};
    use serde::Serialize;

    use super::model::Minion;

    pub async fn setup() -> Result<Router> {
        let minion = Minion::default();

        let router = Router::new()
            .route("/", get(status))
            .layer(Extension(minion));

        Ok(router)
    }

    #[derive(Serialize)]
    struct Status {

    }

    async fn status(Extension(_minion): Extension<Minion>) -> Json<Status> {
        Json(Status {})
    }
}

pub use front::setup;
