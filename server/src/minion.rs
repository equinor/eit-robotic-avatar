mod model {
    use std::sync::Arc;

    use common::{Head, Drive};
    use parking_lot::Mutex;
    use time::OffsetDateTime;

    #[allow(dead_code)]
    enum Status {
        Offline, // Minion robot is offline have made no connection to the server.
        NotReady, // Minion robot have connected but is not ready. (Its staring or stopping)
        Running, // Ready and willing.
        Timeout, // The minion robot have missed a few check ins. (It should have stopped moving.)  
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
        movement: Arc<Mutex<(Head, Drive)>>,
        log: Arc<Vec<LogEntry>>,
        stream: Arc<Stream>,
    }

    impl Default for Minion {
        fn default() -> Minion {
            Minion { 
                status: Arc::new(Status::Offline),
                movement: Arc::default(),
                log: Arc::new(Vec::new()),
                stream: Arc::new(Stream::Disconnected),
            }
        }
    }

    impl Minion {
        pub fn movement(&self) -> (Head, Drive) {
            *self.movement.lock()
        }

        pub fn movement_set(&mut self, head: Head, drive: Drive ){
            *self.movement.lock() = (head, drive)
        }
    }

}

mod front{
    use anyhow::{Result, Ok};
    use axum::{Router, Extension, routing::get, Json};
    use common::Tracking;
    use serde::Serialize;

    use super::model::Minion;

    pub async fn setup() -> Result<Router> {
        let minion = Minion::default();

        let router = Router::new()
            .route("/", get(status))
            .route("/tracking", get(tracking_get).post(tracking_post))
            .layer(Extension(minion));

        Ok(router)
    }

    #[derive(Serialize)]
    struct Status {

    }

    async fn status(Extension(_minion): Extension<Minion>) -> Json<Status> {
        Json(Status {})
    }

    async fn tracking_get(Extension(minion): Extension<Minion>) -> Json<Tracking> {
        let (head, drive) = minion.movement();
        Json(Tracking{head, drive})
    }

    async fn tracking_post(Extension(mut minion): Extension<Minion>, Json(tracking): Json<Tracking>) {
        println!("{:?}", tracking);
        minion.movement_set(tracking.head, tracking.drive)
    }
}

pub use front::setup;
