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
struct Minion {
    status: Status,
    head: Head,
    drive: Drive,
    log: Vec<LogEntry>,
    stream: Stream,
}