use std::process::{Command, self};

fn main() {
    println!("cargo:rerun-if-changed=../www");

    let status = Command::new("yarn")
        .arg("parcel")
        .arg("build")
        .status().unwrap();

    process::exit(status.code().unwrap_or(0));
}