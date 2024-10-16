#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

mod cal;
mod ros_node;
mod ui;

use std::{io::Write, sync::Arc};
use tokio::sync::Mutex;
use tokio::time::{sleep, Duration};

fn main() {
    let rt = tokio::runtime::Runtime::new().expect("Unable to create Runtime");
    let _enter = rt.enter();

    std::thread::spawn(move || {
        rt.block_on(async {
            loop {
                tokio::time::sleep(Duration::from_secs(3600)).await;
            }
        })
    });

    let (mut rn, mut node, imu_rx, mag_rx) = ros_node::Node::new();

    let ros_task = tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(1));
    });

    ui::init(imu_rx, mag_rx);
}
