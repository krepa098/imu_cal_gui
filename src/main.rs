#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

use std::time::Duration;

use serial_data_provider::SerialDataProvider;

mod cal;
mod data_provider;
mod ros_node;
mod serial_data_provider;
mod ui;

fn main() {
    let rt = tokio::runtime::Runtime::new().expect("Unable to create Runtime");
    let _enter = rt.enter();

    // let (_rn, mut node, imu_rx, mag_rx) = ros_node::Node::new();

    // std::thread::spawn(move || {
    //     rt.block_on(async {
    //         loop {
    //             node.spin_once(Duration::from_millis(1));
    //         }
    //     })
    // });

    let (provider, imu_rx, mag_rx) = SerialDataProvider::new();

    ui::init(provider, imu_rx, mag_rx).unwrap();
}
