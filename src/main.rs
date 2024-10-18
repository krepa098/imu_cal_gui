#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

use serial_data_provider::SerialDataProvider;

mod cal;
mod data_provider;
#[cfg(feature = "ros")]
mod ros_data_provider;
mod serial_data_provider;
mod ui;

fn main() {
    let rt = tokio::runtime::Runtime::new().expect("Unable to create Runtime");
    let _enter = rt.enter();

    let (provider, imu_rx, mag_rx) = {
        #[cfg(feature = "ros")]
        {
            let (provider, mut node, imu_rx, mag_rx) = ros_data_provider::Node::new();

            std::thread::spawn(move || {
                rt.block_on(async {
                    loop {
                        node.spin_once(std::time::Duration::from_millis(1));
                    }
                })
            });

            (Box::new(provider), imu_rx, mag_rx)
        }
        #[cfg(not(feature = "ros"))]
        {
            let (provider, imu_rx, mag_rx) = SerialDataProvider::new();
            (provider, imu_rx, mag_rx)
        }
    };

    ui::init(provider, imu_rx, mag_rx).unwrap();
}
