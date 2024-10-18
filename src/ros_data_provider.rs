use std::sync::mpsc::Receiver;

use crate::data_provider::DataProviderUi;
use crate::data_provider::{ImuData, MagData};
use futures::{future, StreamExt};
use r2r::sensor_msgs;
use r2r::QosProfile;

pub struct Node {}

impl Node {
    pub fn new() -> (Self, r2r::Node, Receiver<ImuData>, Receiver<MagData>) {
        let (imu_tx, imu_rx) = std::sync::mpsc::channel();
        let (mag_tx, mag_rx) = std::sync::mpsc::channel();

        let ctx = r2r::Context::create().unwrap();
        let mut node = r2r::Node::create(ctx, "imu_cal", "").unwrap();

        let imu_sub = node
            .subscribe::<sensor_msgs::msg::Imu>("/robot/rcu_com/imu", QosProfile::default())
            .unwrap();
        let mag_sub = node
            .subscribe::<sensor_msgs::msg::MagneticField>(
                "/robot/rcu_com/mag",
                QosProfile::default(),
            )
            .unwrap();

        tokio::task::spawn(async move {
            imu_sub
                .for_each(move |msg| {
                    imu_tx
                        .send(ImuData {
                            lin_acc: nalgebra::Vector3::new(
                                msg.linear_acceleration.x,
                                msg.linear_acceleration.y,
                                msg.linear_acceleration.z,
                            ),
                            ang_vel: nalgebra::Vector3::new(
                                msg.angular_velocity.x,
                                msg.angular_velocity.y,
                                msg.angular_velocity.z,
                            ),
                        })
                        .unwrap();
                    future::ready(())
                })
                .await
        });

        tokio::task::spawn(async move {
            mag_sub
                .for_each(move |msg| {
                    mag_tx
                        .send(MagData {
                            field: nalgebra::Vector3::new(
                                msg.magnetic_field.x,
                                msg.magnetic_field.y,
                                msg.magnetic_field.z,
                            ),
                        })
                        .unwrap();
                    future::ready(())
                })
                .await
        });

        (Self {}, node, imu_rx, mag_rx)
    }
}

impl DataProviderUi for Node {
    fn show(&mut self, ui: &mut eframe::egui::Ui) {
        ui.heading("Ros Topics");
        ui.label("/imu");
        ui.label("/mag");
    }
}
