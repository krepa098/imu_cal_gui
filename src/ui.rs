use std::sync::mpsc::Receiver;

use crate::cal::*;
use eframe::egui::{self, Color32};
use egui_modal::Modal;
use egui_plot::Legend;
use r2r::sensor_msgs;

pub fn init(imu_rx: Receiver<ImuData>, mag_rx: Receiver<MagData>) -> eframe::Result {
    env_logger::init();
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1024.0, 768.0]),
        ..Default::default()
    };
    eframe::run_native(
        "IMU Calibration GUI",
        options,
        Box::new(|cc| Ok(Box::new(MyApp::new(imu_rx, mag_rx)))),
    )
}

struct MyApp {
    imu_rx: Receiver<ImuData>,
    mag_rx: Receiver<MagData>,
    cal: Cal,
    collect_mag: bool,
    collect_gyro: bool,
    collect_acc: bool,
    filter_standstill: bool,
    cal_data: Option<CalData>,
}

impl MyApp {
    pub fn new(imu_rx: Receiver<ImuData>, mag_rx: Receiver<MagData>) -> Self {
        Self {
            imu_rx,
            mag_rx,
            cal: Cal::new(),
            collect_mag: false,
            collect_gyro: true,
            collect_acc: false,
            filter_standstill: false,
            cal_data: None,
        }
    }
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        while let Ok(msg) = self.imu_rx.try_recv() {
            if self.collect_acc {
                if self.filter_standstill {
                    self.cal.add_acc_measurement_still(msg.lin_acc);
                } else {
                    self.cal.add_acc_measurement(msg.lin_acc);
                }
            }

            if self.collect_gyro {
                if self.filter_standstill {
                    self.cal.add_gyro_measurement_still(msg.ang_vel);
                } else {
                    self.cal.add_gyro_measurement(msg.ang_vel);
                }
            }
        }

        while let Ok(msg) = self.mag_rx.try_recv() {
            if self.collect_mag {
                self.cal.add_mag_measurement(msg.field);
            }
        }

        let modal_cal_data = Modal::new(ctx, "cal_data");
        // What goes inside the modal
        modal_cal_data.show(|ui| {
            let cal_data = self.cal_data.as_ref().unwrap();
            let info = format!(
                "Gyro offset: [{:+e}, {:+e}, {:+e}]\nAcc offset: [{:+e}, {:+e}, {:+e}]\nAcc scale: [{:+e}, {:+e}, {:+e}]",
                cal_data.gyro_offset.x,
                cal_data.gyro_offset.y,
                cal_data.gyro_offset.z,
                cal_data.acc_offset.x,
                cal_data.acc_offset.y,
                cal_data.acc_offset.z,
                cal_data.acc_scale.x,
                cal_data.acc_scale.y,
                cal_data.acc_scale.z
            );

            modal_cal_data.title(ui, "Calibration Data");
            modal_cal_data.frame(ui, |ui| {
                modal_cal_data.body(ui, info.clone());
            });
            modal_cal_data.buttons(ui, |ui| {
                if modal_cal_data.caution_button(ui, "close").clicked() {
                    // After clicking, the modal is automatically closed
                };
                if ui.button("🗐 copy to clipboard").clicked() {
                    ui.output_mut(|p| p.copied_text = info);
                };
            });
        });

        egui::SidePanel::left("left_panel").show(ctx, |ui| {
            ui.heading("Data Sources");
            if ui.toggle_value(&mut self.collect_gyro, "Gyro").changed() {};
            if ui.toggle_value(&mut self.collect_acc, "Accel").changed() {};
            if ui.toggle_value(&mut self.collect_mag, "Mag").changed() {};
            ui.separator();

            ui.heading("Clear Data");
            if ui.button("🗑 gyro").clicked() {
                self.cal.clear_gyro_measurements();
            }
            if ui.button("🗑 accel").clicked() {
                self.cal.clear_accel_measurements();
            }
            if ui.button("🗑 mag").clicked() {
                self.cal.clear_mag_measurements();
            }
            ui.separator();

            ui.heading("Filter");
            if ui
                .checkbox(&mut self.filter_standstill, "Standstill")
                .changed()
            {
                self.cal.clear_accel_measurements();
                self.cal.clear_mag_measurements();
                self.cal.clear_gyro_measurements();
            };
            ui.separator();

            ui.heading("Calibration");
            if ui.button("Calibrate now").clicked() {
                self.cal_data = Some(self.cal.calibrate());
                modal_cal_data.open();
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            // gyro plot
            if self.collect_gyro {
                egui::Window::new("Gyro").show(ctx, |ui| {
                    egui_plot::Plot::new("gyro_plot")
                        .allow_zoom(true)
                        .allow_drag(true)
                        .allow_scroll(false)
                        .allow_boxed_zoom(false)
                        .data_aspect(1.0)
                        .view_aspect(1.0)
                        .x_axis_label("rad/s")
                        .y_axis_label("rad/s")
                        .legend(Legend::default())
                        .show(ui, |plot_ui| {
                            plot_ui.points(
                                egui_plot::Points::new(
                                    self.cal
                                        .gyro_measurements()
                                        .iter()
                                        .map(|p| [p.x, p.y])
                                        .collect::<Vec<_>>(),
                                )
                                .name("XY"),
                            );

                            plot_ui.points(
                                egui_plot::Points::new(
                                    self.cal
                                        .gyro_measurements()
                                        .iter()
                                        .map(|p| [p.x, p.z])
                                        .collect::<Vec<_>>(),
                                )
                                .name("XZ"),
                            );

                            plot_ui.points(
                                egui_plot::Points::new(
                                    self.cal
                                        .gyro_measurements()
                                        .iter()
                                        .map(|p| [p.y, p.z])
                                        .collect::<Vec<_>>(),
                                )
                                .name("YZ"),
                            );

                            let gyro_measurements_with_cal = self.cal.gyro_measurements_with_cal();

                            plot_ui.points(
                                egui_plot::Points::new(
                                    gyro_measurements_with_cal
                                        .iter()
                                        .map(|p| [p.x, p.y])
                                        .collect::<Vec<_>>(),
                                )
                                .name("XY (cal)"),
                            );

                            plot_ui.points(
                                egui_plot::Points::new(
                                    gyro_measurements_with_cal
                                        .iter()
                                        .map(|p| [p.x, p.z])
                                        .collect::<Vec<_>>(),
                                )
                                .name("XZ (cal)"),
                            );

                            plot_ui.points(
                                egui_plot::Points::new(
                                    gyro_measurements_with_cal
                                        .iter()
                                        .map(|p| [p.y, p.z])
                                        .collect::<Vec<_>>(),
                                )
                                .name("YZ (cal)"),
                            );
                        });
                });
            }

            // acc plot
            if self.collect_acc {
                egui::Window::new("Accelerometer").show(ctx, |ui| {
                    egui_plot::Plot::new("acc_plot")
                        .allow_zoom(true)
                        .allow_drag(true)
                        .allow_boxed_zoom(false)
                        .allow_scroll(false)
                        .data_aspect(1.0)
                        .view_aspect(1.0)
                        .x_axis_label("m/s^2")
                        .y_axis_label("m/s^2")
                        .legend(Legend::default())
                        .show(ui, |plot_ui| {
                            plot_ui.points(
                                egui_plot::Points::new(
                                    self.cal
                                        .acc_measurements()
                                        .iter()
                                        .map(|p| [p.x, p.y])
                                        .collect::<Vec<_>>(),
                                )
                                .name("XY"),
                            );

                            plot_ui.points(
                                egui_plot::Points::new(
                                    self.cal
                                        .acc_measurements()
                                        .iter()
                                        .map(|p| [p.x, p.z])
                                        .collect::<Vec<_>>(),
                                )
                                .name("XZ"),
                            );

                            plot_ui.points(
                                egui_plot::Points::new(
                                    self.cal
                                        .acc_measurements()
                                        .iter()
                                        .map(|p| [p.y, p.z])
                                        .collect::<Vec<_>>(),
                                )
                                .name("YZ"),
                            );

                            let acc_measurements_with_cal = self.cal.acc_measurements_with_cal();

                            plot_ui.points(
                                egui_plot::Points::new(
                                    acc_measurements_with_cal
                                        .iter()
                                        .map(|p| [p.x, p.y])
                                        .collect::<Vec<_>>(),
                                )
                                .name("XY (cal)"),
                            );

                            plot_ui.points(
                                egui_plot::Points::new(
                                    acc_measurements_with_cal
                                        .iter()
                                        .map(|p| [p.x, p.z])
                                        .collect::<Vec<_>>(),
                                )
                                .name("XZ (cal)"),
                            );

                            plot_ui.points(
                                egui_plot::Points::new(
                                    acc_measurements_with_cal
                                        .iter()
                                        .map(|p| [p.y, p.z])
                                        .collect::<Vec<_>>(),
                                )
                                .name("YZ (cal)"),
                            );
                        });
                });
            }

            // gyro plot
            if self.collect_mag {
                egui::Window::new("Mag").show(ctx, |ui| {
                    egui_plot::Plot::new("mag_plot")
                        .allow_zoom(true)
                        .allow_drag(true)
                        .allow_scroll(false)
                        .allow_boxed_zoom(false)
                        .data_aspect(1.0)
                        .view_aspect(1.0)
                        .x_axis_label("T")
                        .y_axis_label("T")
                        .legend(Legend::default())
                        .show(ui, |plot_ui| {
                            plot_ui.points(
                                egui_plot::Points::new(
                                    self.cal
                                        .mag_measurements()
                                        .iter()
                                        .map(|p| [p.x, p.y])
                                        .collect::<Vec<_>>(),
                                )
                                .name("XY"),
                            );

                            plot_ui.points(
                                egui_plot::Points::new(
                                    self.cal
                                        .mag_measurements()
                                        .iter()
                                        .map(|p| [p.x, p.z])
                                        .collect::<Vec<_>>(),
                                )
                                .name("XZ"),
                            );

                            plot_ui.points(
                                egui_plot::Points::new(
                                    self.cal
                                        .mag_measurements()
                                        .iter()
                                        .map(|p| [p.y, p.z])
                                        .collect::<Vec<_>>(),
                                )
                                .name("YZ"),
                            );

                            // let gyro_measurements_with_cal = self.cal.gyro_measurements_with_cal();

                            // plot_ui.points(
                            //     egui_plot::Points::new(
                            //         gyro_measurements_with_cal
                            //             .iter()
                            //             .map(|p| [p.x, p.y])
                            //             .collect::<Vec<_>>(),
                            //     )
                            //     .name("XY (cal)"),
                            // );

                            // plot_ui.points(
                            //     egui_plot::Points::new(
                            //         gyro_measurements_with_cal
                            //             .iter()
                            //             .map(|p| [p.x, p.z])
                            //             .collect::<Vec<_>>(),
                            //     )
                            //     .name("XZ (cal)"),
                            // );

                            // plot_ui.points(
                            //     egui_plot::Points::new(
                            //         gyro_measurements_with_cal
                            //             .iter()
                            //             .map(|p| [p.y, p.z])
                            //             .collect::<Vec<_>>(),
                            //     )
                            //     .name("YZ (cal)"),
                            // );
                        });
                });
            }
        });

        ctx.request_repaint();
    }
}
