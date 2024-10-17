use std::sync::mpsc::Receiver;

use crate::cal::*;
use eframe::egui::{self};
use eframe::egui::{Style, Visuals};
use egui::menu;
use egui_modal::Modal;
use egui_plot::Legend;

pub fn init(imu_rx: Receiver<ImuData>, mag_rx: Receiver<MagData>) -> eframe::Result {
    env_logger::init();
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1600.0, 900.0]),
        ..Default::default()
    };
    eframe::run_native(
        "IMU Calibration GUI",
        options,
        Box::new(|cc| {
            let style = Style {
                visuals: Visuals::dark(),
                ..Style::default()
            };
            cc.egui_ctx.set_style(style);

            Ok(Box::new(MyApp::new(imu_rx, mag_rx)))
        }),
    )
}

struct MyApp {
    imu_rx: Receiver<ImuData>,
    mag_rx: Receiver<MagData>,
    cal: Cal,
    collect_mag: bool,
    collect_gyro: bool,
    collect_acc: bool,
    show_mag: bool,
    show_gyro: bool,
    show_acc: bool,
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
            show_gyro: true,
            show_acc: true,
            show_mag: true,
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

            let info = cal_data.as_json_string();

            modal_cal_data.title(ui, "Calibration Results");
            modal_cal_data.frame(ui, |ui| {
                modal_cal_data.body(ui, "");
                ui.heading("gyro offset");
                egui::Grid::new("grid_gyro_offset")
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label(format!("{:.4e}", cal_data.gyro_offset.x));
                        ui.label(format!("{:.4e}", cal_data.gyro_offset.y));
                        ui.label(format!("{:.4e}", cal_data.gyro_offset.z));
                    });
                ui.separator();
                ui.heading("accel offset");
                egui::Grid::new("grid_acc_offset")
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label(format!("{:.4e}", cal_data.acc_offset.x));
                        ui.label(format!("{:.4e}", cal_data.acc_offset.y));
                        ui.label(format!("{:.4e}", cal_data.acc_offset.z));
                    });
                ui.separator();
                ui.heading("accel scale");
                egui::Grid::new("grid_acc_scale")
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label(format!("{:.4e}", cal_data.acc_scale.x));
                        ui.label(format!("{:.4e}", cal_data.acc_scale.y));
                        ui.label(format!("{:.4e}", cal_data.acc_scale.z));
                    });
                ui.separator();
                ui.heading("mag soft iron transform");
                egui::Grid::new("grid_soft_iron")
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label(format!("{:.4e}", cal_data.soft_iron_transf[(0, 0)]));
                        ui.label(format!("{:.4e}", cal_data.soft_iron_transf[(0, 1)]));
                        ui.label(format!("{:.4e}", cal_data.soft_iron_transf[(0, 2)]));
                        ui.end_row();
                        ui.label(format!("{:.4e}", cal_data.soft_iron_transf[(1, 0)]));
                        ui.label(format!("{:.4e}", cal_data.soft_iron_transf[(1, 1)]));
                        ui.label(format!("{:.4e}", cal_data.soft_iron_transf[(1, 2)]));
                        ui.end_row();
                        ui.label(format!("{:.4e}", cal_data.soft_iron_transf[(2, 0)]));
                        ui.label(format!("{:.4e}", cal_data.soft_iron_transf[(2, 1)]));
                        ui.label(format!("{:.4e}", cal_data.soft_iron_transf[(2, 2)]));
                    });
                ui.separator();
                ui.heading("mag hard iron bias");
                egui::Grid::new("grid_hard_iron")
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label(format!("{:.4e}", cal_data.hard_iron_bias.x));
                        ui.label(format!("{:.4e}", cal_data.hard_iron_bias.y));
                        ui.label(format!("{:.4e}", cal_data.hard_iron_bias.z));
                    });
            });
            modal_cal_data.buttons(ui, |ui| {
                if modal_cal_data.caution_button(ui, "close").clicked() {
                    // After clicking, the modal is automatically closed
                };
                if ui.button("🗐 copy as json").clicked() {
                    ui.output_mut(|p| p.copied_text = info);
                };
            });
        });

        egui::SidePanel::left("left_panel").show(ctx, |ui| {
            menu::bar(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("🗁 Open").clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("data", &["json"])
                            .pick_file()
                        {
                            self.cal.load_from_file(path);
                            ui.close_menu();
                        }
                    }
                    if ui.button("🖴 Save").clicked() {
                        if let Some(mut path) = rfd::FileDialog::new()
                            .add_filter("data", &["json"])
                            .save_file()
                        {
                            path.set_extension("json");
                            self.cal.save_to_file(path);
                            ui.close_menu();
                        }
                    }
                });
            });
            ui.separator();

            ui.heading("Data Sources");
            ui.toggle_value(&mut self.collect_gyro, "Gyro");
            ui.toggle_value(&mut self.collect_acc, "Accel");
            ui.toggle_value(&mut self.collect_mag, "Mag");
            ui.separator();

            ui.heading("Clear Data");
            egui::Grid::new("grid")
                .num_columns(2)
                .striped(true)
                .show(ui, |ui| {
                    ui.label("Gyro");
                    if ui.button("🗑").clicked() {
                        self.cal.clear_gyro_measurements();
                    }
                    ui.end_row();
                    ui.label("Accel");
                    if ui.button("🗑").clicked() {
                        self.cal.clear_accel_measurements();
                    }
                    ui.end_row();
                    ui.label("Mag");
                    if ui.button("🗑").clicked() {
                        self.cal.clear_mag_measurements();
                    }
                });

            ui.separator();

            ui.heading("Filter");
            ui.checkbox(&mut self.filter_standstill, "Await standstill");
            ui.separator();

            ui.heading("Calibration");
            if ui.button("Calibrate now").clicked() {
                self.cal_data = Some(self.cal.calibrate());
                modal_cal_data.open();
            }
            if let Some(cal_data) = self.cal_data {
                if ui.button("🗐 copy as json").clicked() {
                    ui.output_mut(|w| w.copied_text = cal_data.as_json_string())
                }
            }
            ui.separator();

            ui.heading("View");
            ui.toggle_value(&mut self.show_gyro, "Gyro");
            ui.toggle_value(&mut self.show_acc, "Accel");
            ui.toggle_value(&mut self.show_mag, "Mag");

            ui.separator();
        });

        egui::CentralPanel::default().show(ctx, |_ui| {
            // gyro plot
            if self.show_gyro {
                plot_window(ctx, "Gyro", "rad/s", self.cal.gyro_measurements());

                if self.cal_data.is_some() {
                    let measurements_with_cal = self.cal.gyro_measurements_with_cal();
                    plot_window(ctx, "Gyro (calibrated)", "rad/s", &measurements_with_cal);
                }
            }

            // acc plot
            if self.show_acc {
                plot_window(ctx, "Accel", "m/s²", self.cal.acc_measurements());

                if self.cal_data.is_some() {
                    let measurements_with_cal = self.cal.acc_measurements_with_cal();
                    plot_window(ctx, "Accel (calibrated)", "m/s²", &measurements_with_cal);
                }
            }

            // mag plot
            if self.show_mag {
                plot_window(ctx, "Mag", "µT", self.cal.mag_measurements());

                if self.cal_data.is_some() {
                    let measurements_with_cal = self.cal.mag_measurements_with_cal();
                    plot_window(ctx, "Mag (calibrated)", "µT", &measurements_with_cal);
                }
            }
        });

        ctx.request_repaint();
    }
}

fn plot_window(
    ctx: &egui::Context,
    window_title: &str,
    unit: &str,
    data: &Vec<nalgebra::Vector3<f64>>,
) {
    egui::Window::new(window_title).show(ctx, |ui| {
        egui_plot::Plot::new(window_title)
            .allow_zoom(true)
            .allow_drag(true)
            .allow_scroll(false)
            .allow_boxed_zoom(false)
            .data_aspect(1.0)
            .view_aspect(1.0)
            .x_axis_label(unit)
            .y_axis_label(unit)
            .legend(Legend::default())
            .show(ui, |plot_ui| {
                plot_ui.points(
                    egui_plot::Points::new(data.iter().map(|p| [p.x, p.y]).collect::<Vec<_>>())
                        .name("XY"),
                );

                plot_ui.points(
                    egui_plot::Points::new(data.iter().map(|p| [p.x, p.z]).collect::<Vec<_>>())
                        .name("XZ"),
                );

                plot_ui.points(
                    egui_plot::Points::new(data.iter().map(|p| [p.y, p.z]).collect::<Vec<_>>())
                        .name("YZ"),
                );
            });
    });
}
