use crate::data_provider::*;
use bytes::BytesMut;
use core::str;
use eframe::egui;
use futures::prelude::*;
use nalgebra::vector;
use std::sync::mpsc::{Receiver, Sender};
use stream_cancel::StreamExt;
use tokio_serial::{SerialPort, SerialPortBuilderExt};
use tokio_util::codec::Decoder;

const BAUDRATES: [u32; 9] = [
    4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
];

pub struct SerialDataProvider {
    imu_tx: Sender<ImuData>,
    mag_tx: Sender<MagData>,
    serial_port_info: Option<tokio_serial::SerialPortInfo>,
    baud_rate: u32,
    trigger: Option<stream_cancel::Trigger>,
}

impl SerialDataProvider {
    pub fn new() -> (Box<Self>, Receiver<ImuData>, Receiver<MagData>) {
        let (imu_tx, imu_rx) = std::sync::mpsc::channel();
        let (mag_tx, mag_rx) = std::sync::mpsc::channel();

        (
            Box::new(Self {
                imu_tx,
                mag_tx,
                serial_port_info: None,
                baud_rate: 115200,
                trigger: None,
            }),
            imu_rx,
            mag_rx,
        )
    }
}

impl DataProviderUi for SerialDataProvider {
    fn show(&mut self, ui: &mut eframe::egui::Ui) {
        ui.heading("Serial");
        if self.trigger.is_some() {
            ui.label(format!(
                "'{}' 8-N-1",
                self.serial_port_info.as_ref().map_or("", |p| &p.port_name)
            ));
        } else {
            egui::ComboBox::new("ports", "Port")
                .selected_text(self.serial_port_info.as_ref().map_or("", |p| &p.port_name))
                .show_ui(ui, |ui| {
                    for port in tokio_serial::available_ports().unwrap() {
                        // remove /dev/ttySx.
                        if port.port_name.contains("/dev/ttyS") {
                            continue;
                        }

                        let port_name = port.port_name.clone();
                        ui.selectable_value(
                            &mut self.serial_port_info,
                            Some(port),
                            port_name.clone(),
                        );
                    }
                });
        }

        egui::ComboBox::new("baudrates", "Baud rate")
            .selected_text(format!("{}", self.baud_rate))
            .show_ui(ui, |ui| {
                for baudrate in BAUDRATES {
                    ui.selectable_value(&mut self.baud_rate, baudrate, format!("{baudrate}"));
                }
            });

        if let Some(serial_port_info) = &self.serial_port_info {
            if self.trigger.is_some() {
                if ui.button("Close").clicked() {
                    self.trigger.take();
                }
            } else {
                if ui.button("Open").clicked() {
                    let mut port = tokio_serial::new(&serial_port_info.port_name, self.baud_rate)
                        .data_bits(tokio_serial::DataBits::Eight)
                        .flow_control(tokio_serial::FlowControl::None)
                        .parity(tokio_serial::Parity::None)
                        .stop_bits(tokio_serial::StopBits::One)
                        .open_native_async()
                        .unwrap();
                    port.write_data_terminal_ready(true).unwrap(); // dtr: required for Arduinos to send data
                    println!("Open serial port: {}", serial_port_info.port_name);

                    let (trigger, tripwire) = stream_cancel::Tripwire::new();
                    self.trigger = Some(trigger);

                    let reader = LineCodec.framed(port);

                    let imu_tx = self.imu_tx.clone();
                    let mag_tx = self.mag_tx.clone();

                    tokio::spawn(async move {
                        let mut incoming = reader.take_until_if(tripwire);

                        while let Some(line) = incoming.next().await {
                            if let Ok(line) = line {
                                let mut gyro_x = 0.0;
                                let mut gyro_y = 0.0;
                                let mut gyro_z = 0.0;
                                let mut acc_x = 0.0;
                                let mut acc_y = 0.0;
                                let mut acc_z = 0.0;
                                let mut mag_x = 0.0;
                                let mut mag_y = 0.0;
                                let mut mag_z = 0.0;

                                if scanf::sscanf!(
                                    &line,
                                    "imu {} {} {} {} {} {}\n",
                                    gyro_x,
                                    gyro_y,
                                    gyro_z,
                                    acc_x,
                                    acc_y,
                                    acc_z
                                )
                                .is_ok()
                                {
                                    imu_tx
                                        .send(ImuData {
                                            lin_acc: vector![acc_x, acc_y, acc_z],
                                            ang_vel: vector![gyro_x, gyro_y, gyro_z],
                                        })
                                        .ok();
                                }

                                if scanf::sscanf!(&line, "mag {} {} {}\n", mag_x, mag_y, mag_z,)
                                    .is_ok()
                                {
                                    mag_tx
                                        .send(MagData {
                                            field: vector![mag_x, mag_y, mag_z],
                                        })
                                        .ok();
                                }
                            }
                        }
                    });
                }
            }
        }
    }
}

struct LineCodec;

impl Decoder for LineCodec {
    type Item = String;
    type Error = std::io::Error;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        let newline = src.as_ref().iter().position(|b| *b == b'\n');
        if let Some(n) = newline {
            let line = src.split_to(n + 1);
            return match str::from_utf8(line.as_ref()) {
                Ok(s) => Ok(Some(s.to_string())),
                Err(_) => Err(std::io::Error::new(
                    std::io::ErrorKind::Other,
                    "Invalid String",
                )),
            };
        }
        Ok(None)
    }
}
