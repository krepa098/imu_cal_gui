use eframe::egui;
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    pub lin_acc: Vector3<f64>,
    pub ang_vel: Vector3<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct MagData {
    pub field: Vector3<f64>,
}

pub trait DataProviderUi {
    fn show(&mut self, ui: &mut egui::Ui);
}
