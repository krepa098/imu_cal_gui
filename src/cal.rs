const G0: f64 = 9.80665;
const G0_THR: f64 = G0 * 0.75;

#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    pub lin_acc: nalgebra::Vector3<f64>,
    pub ang_vel: nalgebra::Vector3<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct MagData {
    pub field: nalgebra::Vector3<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct CalData {
    pub gyro_offset: nalgebra::Vector3<f64>,
    pub acc_offset: nalgebra::Vector3<f64>,
    pub acc_scale: nalgebra::Vector3<f64>,
}

#[derive(Debug)]
pub struct Cal {
    gyro_points: Vec<nalgebra::Vector3<f64>>,
    acc_points: Vec<nalgebra::Vector3<f64>>,
    mag_points: Vec<nalgebra::Vector3<f64>>,

    acc_points_avg: nalgebra::Vector3<f64>,
    gyro_points_avg: nalgebra::Vector3<f64>,

    cal_data: Option<CalData>,
}

impl Cal {
    pub fn new() -> Self {
        Self {
            gyro_points: vec![],
            acc_points: vec![],
            mag_points: vec![],
            acc_points_avg: Default::default(),
            gyro_points_avg: Default::default(),
            cal_data: None,
        }
    }

    pub fn add_acc_measurement_still(&mut self, data: nalgebra::Vector3<f64>) {
        let alpha = 0.95;
        self.acc_points_avg = self.acc_points_avg * alpha + data * (1.0 - alpha);

        if (self.acc_points_avg - data).norm() < 1e-2 {
            self.acc_points.push(data);
        }
    }

    pub fn add_gyro_measurement_still(&mut self, data: nalgebra::Vector3<f64>) {
        let alpha = 0.98;
        self.gyro_points_avg = self.gyro_points_avg * alpha + data * (1.0 - alpha);

        if (self.gyro_points_avg - data).norm() < 1e-3 {
            self.gyro_points.push(data);
        }
    }

    pub fn add_gyro_measurement(&mut self, data: nalgebra::Vector3<f64>) {
        self.gyro_points.push(data);
    }

    pub fn add_acc_measurement(&mut self, data: nalgebra::Vector3<f64>) {
        self.acc_points.push(data);
    }

    pub fn add_mag_measurement(&mut self, data: nalgebra::Vector3<f64>) {
        self.mag_points.push(data);
    }

    pub fn gyro_measurements(&self) -> &Vec<nalgebra::Vector3<f64>> {
        &self.gyro_points
    }

    pub fn acc_measurements(&self) -> &Vec<nalgebra::Vector3<f64>> {
        &self.acc_points
    }

    pub fn gyro_measurements_with_cal(&self) -> Vec<nalgebra::Vector3<f64>> {
        if let Some(cal_data) = self.cal_data {
            self.gyro_points
                .iter()
                .map(|p| *p - cal_data.gyro_offset)
                .collect::<Vec<_>>()
        } else {
            vec![]
        }
    }

    pub fn acc_measurements_with_cal(&self) -> Vec<nalgebra::Vector3<f64>> {
        if let Some(cal_data) = self.cal_data {
            self.acc_points
                .iter()
                .map(|p| (*p + cal_data.acc_offset).component_mul(&cal_data.acc_scale))
                .collect::<Vec<_>>()
        } else {
            vec![]
        }
    }

    pub fn mag_measurements(&self) -> &Vec<nalgebra::Vector3<f64>> {
        &self.mag_points
    }

    pub fn clear_gyro_measurements(&mut self) {
        self.gyro_points.clear();
    }

    pub fn clear_accel_measurements(&mut self) {
        self.acc_points.clear();
    }

    pub fn clear_mag_measurements(&mut self) {
        self.mag_points.clear();
    }

    pub fn calibrate(&mut self) -> CalData {
        // gyro
        let gyro_offset = {
            let sum_x: f64 = self.gyro_points.iter().map(|p| p.x).sum();
            let sum_y: f64 = self.gyro_points.iter().map(|p| p.y).sum();
            let sum_z: f64 = self.gyro_points.iter().map(|p| p.z).sum();
            let count = self.gyro_points.len() as f64;

            nalgebra::Vector3::new(sum_x / count, sum_y / count, sum_z / count)
        };

        // acc
        let x_p: Vec<f64> = self
            .acc_points
            .iter()
            .map(|p| p.x)
            .filter(|p| *p > G0_THR)
            .collect();
        let x_m: Vec<f64> = self
            .acc_points
            .iter()
            .map(|p| p.x)
            .filter(|p| *p < -G0_THR)
            .collect();
        let y_p: Vec<f64> = self
            .acc_points
            .iter()
            .map(|p| p.y)
            .filter(|p| *p > G0_THR)
            .collect();
        let y_m: Vec<f64> = self
            .acc_points
            .iter()
            .map(|p| p.y)
            .filter(|p| *p < -G0_THR)
            .collect();
        let z_p: Vec<f64> = self
            .acc_points
            .iter()
            .map(|p| p.z)
            .filter(|p| *p > G0_THR)
            .collect();
        let z_m: Vec<f64> = self
            .acc_points
            .iter()
            .map(|p| p.z)
            .filter(|p| *p < -G0_THR)
            .collect();

        let acc_offset = {
            let sx_p = x_p.iter().sum::<f64>() / x_p.len() as f64;
            let sx_m = x_m.iter().sum::<f64>() / x_m.len() as f64;
            let sy_p = y_p.iter().sum::<f64>() / y_p.len() as f64;
            let sy_m = y_m.iter().sum::<f64>() / y_m.len() as f64;
            let sz_p = z_p.iter().sum::<f64>() / z_p.len() as f64;
            let sz_m = z_m.iter().sum::<f64>() / z_m.len() as f64;

            nalgebra::Vector3::new(sx_p + sx_m, sy_p + sy_m, sz_p + sz_m)
        };
        let acc_scale = {
            let range_x: f64 = (x_p.iter().sum::<f64>() / x_p.len() as f64)
                - (x_m.iter().sum::<f64>() / x_m.len() as f64);
            let range_y: f64 = (y_p.iter().sum::<f64>() / y_p.len() as f64)
                - (x_m.iter().sum::<f64>() / x_m.len() as f64);
            let range_z: f64 = (z_p.iter().sum::<f64>() / z_p.len() as f64)
                - (x_m.iter().sum::<f64>() / x_m.len() as f64);

            let scale_x = 2.0 * G0 / range_x;
            let scale_y = 2.0 * G0 / range_y;
            let scale_z = 2.0 * G0 / range_z;

            let count = self.acc_points.len() as f64;

            nalgebra::Vector3::new(scale_x, scale_y, scale_z)
        };

        let cal_data = CalData {
            gyro_offset,
            acc_offset,
            acc_scale,
        };
        self.cal_data = Some(cal_data);

        cal_data
    }
}

pub enum CalStage {
    Gyro,
    AccTop,
    AccBottom,
    AccLeft,
    AccRight,
    AccFront,
    AccBack,
}
