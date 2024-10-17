const G0: f64 = 9.80665;
const G0_THR: f64 = G0 * 0.75;
const F0: f64 = 48.8819; // uT

use std::fs::File;
use std::io::prelude::*;
use std::path::PathBuf;

use nalgebra::{Dyn, Matrix3, Vector3, U10};

#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    pub lin_acc: Vector3<f64>,
    pub ang_vel: Vector3<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct MagData {
    pub field: Vector3<f64>,
}

#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct CalData {
    pub gyro_offset: Vector3<f64>,
    pub acc_offset: Vector3<f64>,
    pub acc_scale: Vector3<f64>,
    pub soft_iron_transf: nalgebra::Matrix3<f64>,
    pub hard_iron_bias: Vector3<f64>,
}

impl CalData {
    pub fn apply_mag_cal(&self, mag_point: &Vector3<f64>) -> Vector3<f64> {
        self.soft_iron_transf * (mag_point - self.hard_iron_bias)
    }
}

#[derive(Debug)]
pub struct Cal {
    gyro_points: Vec<Vector3<f64>>,
    acc_points: Vec<Vector3<f64>>,
    mag_points: Vec<Vector3<f64>>,

    acc_points_avg: Vector3<f64>,
    gyro_points_avg: Vector3<f64>,

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

    pub fn save_to_file(&self, path: PathBuf) {
        let mut data = std::collections::HashMap::new();
        data.insert("acc", self.acc_points.clone());
        data.insert("gyro", self.gyro_points.clone());
        data.insert("mag", self.mag_points.clone());

        let json_string = serde_json::to_string(&data).unwrap();

        let mut file = File::create(path).unwrap();
        file.write_all(json_string.as_bytes()).unwrap();
    }

    pub fn load_from_file(&mut self, path: PathBuf) {
        let mut file = File::open(path).unwrap();
        let mut json_string = String::new();
        file.read_to_string(&mut json_string).unwrap();

        let data: std::collections::HashMap<&str, Vec<Vector3<f64>>> =
            serde_json::de::from_str(&json_string).unwrap();

        self.acc_points.extend_from_slice(&data["acc"]);
        self.gyro_points.extend_from_slice(&data["gyro"]);
        self.mag_points.extend_from_slice(&data["mag"]);
    }

    pub fn add_acc_measurement_still(&mut self, data: Vector3<f64>) {
        let alpha = 0.95;
        self.acc_points_avg = self.acc_points_avg * alpha + data * (1.0 - alpha);

        if (self.acc_points_avg - data).norm() < 1e-2 {
            self.acc_points.push(data);
        }
    }

    pub fn add_gyro_measurement_still(&mut self, data: Vector3<f64>) {
        let alpha = 0.98;
        self.gyro_points_avg = self.gyro_points_avg * alpha + data * (1.0 - alpha);

        if (self.gyro_points_avg - data).norm() < 1e-3 {
            self.gyro_points.push(data);
        }
    }

    pub fn add_gyro_measurement(&mut self, data: Vector3<f64>) {
        self.gyro_points.push(data);
    }

    pub fn add_acc_measurement(&mut self, data: Vector3<f64>) {
        self.acc_points.push(data);
    }

    pub fn add_mag_measurement(&mut self, data: Vector3<f64>) {
        self.mag_points.push(data);
    }

    pub fn gyro_measurements(&self) -> &Vec<Vector3<f64>> {
        &self.gyro_points
    }

    pub fn acc_measurements(&self) -> &Vec<Vector3<f64>> {
        &self.acc_points
    }

    pub fn gyro_measurements_with_cal(&self) -> Vec<Vector3<f64>> {
        if let Some(cal_data) = self.cal_data {
            self.gyro_points
                .iter()
                .map(|p| *p - cal_data.gyro_offset)
                .collect::<Vec<_>>()
        } else {
            vec![]
        }
    }

    pub fn acc_measurements_with_cal(&self) -> Vec<Vector3<f64>> {
        if let Some(cal_data) = self.cal_data {
            self.acc_points
                .iter()
                .map(|p| (*p + cal_data.acc_offset).component_mul(&cal_data.acc_scale))
                .collect::<Vec<_>>()
        } else {
            vec![]
        }
    }

    pub fn mag_measurements_with_cal(&self) -> Vec<Vector3<f64>> {
        if let Some(cal_data) = self.cal_data {
            self.mag_points
                .iter()
                .map(|p| cal_data.apply_mag_cal(p))
                .collect::<Vec<_>>()
        } else {
            vec![]
        }
    }

    pub fn mag_measurements(&self) -> &Vec<Vector3<f64>> {
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

            Vector3::new(sum_x / count, sum_y / count, sum_z / count)
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

            Vector3::new(sx_p + sx_m, sy_p + sy_m, sz_p + sz_m)
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

            Vector3::new(scale_x, scale_y, scale_z)
        };

        // mag
        //
        // refs:
        // https://teslabs.com/articles/magnetometer-calibration/
        let (m, n, d) = Self::fit_mag_ellipsoid(&self.mag_points);
        let (a1, b) = Self::cac_mag_params_from_fit(F0, m, n, d);

        let cal_data = CalData {
            gyro_offset,
            acc_offset,
            acc_scale,
            soft_iron_transf: a1,
            hard_iron_bias: b,
        };
        self.cal_data = Some(cal_data);

        cal_data
    }

    pub fn cac_mag_params_from_fit(
        f: f64, // magnitude of the magnetic field, this can be 1.0 for navigation as the magnitude does not matter
        m: nalgebra::Matrix3<f64>,
        n: Vector3<f64>,
        d: f64,
    ) -> (Matrix3<f64>, Vector3<f64>) {
        let m_1 = m.try_inverse().unwrap();
        let b = -(m_1 * n);

        let m_msqrt = mat3_m_sqrt(m, 10);

        let x1 = (n.transpose() * (m_1 * n)).add_scalar(-d);
        let x1_sqrt = x1.map(|x| x.sqrt()); // element-wise

        let a_1 = m_msqrt * (f / x1_sqrt[0]);

        (a_1, b)
    }

    pub fn fit_mag_ellipsoid(
        mag_points: &[Vector3<f64>],
    ) -> (nalgebra::Matrix3<f64>, Vector3<f64>, f64) {
        // refs:
        // https://ieeexplore.ieee.org/abstract/document/1290055/
        // https://github.com/nliaudat/magnetometer_calibration/blob/main/calibrate.py
        // https://teslabs.com/articles/magnetometer-calibration/

        type Matrix10xNf64 = nalgebra::OMatrix<f64, U10, Dyn>;

        let d_cols: Vec<_> = mag_points
            .iter()
            .map(|p| {
                nalgebra::vector![
                    p.x * p.x,
                    p.y * p.y,
                    p.z * p.z,
                    2.0 * p.y * p.z,
                    2.0 * p.x * p.z,
                    2.0 * p.x * p.y,
                    2.0 * p.x,
                    2.0 * p.y,
                    2.0 * p.z,
                    1.0
                ]
            })
            .collect();

        let d_mat = Matrix10xNf64::from_columns(&d_cols);

        let s = d_mat.clone() * d_mat.transpose();
        let s_11 = s.fixed_view::<6, 6>(0, 0);
        let s_12: nalgebra::Matrix<
            f64,
            nalgebra::Const<6>,
            nalgebra::Const<4>,
            nalgebra::ViewStorage<
                '_,
                f64,
                nalgebra::Const<6>,
                nalgebra::Const<4>,
                nalgebra::Const<1>,
                nalgebra::Const<10>,
            >,
        > = s.fixed_view::<6, 4>(0, 6);
        let s_21 = s.fixed_view::<4, 6>(6, 0);
        let s_22 = s.fixed_view::<4, 4>(6, 6);

        let c = nalgebra::matrix![
            -1.0, 1.0, 1.0, 0.0, 0.0, 0.0; //
            1.0, -1.0, 1.0, 0.0, 0.0, 0.0; //
            1.0, 1.0, -1.0, 0.0, 0.0, 0.0; //
            0.0, 0.0, 0.0, -4.0, 0.0, 0.0; //
            0.0, 0.0, 0.0, 0.0, -4.0, 0.0; //
            0.0, 0.0, 0.0, 0.0, 0.0, -4.0
        ];

        let c_inv = c.try_inverse().unwrap();
        let s_22_inv = s_22.try_inverse().unwrap();

        let e = c_inv * (s_11 - s_12 * (s_22_inv * s_21));

        let e_eigen = nalgebra_lapack::Eigen::new(e, true, true).unwrap();

        let e_v = e_eigen.eigenvectors.unwrap();
        let e_w = e_eigen.eigenvalues_re;

        // println!("E {}", e_v);

        let mut v_1 = (e_v.column(e_w.argmax().0)).clone_owned();

        if v_1[0] < 0.0 {
            v_1.neg_mut();
        }

        let v_2 = (-s_22_inv * s_21) * v_1;

        let m = nalgebra::matrix![
            v_1[0], v_1[5], v_1[4];
            v_1[5], v_1[1], v_1[3];
            v_1[4], v_1[3], v_1[2];
        ];

        let n = nalgebra::vector![v_2[0], v_2[1], v_2[2]];

        let d = v_2[3];

        // println!("{}", E_v);
        // println!("{}", E_w);
        // println!("{}", v_2);

        (m, n, d)
    }
}

pub fn mat3_m_sqrt(a: nalgebra::Matrix3<f64>, iter_count: usize) -> nalgebra::Matrix3<f64> {
    // https://en.wikipedia.org/wiki/Square_root_of_a_matrix
    // Babylonian method
    let mut x = nalgebra::Matrix3::identity();

    for _ in 0..iter_count {
        x = 0.5 * (x + a * x.try_inverse().unwrap());
    }

    x
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn mat_msqrt() {
        let a = nalgebra::matrix![1.0,3.0; 1.0,4.0];

        let mut x = nalgebra::Matrix2::identity();

        for _ in 0..10 {
            x = 0.5 * (x + a * x.try_inverse().unwrap());
        }

        println!("{}", x);
        println!("{}", x * x); // = a
    }

    #[test]
    fn fit() {
        let mag_points: Vec<_> = MAG_TEST_DATA
            .iter()
            .map(|p| nalgebra::vector![p[0], p[1], p[2]])
            .collect();

        // calculate with
        // https://github.com/nliaudat/magnetometer_calibration/blob/main/calibrate.py
        //
        // M =
        // [[ 0.47604884  0.00960189 -0.04458678]
        //  [ 0.00960189  0.48901435  0.01152682]
        //  [-0.04458678  0.01152682  0.72940347]]
        // n =
        // [[   6.66405239]
        // [  36.98828441]
        // [-412.64328998]]
        // d =
        // 220982.468485425

        let (m, n, d) = Cal::fit_mag_ellipsoid(&mag_points);
        println!("M {}, n {}, d {}", m, n, d);

        // calculate with
        // https://github.com/nliaudat/magnetometer_calibration/blob/main/calibrate.py
        //
        // A_1 =
        // [[ 5.26589832  0.05402899 -0.22103196]
        // [ 0.05402899  5.34152182  0.05771691]
        // [-0.22103196  0.05771691  6.52032584]]
        // b =
        // [[41.16886643];
        // [-89.87465738];
        // [569.66392911]]

        let (a_1, b) = Cal::cac_mag_params_from_fit(1000.0, m, n, d);
        println!("A_1 {}, b {}", a_1, b);

        // calculate with
        // https://github.com/nliaudat/magnetometer_calibration/blob/main/calibrate.py
        //
        // [[ -32.66246512]
        // [1004.79174348]
        // [  22.6600199 ]]
        let mag_point = mag_points.first().unwrap();
        println!("first element with cal: {}", a_1 * (mag_point - b))
    }

    const MAG_TEST_DATA: [[f64; 3]; 243] = [
        [33.1, 98.3, 571.2],
        [33.1, 98.3, 571.2],
        [33.1, 98.3, 571.2],
        [33.1, 98.7, 571.2],
        [33.5, 98.3, 571.2],
        [-3.0, 94.0, 569.4],
        [-6.1, 93.5, 568.5],
        [-27.8, 86.6, 566.8],
        [-35.2, 84.0, 565.9],
        [-50.5, 77.0, 564.6],
        [-58.7, 72.2, 563.3],
        [-67.4, 67.0, 562.5],
        [-67.4, 66.6, 562.5],
        [-68.3, 66.1, 562.0],
        [-83.1, 55.2, 560.3],
        [-91.8, 47.9, 559.4],
        [-105.7, 32.6, 557.7],
        [-109.2, 28.3, 557.2],
        [-119.6, 13.5, 555.9],
        [-125.3, 4.8, 555.1],
        [-136.6, -20.4, 553.3],
        [-137.5, -22.6, 552.9],
        [-140.9, -33.5, 552.5],
        [-144.9, -47.9, 552.0],
        [-147.9, -74.0, 551.6],
        [-148.8, -81.3, 551.6],
        [-147.9, -101.8, 551.1],
        [-146.6, -115.3, 551.1],
        [-139.2, -146.6, 551.1],
        [-132.2, -165.3, 550.7],
        [-124.0, -181.4, 551.1],
        [-117.9, -191.0, 551.1],
        [-105.7, -207.1, 551.1],
        [-104.0, -209.2, 551.1],
        [-91.3, -221.9, 551.6],
        [-88.3, -224.9, 551.6],
        [-70.5, -238.4, 552.0],
        [-64.8, -242.3, 552.0],
        [-40.9, -255.8, 553.3],
        [-130.5, -169.7, 550.7],
        [-65.7, -241.9, 552.0],
        [-44.4, -254.0, 552.9],
        [-19.1, -264.0, 554.2],
        [-3.9, -268.4, 555.1],
        [16.5, -272.7, 555.9],
        [26.1, -273.6, 556.4],
        [48.7, -274.5, 557.2],
        [57.4, -274.5, 557.7],
        [86.1, -271.0, 558.5],
        [102.7, -267.1, 559.4],
        [111.8, -264.0, 559.8],
        [124.4, -259.3, 560.7],
        [143.6, -248.8, 562.5],
        [151.8, -244.0, 562.9],
        [172.7, -228.8, 564.2],
        [179.7, -221.9, 565.1],
        [201.8, -193.6, 566.8],
        [205.8, -187.5, 567.2],
        [219.2, -157.9, 566.8],
        [223.2, -144.9, 566.8],
        [227.9, -124.4, 566.4],
        [229.7, -106.1, 565.9],
        [230.1, -97.0, 565.9],
        [227.9, -62.6, 565.5],
        [221.9, -38.3, 566.8],
        [213.6, -15.7, 567.7],
        [206.2, -1.3, 568.1],
        [184.4, 30.0, 569.9],
        [168.3, 46.5, 570.7],
        [150.5, 60.9, 570.7],
        [139.6, 68.3, 570.7],
        [116.6, 80.0, 570.7],
        [102.7, 85.7, 569.9],
        [67.4, 94.4, 568.5],
        [57.0, 95.7, 568.1],
        [41.3, 96.6, 566.8],
        [32.6, 96.6, 566.4],
        [6.1, 94.0, 564.6],
        [-1.7, 92.2, 564.6],
        [-35.7, 81.8, 562.9],
        [-47.0, 76.6, 562.0],
        [-57.0, 71.3, 561.6],
        [-72.6, 60.9, 560.3],
        [-89.6, 47.4, 559.0],
        [-107.0, 28.3, 556.8],
        [-137.9, -29.6, 553.3],
        [-145.3, -60.0, 552.5],
        [-146.6, -98.7, 552.0],
        [-140.9, -134.9, 551.1],
        [-110.5, -198.8, 549.4],
        [-79.2, -231.4, 549.4],
        [-29.1, -260.1, 551.1],
        [-4.8, -268.0, 552.5],
        [67.0, -273.6, 559.0],
        [86.6, -270.6, 560.7],
        [137.5, -252.7, 563.8],
        [161.8, -237.1, 565.1],
        [198.4, -197.5, 565.9],
        [214.9, -166.6, 565.9],
        [227.1, -120.5, 566.8],
        [228.4, -102.7, 567.2],
        [221.9, -40.9, 568.5],
        [210.5, -11.7, 570.3],
        [182.7, 31.3, 572.9],
        [160.1, 53.5, 573.8],
        [119.6, 78.7, 572.9],
        [99.6, 86.6, 571.6],
        [44.8, 96.6, 567.2],
        [23.9, 96.1, 565.1],
        [-25.2, 86.1, 561.6],
        [-35.7, 82.2, 560.7],
        [-51.8, 74.4, 559.8],
        [-78.7, 56.5, 559.0],
        [-95.3, 41.8, 558.1],
        [-127.0, -2.6, 555.1],
        [-139.2, -32.2, 553.3],
        [-147.5, -80.9, 551.1],
        [-146.2, -110.5, 550.3],
        [-112.2, -197.9, 551.1],
        [-43.9, -254.5, 552.5],
        [82.2, -271.4, 559.0],
        [138.8, -251.9, 561.2],
        [229.2, -78.7, 568.5],
        [201.4, 6.1, 572.5],
        [-26.1, 85.3, 561.2],
        [-122.7, 5.2, 556.4],
        [-126.6, -174.9, 549.0],
        [-80.0, -230.6, 550.7],
        [105.3, -265.8, 559.0],
        [183.1, -216.6, 564.2],
        [229.2, -96.1, 568.5],
        [221.9, -40.0, 571.2],
        [153.1, 58.7, 569.9],
        [87.4, 90.9, 568.1],
        [-30.0, 84.4, 565.1],
        [-140.5, -35.2, 554.2],
        [40.9, -274.5, 557.2],
        [144.9, -248.8, 561.6],
        [221.9, -149.2, 567.7],
        [227.5, -63.1, 568.1],
        [127.5, 74.4, 568.1],
        [114.8, 80.5, 568.1],
        [97.9, 87.0, 568.1],
        [-10.0, 91.3, 565.9],
        [-117.0, 17.0, 555.1],
        [-147.5, -66.1, 551.1],
        [54.4, -274.5, 557.2],
        [203.1, -191.8, 566.4],
        [11.3, 96.1, 565.1],
        [-143.1, -43.9, 550.7],
        [174.0, -227.1, 564.2],
        [219.7, -30.9, 568.5],
        [-147.9, -94.4, 551.6],
        [-80.0, -232.7, 550.3],
        [224.0, -43.5, 568.5],
        [177.9, 37.4, 572.0],
        [229.7, -74.0, 567.2],
        [212.7, -173.1, 562.0],
        [172.3, -227.1, 561.2],
        [-45.2, -254.9, 551.1],
        [-114.4, -199.2, 550.3],
        [-120.1, 20.4, 556.8],
        [-80.5, 61.8, 560.3],
        [30.9, 100.1, 567.7],
        [97.9, 89.6, 572.9],
        [228.4, -65.7, 567.2],
        [226.2, -134.0, 565.5],
        [148.3, -244.9, 561.2],
        [6.1, -270.1, 553.8],
        [-113.1, -199.7, 551.6],
        [-150.9, -95.3, 552.5],
        [-48.3, 81.8, 560.7],
        [6.1, 97.9, 564.6],
        [-68.7, 58.7, 542.4],
        [-117.0, -179.7, 531.6],
        [216.2, -147.9, 545.1],
        [225.8, -97.4, 542.9],
        [221.4, -112.7, 542.0],
        [216.6, -45.2, 537.7],
        [209.7, -157.5, 538.1],
        [63.5, 89.6, 539.4],
        [-91.3, 35.2, 522.0],
        [-56.5, -233.6, 512.0],
        [-24.4, 79.6, 527.2],
        [212.3, -38.3, 528.1],
        [209.7, -31.8, 527.2],
        [220.1, -97.0, 525.9],
        [86.6, 83.5, 529.0],
        [-54.4, -234.0, 511.1],
        [44.4, -258.8, 508.1],
        [101.8, -249.3, 513.3],
        [-134.0, -35.2, 508.5],
        [219.2, -94.0, 521.6],
        [97.0, 80.9, 529.4],
        [127.0, -239.7, 513.7],
        [195.3, -178.4, 520.3],
        [-60.0, 65.7, 517.7],
        [-138.8, -61.3, 506.3],
        [218.8, -67.0, 525.9],
        [65.7, 88.7, 524.6],
        [184.4, 14.8, 527.2],
        [189.2, 7.8, 525.9],
        [121.4, 68.7, 526.4],
        [-42.6, 70.0, 515.5],
        [-106.1, 15.7, 511.6],
        [-136.6, -70.9, 505.0],
        [-127.9, -140.5, 503.3],
        [-97.9, -194.9, 503.3],
        [-19.1, -248.8, 506.3],
        [-13.1, -268.8, 559.0],
        [37.4, -277.1, 563.3],
        [168.8, -235.3, 568.1],
        [227.5, -134.9, 568.5],
        [230.1, -121.8, 568.5],
        [126.2, 77.4, 576.8],
        [32.2, 97.9, 571.2],
        [-42.6, 80.5, 566.4],
        [-71.8, 63.5, 565.1],
        [-148.8, -82.7, 555.5],
        [-141.4, -140.9, 554.2],
        [-63.9, -244.9, 558.1],
        [-9.1, -268.4, 560.3],
        [126.2, -258.8, 565.9],
        [172.3, -229.2, 568.5],
        [231.0, -96.1, 574.2],
        [210.1, -7.0, 576.8],
        [80.0, 93.1, 572.9],
        [-39.2, 80.9, 567.7],
        [-113.1, 22.2, 561.6],
        [-147.5, -64.4, 558.1],
        [-47.4, -253.6, 558.5],
        [53.9, -275.8, 560.3],
        [203.6, -193.6, 570.7],
        [230.6, -107.4, 571.6],
        [126.6, 75.7, 574.2],
        [81.3, 91.8, 574.6],
        [10.0, 95.7, 572.5],
        [10.0, 95.7, 572.5],
        [10.0, 95.7, 572.5],
        [10.0, 95.7, 572.5],
        [10.0, 95.3, 572.5],
        [10.0, 95.7, 572.5],
        [10.0, 95.7, 572.5],
    ];
}
