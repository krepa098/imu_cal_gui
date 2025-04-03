// borrowed from:
// https://github.com/PaulStoffregen/MotionCal/blob/master/quality.c
// discussion here:
// https://forum.pjrc.com/threads/59277-Motion-Sensor-Calibration-Tool-Parameter-Understanding

use nalgebra::Vector3;
use serde::ser::SerializeStruct;
use serde::Serialize;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

const REGION_COUNT: usize = 100;

fn sphere_region(p: Vector3<f64>) -> i32 {
    let longitude = p.y.atan2(p.x) + PI;
    let latitude = FRAC_PI_2 - ((p.x * p.x + p.y * p.y).sqrt()).atan2(p.z);
    let mut region;

    if latitude > 1.37046 {
        // arctic cap, 1 region
        region = 0;
    } else if latitude < -1.37046 {
        // antarctic cap, 1 region
        region = 99;
    } else if latitude > 0.74776 || latitude < -0.74776 {
        // temperate zones, 15 regions each
        region = (longitude * (15.0 / TAU)).floor() as i32;
        if region < 0 {
            region = 0;
        } else if region > 14 {
            region = 14;
        }
        if latitude > 0.0 {
            region += 1; // 1 to 15
        } else {
            region += 84; // 84 to 98
        }
    } else {
        // tropic zones, 34 regions each
        region = (longitude * (34.0 / TAU)).floor() as i32;
        if region < 0 {
            region = 0;
        } else if region > 33 {
            region = 33;
        }
        if latitude >= 0.0 {
            region += 16; // 16 to 49
        } else {
            region += 50; // 50 to 83
        }
    }

    region
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct Quality {
    #[serde(skip_serializing)]
    sphere_dist: Vec<i32>,
    #[serde(skip_serializing)]
    sphere_data: Vec<Vector3<f64>>,
    #[serde(skip_serializing)]
    sphere_ideal: Vec<Vector3<f64>>,
    #[serde(skip_serializing)]
    sphereideal_initialized: bool,
    #[serde(skip_serializing)]
    magnitude: Vec<f64>,
    quality_gaps_buffer: f64,
    quality_variance_buffer: f64,
    quality_wobble_buffer: f64,

    #[serde(skip_serializing)]
    quality_gaps_computed: bool,
    #[serde(skip_serializing)]
    quality_variance_computed: bool,
    #[serde(skip_serializing)]
    quality_wobble_computed: bool,
}

impl Default for Quality {
    fn default() -> Self {
        Self {
            sphere_dist: vec![0; REGION_COUNT],
            sphere_data: vec![Vector3::zeros(); REGION_COUNT],
            sphere_ideal: vec![Vector3::zeros(); REGION_COUNT],
            magnitude: vec![],
            sphereideal_initialized: false,
            quality_gaps_computed: false,
            quality_variance_computed: false,
            quality_wobble_computed: false,
            quality_gaps_buffer: 0.0,
            quality_variance_buffer: 0.0,
            quality_wobble_buffer: 0.0,
        }
    }
}

// impl serde::ser::Serialize for Quality {
//     fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
//     where
//         S: serde::Serializer,
//     {
//         let mut s = serializer.serialize_struct("Quality", 3)?;
//         s.serialize_field("gap_err", "")
//         s.end()
//     }
// }

impl Quality {
    pub fn reset(&mut self) {
        self.magnitude.clear();
        self.sphere_dist.fill_with(|| 0);
        self.sphere_data.fill_with(|| Vector3::zeros());

        if !self.sphereideal_initialized {
            self.sphere_ideal[0].x = 0.0;
            self.sphere_ideal[0].y = 0.0;
            self.sphere_ideal[0].z = 1.0;

            for i in 1..=15 {
                let longitude = ((i - 1) as f64 + 0.5) * (TAU / 15.0);
                self.sphere_ideal[i].x = longitude.cos() * 1.05911_f64.cos() * -1.0;
                self.sphere_ideal[i].y = longitude.sin() * 1.05911_f64.cos() * -1.0;
                self.sphere_ideal[i].z = 1.05911_f64.sin();
            }
            for i in 16..=49 {
                let longitude = ((i - 16) as f64 + 0.5) * (TAU / 34.0);
                self.sphere_ideal[i].x = longitude.cos() * 0.37388_f64.cos() * -1.0;
                self.sphere_ideal[i].y = longitude.sin() * 0.37388_f64.cos() * -1.0;
                self.sphere_ideal[i].z = 0.37388_f64.sin();
            }
            for i in 50..=83 {
                let longitude = ((i - 50) as f64 + 0.5) * (TAU / 34.0);
                self.sphere_ideal[i].x = longitude.cos() * 0.37388_f64.cos() * -1.0;
                self.sphere_ideal[i].y = longitude.sin() * 0.37388_f64.cos() * -1.0;
                self.sphere_ideal[i].z = -0.37388_f64.sin();
            }
            for i in 84..=98 {
                let longitude = ((i - 1) as f64 + 0.5) * (TAU / 15.0);
                self.sphere_ideal[i].x = longitude.cos() * 1.05911_f64.cos() * -1.0;
                self.sphere_ideal[i].y = longitude.sin() * 1.05911_f64.cos() * -1.0;
                self.sphere_ideal[i].z = -1.05911_f64.sin();
            }

            self.sphere_ideal[99].x = 0.0;
            self.sphere_ideal[99].y = 0.0;
            self.sphere_ideal[99].z = -1.0;
            self.sphereideal_initialized = true;
        }

        self.quality_gaps_computed = false;
        self.quality_variance_computed = false;
        self.quality_wobble_computed = false;
    }

    pub fn update(&mut self, p: Vector3<f64>) {
        self.magnitude
            .push((p.x * p.x + p.y * p.y + p.z * p.z).sqrt());
        let region = sphere_region(p) as usize;
        self.sphere_dist[region] += 1;
        self.sphere_data[region].x += p.x;
        self.sphere_data[region].y += p.y;
        self.sphere_data[region].z += p.z;
        self.quality_gaps_computed = false;
        self.quality_variance_computed = false;
        self.quality_wobble_computed = false;

        self.calc_magnitude_variance_error();
        self.calc_surface_gap_error();
        self.calc_wobble_error();
    }

    pub fn calc_surface_gap_error(&mut self) -> f64 {
        let mut error = 0.0;

        if self.quality_gaps_computed {
            return self.quality_gaps_buffer;
        }

        for i in 0..REGION_COUNT {
            let num = self.sphere_dist[i];
            if num == 0 {
                error += 1.0;
            } else if num == 1 {
                error += 0.2;
            } else if num == 2 {
                error += 0.01;
            }
        }
        self.quality_gaps_buffer = error;
        self.quality_gaps_computed = true;
        return self.quality_gaps_buffer;
    }

    fn calc_magnitude_variance_error(&mut self) -> f64 {
        if self.quality_variance_computed {
            return self.quality_variance_buffer;
        }

        let mut sum = 0.0;
        for i in 0..self.magnitude.len() {
            sum += self.magnitude[i];
        }
        let mean = sum / self.magnitude.len() as f64;
        let mut variance = 0.0;
        for i in 0..self.magnitude.len() {
            let diff = self.magnitude[i] - mean;
            variance += diff * diff;
        }
        variance /= self.magnitude.len() as f64;
        self.quality_variance_buffer = variance.sqrt() / mean * 100.0;
        self.quality_variance_computed = true;
        return self.quality_variance_buffer;
    }

    pub fn calc_wobble_error(&mut self) -> f64 {
        let mut xoff = 0.0;
        let mut yoff = 0.0;
        let mut zoff = 0.0;
        let mut n = 0;

        if self.quality_wobble_computed {
            return self.quality_wobble_buffer;
        }
        let mut sum = 0.0;
        for i in 0..self.magnitude.len() {
            sum += self.magnitude[i];
        }
        let radius = sum / self.magnitude.len() as f64;

        for i in 0..REGION_COUNT {
            if self.sphere_dist[i] > 0 {
                let x = self.sphere_data[i].x / self.sphere_dist[i] as f64;
                let y = self.sphere_data[i].y / self.sphere_dist[i] as f64;
                let z = self.sphere_data[i].z / self.sphere_dist[i] as f64;

                let xi = self.sphere_ideal[i].x * radius;
                let yi = self.sphere_ideal[i].y * radius;
                let zi = self.sphere_ideal[i].z * radius;

                xoff += x - xi;
                yoff += y - yi;
                zoff += z - zi;

                n += 1;
            }
        }
        if n == 0 {
            return 100.0;
        }

        xoff /= n as f64;
        yoff /= n as f64;
        zoff /= n as f64;

        self.quality_wobble_buffer =
            (xoff * xoff + yoff * yoff + zoff * zoff).sqrt() / radius * 100.0;
        self.quality_wobble_computed = true;
        return self.quality_wobble_buffer;
    }

    pub fn magnitude_variance_error(&self) -> f64 {
        self.quality_variance_buffer
    }

    pub fn wobble_error(&self) -> f64 {
        self.quality_wobble_buffer
    }

    pub fn surface_gap_error(&self) -> f64 {
        self.quality_gaps_buffer
    }
}
