use crate::channels::{CONTROL_CH, ControlEvent};
use embassy_time::{Duration, Timer};

pub struct Kalman1D {
    x: f32,
    p: f32,
    q: f32,
    r: f32,
}

impl Kalman1D {
    pub fn new(q: f32, r: f32, init: f32) -> Self {
        Self {
            x: init,
            p: 1.0,
            q,
            r,
        }
    }

    pub fn update(&mut self, z: f32) -> f32 {
        self.p += self.q;
        let k = self.p / (self.p + self.r);
        self.x += k * (z - self.x);
        self.p *= 1.0 - k;
        self.x
    }
}
