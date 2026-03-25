pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    prev_error: f32,
    out_min: f32,
    out_max: f32,
}

impl Pid {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
            out_min: 0.0,
            out_max: 1.0,
        }
    }

    pub fn update(&mut self, setpoint: f32, measured: f32, dt: f32) -> f32 {
        let error = setpoint - measured;

        self.integral += error * dt;

        let derivative = (error - self.prev_error) / dt;

        self.prev_error = error;

        let mut out = self.kp * error + self.ki * self.integral + self.kd * derivative;

        out = out.clamp(self.out_min, self.out_max);

        out
    }
}
