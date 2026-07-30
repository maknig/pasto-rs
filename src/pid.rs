pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    prev_error: f32,
    out_min: f32,
    out_max: f32,
    damping_factor: f32,
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
            damping_factor: 0.01,
        }
    }
    pub fn integral(&self) -> f32 {
        self.integral
    }

    pub fn update(&mut self, setpoint: f32, measured: f32, dt: f32) -> f32 {
        let error = setpoint - measured;

        let p_value = self.kp * error;
        //let p_value = self.kp * (error / (1.0 + self.damping_factor * error.abs()));

        if dt <= 0.0 {
            return p_value.clamp(self.out_min, self.out_max);
        }

        let d_value = self.kd * (error - self.prev_error) / dt;

        // Integrate in OUTPUT units and clamp to the actuator range (anti-windup)
        // so the integral alone can supply the steady hold duty (~0.037 at 94 C)
        // and zero the error. The old form clamped the raw error-integral to
        // +/-1, which with ki=0.005 capped the integral's authority at +/-0.005
        // duty -- far below the hold duty, leaving a ~1 C proportional offset at
        // setpoint.
        self.integral += self.ki * error * dt;
        self.integral = self.integral.clamp(self.out_min, self.out_max);

        let i_value = self.integral;

        self.prev_error = error;

        let mut out = p_value + i_value + d_value;

        out = out.clamp(self.out_min, self.out_max);

        out
    }
}
