#![no_std]

//! Generic Smith predictor library. No model, gains, or timing constants
//! live here -- all of that (model_a/b/c/d, delay_steps, T_AMB, PID gains,
//! buffer capacity via the const generic N) is owned entirely by the
//! calling module (control_task.rs), which is where hardware-specific
//! calibration belongs.

/// A simple discrete 2-state space model:
/// x[k+1] = A * x[k] + B * u[k]
/// y[k]   = C * x[k] + D * u[k]
#[derive(Debug, Clone, Copy)]
pub struct DiscreteStateSpaceModel {
    pub a: [[f64; 2]; 2],
    pub b: [f64; 2],
    pub c: [f64; 2],
    pub d: f64,
    pub x: [f64; 2],
}

impl DiscreteStateSpaceModel {
    pub const fn new(a: [[f64; 2]; 2], b: [f64; 2], c: [f64; 2], d: f64) -> Self {
        Self {
            a,
            b,
            c,
            d,
            x: [0.0, 0.0],
        }
    }

    /// Read the current delay-free output y[k] without mutating state
    pub fn output(&self, u: f64) -> f64 {
        self.c[0] * self.x[0] + self.c[1] * self.x[1] + self.d * u
    }

    /// Update internal state x[k+1] using control input u[k] and return y[k]
    pub fn update(&mut self, u: f64) -> f64 {
        let y = self.output(u);
        let x0_next = self.a[0][0] * self.x[0] + self.a[0][1] * self.x[1] + self.b[0] * u;
        let x1_next = self.a[1][0] * self.x[0] + self.a[1][1] * self.x[1] + self.b[1] * u;
        self.x = [x0_next, x1_next];
        y
    }

    pub fn reset(&mut self) {
        self.x = [0.0, 0.0];
    }
}

/// Interface trait for the inner PID controller
pub trait PidController {
    fn compute(&mut self, setpoint: f64, process_variable: f64) -> f64;
    fn reset(&mut self);
}

/// Generic PI controller (Td=0) with clamped output and conditional
/// anti-windup, implementing PidController. Gains/limits are supplied by
/// the caller at construction -- nothing model-specific lives in this
/// struct itself.
#[derive(Debug, Clone, Copy)]
pub struct PiController {
    kc: f64,
    ti: f64,
    ts: f64,
    u_min: f64,
    u_max: f64,
    integral: f64,
}

impl PiController {
    pub const fn new(kc: f64, ti: f64, ts: f64, u_min: f64, u_max: f64) -> Self {
        Self {
            kc,
            ti,
            ts,
            u_min,
            u_max,
            integral: 0.0,
        }
    }
}

impl PidController for PiController {
    fn compute(&mut self, setpoint: f64, process_variable: f64) -> f64 {
        let error = setpoint - process_variable;
        let u_unsat = self.kc * (error + self.integral / self.ti);
        let u = if u_unsat > self.u_max {
            self.u_max
        } else if u_unsat < self.u_min {
            self.u_min
        } else {
            u_unsat
        };

        let would_relieve = (u_unsat - u).signum() != error.signum();
        if u == u_unsat || would_relieve {
            self.integral += error * self.ts;
        }

        u
    }

    fn reset(&mut self) {
        self.integral = 0.0;
    }
}

/// Fixed-size zero-allocation ring buffer for dead time delay.
/// N is the compile-time max capacity, chosen by the caller (control.rs)
/// to fit their own delay_steps with headroom -- not fixed here.
#[derive(Debug, Clone, Copy)]
pub struct RingBuffer<const N: usize> {
    data: [f64; N],
    head: usize,
    capacity: usize,
}

impl<const N: usize> RingBuffer<N> {
    pub fn new(capacity: usize) -> Self {
        let cap = if capacity > N { N } else { capacity };
        Self {
            data: [0.0; N],
            head: 0,
            capacity: cap,
        }
    }

    /// Get oldest value in queue (y_m = y_hat[k - d])
    pub fn read_oldest(&self) -> f64 {
        if self.capacity == 0 {
            0.0
        } else {
            self.data[self.head]
        }
    }

    /// Overwrite oldest entry with new sample and advance head
    pub fn push(&mut self, val: f64) {
        if self.capacity > 0 {
            self.data[self.head] = val;
            self.head = (self.head + 1) % self.capacity;
        }
    }

    pub fn reset(&mut self) {
        self.data = [0.0; N];
        self.head = 0;
    }
}

/// Discrete Smith Predictor wrapping a primary PID controller, an
/// undelayed state-space model, and a heapless delay ring buffer.
///
/// N = compile-time max delay buffer capacity (const generic, chosen by
/// the caller). delay_steps (runtime, <= N) = actual dead-time steps for
/// your identified model.
pub struct DiscreteSmithPredictor<C: PidController, const N: usize> {
    pub controller: C,
    pub model: DiscreteStateSpaceModel,
    pub delay_steps: usize,
    buffer: RingBuffer<N>,
}

impl<C: PidController, const N: usize> DiscreteSmithPredictor<C, N> {
    pub fn new(
        controller: C,
        model_a: [[f64; 2]; 2],
        model_b: [f64; 2],
        model_c: [f64; 2],
        model_d: f64,
        delay_steps: usize,
    ) -> Self {
        Self {
            controller,
            model: DiscreteStateSpaceModel::new(model_a, model_b, model_c, model_d),
            delay_steps,
            buffer: RingBuffer::<N>::new(delay_steps),
        }
    }

    /// Primary execution step:
    /// 1. Reads delayed model prediction y_m[k - d] from ring buffer.
    /// 2. Computes error disturbance offset: d_hat = y_meas - y_m.
    /// 3. Computes control signal u[k] using delay-free estimate.
    /// 4. Advances internal undelayed model state and pushes output to delay queue.
    ///
    /// setpoint and y_measured must both be in the SAME units the model
    /// was identified in (e.g. degC deviation from T_AMB, not raw degC --
    /// do that conversion in control.rs, not here).
    pub fn step(&mut self, setpoint: f64, y_measured: f64) -> (f64, f64) {
        self.step_with_feedforward(setpoint, y_measured, 0.0)
    }

    /// Same as `step`, but adds `u_ff` (e.g. a known disturbance-rejection
    /// feedforward) to the control output BEFORE advancing the internal
    /// model. Any additive term applied to the real plant outside of the
    /// PID's own output must go through here, not be added post-hoc to
    /// step()'s return value -- otherwise the internal model silently
    /// diverges from what the real actuator actually received.
    ///
    /// Note: u_total is NOT re-clamped to the controller's own u_min/u_max
    /// after adding u_ff -- if you need a hard actuator ceiling inclusive
    /// of feedforward, clamp the returned value in control.rs.
    pub fn step_with_feedforward(
        &mut self,
        setpoint: f64,
        y_measured: f64,
        u_ff: f64,
    ) -> (f64, f64) {
        let y_m = self.buffer.read_oldest();
        let d_hat = y_measured - y_m;
        let y_hat = self.model.output(0.0);
        let y_feedback = y_hat + d_hat;
        let u_pid = self.controller.compute(setpoint, y_feedback);
        let u_total = u_pid + u_ff;
        let y_hat_next = self.model.update(u_total);
        self.buffer.push(y_hat_next);
        (u_total, y_hat)
    }

    pub fn reset(&mut self) {
        self.controller.reset();
        self.model.reset();
        self.buffer.reset();
    }
}
