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
        self.update_with_disturbance(u, [0.0, 0.0])
    }

    /// Like `update`, but adds a known additive state disturbance `d[k]` to
    /// x[k+1] -- a modeled load acting directly on a node (not through the
    /// input `u`/`b`), e.g. a heat sink at the sensor node discretized into
    /// its own [dx0, dx1] vector by the caller. Keeps the internal model in
    /// step with a real, quantifiable disturbance so y_hat doesn't diverge.
    pub fn update_with_disturbance(&mut self, u: f64, d: [f64; 2]) -> f64 {
        let y = self.output(u);
        let x0_next = self.a[0][0] * self.x[0] + self.a[0][1] * self.x[1] + self.b[0] * u + d[0];
        let x1_next = self.a[1][0] * self.x[0] + self.a[1][1] * self.x[1] + self.b[1] * u + d[1];
        self.x = [x0_next, x1_next];
        y
    }

    pub fn reset(&mut self) {
        self.x = [0.0, 0.0];
    }

    /// Roll a copy of the state forward `horizon` steps with zero input and
    /// return the peak y reached -- a "what if I stopped applying power
    /// right now" projection. Currently unused: the preheat cutoff moved to a
    /// model-independent measured-lead test (PREHEAT_LEAD_C), which is robust to
    /// model error. Kept as the hook for a future model-predictive cutoff.
    #[allow(dead_code)]
    pub fn peak_free_response(&self, horizon: usize) -> f64 {
        let mut x = self.x;
        let mut peak = self.c[0] * x[0] + self.c[1] * x[1];
        for _ in 0..horizon {
            let x0_next = self.a[0][0] * x[0] + self.a[0][1] * x[1];
            let x1_next = self.a[1][0] * x[0] + self.a[1][1] * x[1];
            x = [x0_next, x1_next];
            let y = self.c[0] * x[0] + self.c[1] * x[1];
            if y > peak {
                peak = y;
            }
        }
        peak
    }
}

/// Interface trait for the inner PID controller
pub trait PidController {
    fn compute(&mut self, setpoint: f64, process_variable: f64) -> f64;
    fn reset(&mut self);
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

    /// Overwrite every in-use slot (0..capacity) with the same value and
    /// reset the head -- used to re-seed the delay line from a real
    /// measurement instead of zeroing it (see
    /// `DiscreteSmithPredictor::reseed`).
    pub fn fill(&mut self, val: f64) {
        for i in 0..self.capacity {
            self.data[i] = val;
        }
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
    // Runtime dead-time steps; the buffer is sized from it at construction.
    // No longer read after the preheat cutoff dropped its horizon rollout.
    #[allow(dead_code)]
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
        self.step_with_feedforward(setpoint, y_measured, 0.0, [0.0, 0.0])
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
    ///
    /// `state_dist` is a known additive state disturbance (see
    /// `update_with_disturbance`) applied to the internal model as it
    /// advances -- e.g. the pump's water heat-draw at the sensor node. Pass
    /// [0.0, 0.0] when there is none.
    ///
    /// Returns `(u_total, y_feedback)`. The second element is the
    /// dead-time-compensated estimate `model.output + d_hat`, NOT the raw
    /// open-loop model output -- it tracks the real temperature at steady
    /// state for any model DC gain (the gain cancels via d_hat) and leads
    /// temp by the dead time during transients, so it is the meaningful thing
    /// to display. Reporting it here keeps the model gain from ever needing to
    /// be distorted just to make the plot sit on setpoint.
    pub fn step_with_feedforward(
        &mut self,
        setpoint: f64,
        y_measured: f64,
        u_ff: f64,
        state_dist: [f64; 2],
    ) -> (f64, f64) {
        let y_m = self.buffer.read_oldest();
        let d_hat = y_measured - y_m;
        let y_hat = self.model.output(0.0);
        let y_feedback = y_hat + d_hat;
        let u_pid = self.controller.compute(setpoint, y_feedback);
        let u_total = u_pid + u_ff;
        let y_hat_next = self.model.update_with_disturbance(u_total, state_dist);
        self.buffer.push(y_hat_next);
        (u_total, y_feedback)
    }

    /// Advance the internal model/delay buffer with a known applied input,
    /// without invoking the inner controller -- for open-loop phases (e.g.
    /// preheat/coast) where the PID output is bypassed but the model and
    /// disturbance estimate must stay warm against real sensor data, so
    /// there's no discontinuity when closed-loop control resumes. Returns
    /// the current disturbance estimate d_hat = y_measured - y_m[k - d].
    pub fn advance_open_loop(
        &mut self,
        y_measured: f64,
        u_applied: f64,
        state_dist: [f64; 2],
    ) -> f64 {
        let y_m = self.buffer.read_oldest();
        let d_hat = y_measured - y_m;
        let y_hat_next = self.model.update_with_disturbance(u_applied, state_dist);
        self.buffer.push(y_hat_next);
        d_hat
    }

    /// Re-seed model state and delay buffer from a real measurement instead
    /// of zeroing them. Use this instead of `reset()` whenever the plant is
    /// not actually at the model's zero/ambient reference (e.g.
    /// re-enabling while still hot) -- otherwise the delay buffer reads a
    /// bogus 0.0 against a real hot measurement for the next `delay_steps`
    /// ticks, producing a large spurious d_hat spike right when precision
    /// matters most.
    pub fn reseed(&mut self, y_measured: f64) {
        self.controller.reset();
        self.model.x = [y_measured, y_measured];
        self.buffer.fill(y_measured);
    }

    pub fn reset(&mut self) {
        self.controller.reset();
        self.model.reset();
        self.buffer.reset();
    }
}
