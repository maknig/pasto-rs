use crate::channels::*;
use crate::config::{
    COAST_EXIT_MARGIN_C, COAST_MAX_S, FF_BREW_DECAY, FF_BREW_MAX, FF_BREW_STEADY, HEATER_TIMEOUT_S,
    PREHEAT_LEAD_C, PREHEAT_MAX_S, PREHEAT_MIN_ERROR_C, PREHEAT_POWER, SETPOINT,
};
use crate::heater::determine_heater_state;
use crate::pid::Pid;
use core::sync::atomic::Ordering;
use defmt::{info, warn};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Ticker};

use crate::smith::{DiscreteSmithPredictor, PidController};

impl PidController for Pid {
    fn compute(&mut self, setpoint: f64, process_variable: f64) -> f64 {
        // Ts is now genuinely fixed by the Ticker driving this loop -- see
        // CONTROL_TS_S below. Do not hardcode a literal here separately
        // from that constant, or the two can drift out of sync again.
        self.update(
            setpoint as f32,
            process_variable as f32,
            CONTROL_TS_S as f32,
        ) as f64
    }

    fn reset(&mut self) {
        //self.reset();
    }
}

// Fixed control-loop period. MUST match whatever Ts the model_a/model_b/
// delay_steps below were discretized at -- if you change this, regenerate
// those constants too, don't just edit the number here.
const CONTROL_TS_S: f64 = 0.5;
const T_AMB: f64 = 24.0; // calibrate to your machine's actual ambient at boot

// MAX_DELAY_BUF: compile-time buffer capacity, chosen here to give
// headroom above delay_steps (28) in case theta is re-identified larger
// later without a smith.rs edit. Bump if delay_steps ever exceeds this.
// Module-level (not function-local) so PreheatController's field type
// below can name the same concrete DiscreteSmithPredictor type.
const MAX_DELAY_BUF: usize = 32;

// -------------------------------------------------------------------------
// Gain-scheduled plant model (see the plan / CLAUDE.md sysid workflow).
//
// The machine is NONLINEAR: the block->ambient heat loss differs between
// warming up and holding at 94 C, so a single LTI gain can't serve both. The
// two models here are fit from DIFFERENT regimes and do NOT share coupling --
// regenerate each from its own data, and never hand-edit one constant in
// isolation (that display-driven gain torture is what previously stalled
// heat-up). Both use the physical dead time held at 13.8 s (do NOT let the
// auto-fit collapse it to ~1 s -- that reintroduces the limit cycle).
//
// TRANSIENT: used open-loop in Preheat/Coast. Fit from run_full_power.csv step 1
// (a real cold full-power step from ~25 C plus its coast, dead time held 13.8 s;
// `sysid_fit.py --start 120 --end 455 --dead-time 13.8`, NRMS 3.68%). Matches
// the real warmup step dynamics so the internal model doesn't balloon under
// full-power open loop and the coast-peak prediction is accurate.
// DC gain 594, poles 5.3 s / 1011 s.
const MODEL_TRANSIENT_A: [[f64; 2]; 2] =
    [[0.9934734071, 0.0065040775], [0.0771231289, 0.9163502782]];
const MODEL_TRANSIENT_B: [f64; 2] = [0.3405168541, 0.0133819029];
// SETPOINT: used in Closed (regulation + brewing). block->ambient loss taken
// from the MEASURED ~0.037 hold duty at 94 C, so the model matches the real
// near-setpoint plant -- that's what the PID and brew disturbance rejection are
// tuned against. DC gain 1890, poles 14.5 s / 1844 s.
const MODEL_SETPOINT_A: [[f64; 2]; 2] =
    [[0.9800818046, 0.0199135636], [0.0138567576, 0.9856840714]];
const MODEL_SETPOINT_B: [f64; 2] = [1.2436250543, 0.0087540019];
// Shared output/feedthrough/dead-time for both models.
const MODEL_C: [f64; 2] = [0.0, 1.0];
const MODEL_D: f64 = 0.0;
// Dead time = the gray-box `theta` for this machine (heater -> sensor transport
// lag). Kept explicit at 13.8 s; the auto-fit collapses it to ~1 s, which leaves
// ~13 s of lag uncompensated and produces a limit cycle around setpoint.
const DEAD_TIME_S: f64 = 13.8224;

enum ControlPhase {
    /// Fixed open-loop power, until the model predicts coasting from here
    /// would already reach setpoint.
    Preheat,
    /// Power off (plus pump feedforward), waiting for the thermal lag to
    /// carry temp near setpoint before closed-loop trim engages.
    Coast,
    /// Normal Smith predictor + PID closed-loop control.
    Closed,
}

/// Owns the Smith predictor plus the preheat/coast phase state. Lives
/// entirely inside control_task -- talks to heater_task exactly like plain
/// closed-loop control already does, via HeaterCommand::Power over
/// HEATER_CMD_CH (the caller sends whatever tick() returns, same as before).
struct PreheatController {
    phase: ControlPhase,
    smith: DiscreteSmithPredictor<Pid, MAX_DELAY_BUF>,
    phase_start: Instant,
}

impl PreheatController {
    fn new(smith: DiscreteSmithPredictor<Pid, MAX_DELAY_BUF>) -> Self {
        Self {
            phase: ControlPhase::Closed,
            smith,
            phase_start: Instant::now(),
        }
    }

    /// Load the transient (heat-up) plant model into the Smith predictor.
    /// Only the A/B coefficients change; C/D, dead time and internal state are
    /// untouched. Physically valid open-loop during warmup (see the model
    /// consts above).
    fn use_transient_model(&mut self) {
        self.smith.model.a = MODEL_TRANSIENT_A;
        self.smith.model.b = MODEL_TRANSIENT_B;
    }

    /// Load the near-setpoint plant model (matches the measured 94 C hold).
    fn use_setpoint_model(&mut self) {
        self.smith.model.a = MODEL_SETPOINT_A;
        self.smith.model.b = MODEL_SETPOINT_B;
    }

    /// Called on enable-toggle. Starts an open-loop preheat if the error is
    /// large enough to be worth it; otherwise reseeds and goes straight to
    /// closed-loop control, same as today's small-error behavior. This is the
    /// ONLY place the model state is set back to the real temperature -- disable
    /// and the Coast->Closed handoff no longer reseed.
    fn start_preheat(&mut self, now: Instant, measured_dev: f64, setpoint_dev: f64) {
        if setpoint_dev - measured_dev > PREHEAT_MIN_ERROR_C as f64 {
            // Heat-up: transient model so the internal state doesn't balloon
            // under full-power open loop and the coast-peak prediction is right.
            self.use_transient_model();
            self.smith.reseed(measured_dev);
            self.phase = ControlPhase::Preheat;
            self.phase_start = now;
        } else {
            // Small error: go straight to closed-loop regulation.
            self.use_setpoint_model();
            self.smith.reseed(measured_dev);
            self.phase = ControlPhase::Closed;
        }
    }

    /// Called on disable-toggle. A disable mid-preheat/coast must not silently
    /// resume afterward -- require a fresh enable-toggle. Does NOT reseed: while
    /// disabled `tick` isn't called, so the model just freezes at its last state
    /// until the next enable reseeds it from the real temperature.
    fn on_disable(&mut self) {
        self.phase = ControlPhase::Closed;
    }

    /// Called on fault-trip. Same reasoning as on_disable (re-enable reseeds).
    fn on_fault(&mut self) {
        self.phase = ControlPhase::Closed;
    }

    fn is_preheating(&self) -> bool {
        matches!(self.phase, ControlPhase::Preheat | ControlPhase::Coast)
    }

    /// Primary per-tick step. Returns (power, y_feedback): `power` is the 0..1
    /// heater fraction (same scale in every phase) and `y_feedback` is the
    /// dead-time-compensated temperature estimate for display (see
    /// `DiscreteSmithPredictor::step_with_feedforward`).
    fn tick(
        &mut self,
        now: Instant,
        measured_dev: f64,
        setpoint_dev: f64,
        pump_ff: f64,
        state_dist: [f64; 2],
    ) -> (f64, f64) {
        match self.phase {
            ControlPhase::Preheat => {
                let power_w = PREHEAT_POWER as f64;
                // Advance the model open-loop only to keep the display estimate
                // and d_hat / delay buffer warm -- it no longer drives the cutoff.
                let d_hat = self
                    .smith
                    .advance_open_loop(measured_dev, power_w, state_dist);
                // Dead-time-compensated estimate for display (see Closed branch).
                let y_feedback = self.smith.model.output(0.0) + d_hat;
                let elapsed_s = now.duration_since(self.phase_start).as_millis() as f64 / 1000.0;
                // Model-INDEPENDENT cutoff: cut power once measured temp comes
                // within PREHEAT_LEAD_C of setpoint, then let the thermal lag
                // coast the rest of the way. Replaces the peak_free_response
                // prediction, which fired late (the ~14 s dead time) and swung
                // the peak wildly with every model change.
                if measured_dev >= setpoint_dev - PREHEAT_LEAD_C as f64 {
                    self.phase = ControlPhase::Coast;
                    self.phase_start = now;
                } else if elapsed_s >= PREHEAT_MAX_S as f64 {
                    warn!("preheat safety-cap timeout, forcing coast");
                    self.phase = ControlPhase::Coast;
                    self.phase_start = now;
                }
                (power_w, y_feedback)
            }
            ControlPhase::Coast => {
                let power_w = pump_ff;
                let d_hat = self
                    .smith
                    .advance_open_loop(measured_dev, power_w, state_dist);
                let y_feedback = self.smith.model.output(0.0) + d_hat;
                let elapsed_s = now.duration_since(self.phase_start).as_millis() as f64 / 1000.0;
                let to_closed = measured_dev >= setpoint_dev - COAST_EXIT_MARGIN_C as f64
                    || elapsed_s >= COAST_MAX_S as f64;
                if to_closed {
                    if elapsed_s >= COAST_MAX_S as f64
                        && measured_dev < setpoint_dev - COAST_EXIT_MARGIN_C as f64
                    {
                        warn!("coast safety-cap timeout, forcing closed-loop");
                    }
                    // Hand off to closed-loop regulation: switch to the
                    // near-setpoint DYNAMICS but carry the warm state + delay
                    // buffer over unchanged (NO reseed). The open-loop phases ran
                    // the transient model, which tracks reality and doesn't
                    // balloon, so the shared state is already real -- the setpoint
                    // model inherits genuine history (no flat-buffer bump). Only
                    // A/B change; shared C = [0,1] keeps the switch bumpless.
                    self.use_setpoint_model();
                    self.phase = ControlPhase::Closed;
                }
                (power_w, y_feedback)
            }
            ControlPhase::Closed => {
                self.smith
                    .step_with_feedforward(setpoint_dev, measured_dev, pump_ff, state_dist)
            }
        }
    }
}

#[embassy_executor::task]
pub async fn control_task() {
    // Gains are on the 0..1 fraction output scale (out_max=1.0 in pid.rs) --
    // the whole control path (PID out, PREHEAT_POWER, feedforward, the
    // HeaterCommand::Power fraction, and the model's input) shares that single
    // scale, so no /1000 normalization anywhere.
    //
    // SIMC PI for the gray-box model below (lambda=12s): Kc=50.276, Ti=66.68 in
    // the OLD 0..1000 input scale -> rescale by /1000 for this 0..1 pipeline:
    //   Kp = Kc/1000        = 0.0503
    //   Ki = Kc/Ti/1000     = 0.000754
    // NOTE: Ki below (0.04) is ~50x hotter than that SIMC value. It simulates
    // stable with this model at nominal, but has little robustness margin -- if
    // oscillation reappears on hardware (a sign of model<->plant mismatch),
    // lower Ki toward ~0.001 rather than touching the model.
    let pid = Pid::new(0.05, 0.00075, 0.0); // Kp, Ki, Kd

    // Gain-scheduled model: the two coefficient sets (MODEL_TRANSIENT_* /
    // MODEL_SETPOINT_*) and DEAD_TIME_S live at module scope above; the active
    // model is swapped by phase in PreheatController. Initialize with the
    // setpoint model because PreheatController::new() starts in Closed.
    let delay_steps = (DEAD_TIME_S / CONTROL_TS_S) as usize; // = 28 half-sec steps
    let smith: DiscreteSmithPredictor<Pid, MAX_DELAY_BUF> = DiscreteSmithPredictor::new(
        pid,
        MODEL_SETPOINT_A,
        MODEL_SETPOINT_B,
        MODEL_C,
        MODEL_D,
        delay_steps,
    );
    let mut controller = PreheatController::new(smith);

    let mut enabled = false;
    // Idle auto-off timer: reset on heater enable and on any pump use; when it
    // exceeds HEATER_TIMEOUT_S while enabled, the heater is disabled like a
    // switch-off (re-enable is heater-switch only).
    let mut last_activity = Instant::now();
    let mut last_temp = 25.0_f32;
    let mut last_power = 0.0_f32;
    let mut y_hat = 0.0_f32;
    // Brew feedforward shaping state: prev_pump detects the brew-start edge,
    // ff_dyn holds the decaying boost (max at start -> relaxes to steady).
    let mut prev_pump = false;
    let mut ff_dyn = 0.0_f64;
    let telem_pub = TELEM_PUB.immediate_publisher();

    // Fixed-period ticker drives the control step -- decoupled from
    // whatever rate sensor readings actually arrive at. This is what
    // keeps the discretized model and delay buffer valid.
    let mut ticker = Ticker::every(Duration::from_millis((CONTROL_TS_S * 1000.0) as u64));

    loop {
        match select(ticker.next(), SWITCH_CH.receive()).await {
            Either::First(()) => {
                // Drain any pending temp updates, keep only the latest --
                // control step timing comes from the ticker, not from
                // however many TempUpdate messages piled up this period.
                while let Ok(ControlEvent::TempUpdate(t)) = CONTROL_CH.try_receive() {
                    last_temp = t;
                }
                let t = last_temp;
                let now = Instant::now();
                let pump = PUMP_ON.load(Ordering::Relaxed);

                // Pump use = activity -> reset the idle timer (keeps the heater
                // alive while the machine is being used).
                if pump {
                    last_activity = now;
                }
                // Idle timeout: disable like a switch-off after HEATER_TIMEOUT_S
                // of no activity. Re-enabling is heater-switch only (switch branch).
                if enabled
                    && now.duration_since(last_activity)
                        >= Duration::from_secs(HEATER_TIMEOUT_S as u64)
                {
                    enabled = false;
                    last_power = 0.0;
                    controller.on_disable();
                    HEATER_CMD_CH.send(HeaterCommand::SetEnabled(false)).await;
                    warn!("heater idle timeout ({} s) -> disabled", HEATER_TIMEOUT_S);
                }

                let mut state = determine_heater_state(enabled, t, last_power);

                if matches!(state, HeaterState::Fault) && enabled {
                    enabled = false;
                    last_power = 0.0;
                    controller.on_fault();

                    HEATER_CMD_CH.send(HeaterCommand::SetEnabled(false)).await;
                    HEATER_CMD_CH.send(HeaterCommand::Power(0.0)).await;
                    info!("FAULT: temperature out of range ({} C), heater disabled", t);
                } else {
                    (last_power, y_hat) = if enabled {
                        let setpoint_dev = SETPOINT as f64 - T_AMB;
                        let measured_dev = t as f64 - T_AMB;

                        // Pump on => two coupled effects, modeled at the two
                        // different nodes they physically act on:
                        //  - ff: heater feedforward power added at the CORE
                        //    (via the model input u), before the model advances
                        //    so its state matches the real actuator.
                        //  - WATER_DIST: the brew water's heat draw at the
                        //    BLOCK/sensor node, a modeled state disturbance so
                        //    y_hat dips-and-recovers with the real temp instead
                        //    of diverging upward from the ff-only input.
                        // WATER_DIST derivation: ~90 ml/min * 4.186 J/gK *
                        // (94-20)K ~= 465 W = 0.465 power-fraction, applied as a
                        // heat sink at the block node [0, -1000/C_s] and ZOH-
                        // discretized against model_a -> [dx_core, dx_block].
                        // (Independent of ff -- ff is the heater's response,
                        // this is the physical load it's responding to.)
                        // Discretized against MODEL_SETPOINT_A -- brewing always
                        // happens in Closed on the setpoint model, so the
                        // transient model's (now divergent) dynamics don't apply.
                        const WATER_DIST: [f64; 2] = [-0.0040675, -0.4032353];
                        if pump && !prev_pump {
                            // Brew just started: kick the feedforward to max.
                            ff_dyn = FF_BREW_MAX as f64;
                        }
                        prev_pump = pump;
                        let (ff, state_dist) = if pump {
                            let f = ff_dyn;
                            // Relax toward the steady value (first-order).
                            ff_dyn += (FF_BREW_STEADY as f64 - ff_dyn) * FF_BREW_DECAY as f64;
                            (f, WATER_DIST)
                        } else {
                            (0.0, [0.0, 0.0])
                        };

                        // u_out is already a 0..1 power fraction (out_max=1.0,
                        // PREHEAT_POWER=1.0) -- same scale as HeaterCommand::Power,
                        // so no normalization step.
                        let (u_out, y_hat) =
                            controller.tick(now, measured_dev, setpoint_dev, ff, state_dist);
                        HEATER_CMD_CH.send(HeaterCommand::Power(u_out as f32)).await;
                        (u_out as f32, y_hat as f32)
                    } else {
                        (0.0, 0.0)
                    };

                    if enabled && controller.is_preheating() {
                        state = HeaterState::Preheating;
                    }
                }

                let _ = HEATER_STATE_CH.try_send(state);
                let _ = LED2_STATE_CH.try_send(state);

                let flags = (enabled as u8)
                    | ((VALVE_OPEN.load(Ordering::Relaxed) as u8) << 1)
                    | ((PUMP_ON.load(Ordering::Relaxed) as u8) << 2);

                telem_pub.publish_immediate(TelemetryFrame {
                    time_ms: now.as_millis() as u32,
                    temp: last_temp,
                    setpoint: SETPOINT,
                    power: last_power,
                    // Model output is an ambient-deviation; shift to absolute
                    // degC so it overlays temp/setpoint on the monitor.
                    y_hat: y_hat + T_AMB as f32,
                    flags,
                });
            }
            Either::Second(SwitchEvent::Toggle(e)) => {
                info!("toggle {}", e);
                enabled = e;
                let measured_dev = last_temp as f64 - T_AMB;
                if enabled {
                    // Fresh heater-on: start the 12-min idle window over.
                    last_activity = Instant::now();
                    let setpoint_dev = SETPOINT as f64 - T_AMB;
                    controller.start_preheat(Instant::now(), measured_dev, setpoint_dev);
                } else {
                    controller.on_disable();
                }
                HEATER_CMD_CH.send(HeaterCommand::SetEnabled(enabled)).await;
                let state = determine_heater_state(enabled, last_temp, last_power);
                let _ = HEATER_STATE_CH.try_send(state);
                let _ = LED2_STATE_CH.try_send(state);
            }
        }
    }
}
