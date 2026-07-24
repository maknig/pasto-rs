use crate::channels::*;
use crate::config::{
    COAST_EXIT_MARGIN_C, COAST_MAX_S, FF_BREW_DECAY, FF_BREW_MAX, FF_BREW_STEADY, PREHEAT_MAX_S,
    PREHEAT_MIN_ERROR_C, PREHEAT_POWER, SETPOINT,
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
    horizon: usize,
}

impl PreheatController {
    fn new(smith: DiscreteSmithPredictor<Pid, MAX_DELAY_BUF>) -> Self {
        // Coast-prediction rollout horizon: enough steps to cover the
        // remaining dead-time-buffer contents plus a settle margin.
        let horizon = smith.delay_steps + 40;
        Self {
            phase: ControlPhase::Closed,
            smith,
            phase_start: Instant::now(),
            horizon,
        }
    }

    /// Called on enable-toggle. Starts an open-loop preheat if the error is
    /// large enough to be worth it; otherwise reseeds and goes straight to
    /// closed-loop control, same as today's small-error behavior.
    fn start_preheat(&mut self, now: Instant, measured_dev: f64, setpoint_dev: f64) {
        self.smith.reseed(measured_dev);
        if setpoint_dev - measured_dev > PREHEAT_MIN_ERROR_C as f64 {
            self.phase = ControlPhase::Preheat;
            self.phase_start = now;
        } else {
            self.phase = ControlPhase::Closed;
        }
    }

    /// Called on disable-toggle. A disable mid-preheat/coast must not
    /// silently resume afterward -- require a fresh enable-toggle.
    fn on_disable(&mut self, measured_dev: f64) {
        self.smith.reseed(measured_dev);
        self.phase = ControlPhase::Closed;
    }

    /// Called on fault-trip. Same reasoning as on_disable.
    fn on_fault(&mut self, measured_dev: f64) {
        self.smith.reseed(measured_dev);
        self.phase = ControlPhase::Closed;
    }

    fn is_preheating(&self) -> bool {
        matches!(self.phase, ControlPhase::Preheat | ControlPhase::Coast)
    }

    /// Primary per-tick step. Returns (power_w, y_hat) -- power_w is in the
    /// same Watt scale as the closed-loop path's u_out, so the caller's
    /// existing /1000. normalization applies uniformly across all phases.
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
                let d_hat = self.smith.advance_open_loop(measured_dev, power_w, state_dist);
                let peak = self.smith.model.peak_free_response(self.horizon) + d_hat;
                let elapsed_s = now.duration_since(self.phase_start).as_millis() as f64 / 1000.0;
                if peak >= setpoint_dev {
                    self.phase = ControlPhase::Coast;
                    self.phase_start = now;
                } else if elapsed_s >= PREHEAT_MAX_S as f64 {
                    warn!("preheat safety-cap timeout, forcing coast");
                    self.phase = ControlPhase::Coast;
                    self.phase_start = now;
                }
                (power_w, self.smith.model.output(0.0))
            }
            ControlPhase::Coast => {
                let power_w = pump_ff;
                self.smith.advance_open_loop(measured_dev, power_w, state_dist);
                let elapsed_s = now.duration_since(self.phase_start).as_millis() as f64 / 1000.0;
                if measured_dev >= setpoint_dev - COAST_EXIT_MARGIN_C as f64 {
                    self.phase = ControlPhase::Closed;
                } else if elapsed_s >= COAST_MAX_S as f64 {
                    warn!("coast safety-cap timeout, forcing closed-loop");
                    self.phase = ControlPhase::Closed;
                }
                (power_w, self.smith.model.output(0.0))
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
    let pid = Pid::new(0.05, 0.005, 0.0); // Kp, Ki, Kd

    // Discretized at Ts=CONTROL_TS_S=0.5s from the ACTUAL fitted gray-box
    // parameters (C_h=249.9899 J/K, C_s=572.4037 J/K, R_hs=0.0547 K/W,
    // R_h_amb=1.8929 K/W, R_s_amb=0.9957 K/W, theta=13.8224s) via the
    // Python cont2discrete pipeline. Regenerate all of these together if
    // you re-fit -- do not hand-edit one without the others, and do not
    // substitute values from a different fit run or model structure.
    //
    // let model_a = [[0.9633608564, 0.0355862980], [0.0155418546, 0.9835798425]];
    // let model_b = [0.0019631157, 0.0000156836];
    // let model_c = [0.0, 1.0];
    // let model_d = 0.0;
    // let dead_time = 13.8224; // s, from the same fit run as model_a/b above
    // let delay_steps = (dead_time / CONTROL_TS_S) as usize; // 28

    // Slow pole re-tuned to the MEASURED cooldown: the gray-box fit matched the
    // fast heat-up dynamics and cooled ~2.8x too fast (its slow pole was 537s vs
    // the ~1485s tail measured in sysid/run_1.csv). Scaling the modeled ambient
    // resistances (R_h_amb, R_s_amb) x2.77 moves the discrete slow pole to
    // ~1486s (fast pole ~9.5s unchanged) so y_hat cools at the real rate.
    let model_a = [[0.9640114978, 0.0356082855], [0.0155514574, 0.9841313768]];
    // DC gain 1904 degC/fraction (B_cont = 1000/C_h, input 0..1). Partner of the
    // slow model_a above -- the ambient resistances that set the ~25min cooldown
    // set the gain -- then trimmed +6% (x1.0606) from the physical 1795 so that
    // y_hat lands on the setpoint at the measured steady hold duty (~0.037): the
    // model settled y_hat at 90C (=1795*0.037+24) but temp holds at 94, so the
    // gain was nudged up to close that 4C steady-state gap. model_b alone is the
    // DC-gain knob (won't affect the cooldown rate set by model_a).
    let model_b = [2.0827911066, 0.0166409940];
    let model_c = [0.0, 1.0];
    let model_d = 0.0;
    // Dead time = the gray-box `theta` identified for this machine (heater ->
    // sensor transport lag). The auto-fit had collapsed this to ~1s, which left
    // ~13s of lag uncompensated in the Smith loop and produced a limit cycle
    // around setpoint once the integral clamp was opened. delay_steps derived
    // from it (not hardcoded) so the two can't drift apart.
    let dead_time = 13.8224; // s
    let delay_steps = (dead_time / CONTROL_TS_S) as usize; // ~= 28 half-sec steps

    let smith: DiscreteSmithPredictor<Pid, MAX_DELAY_BUF> =
        DiscreteSmithPredictor::new(pid, model_a, model_b, model_c, model_d, delay_steps);
    let mut controller = PreheatController::new(smith);

    let mut enabled = false;
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

                let mut state = determine_heater_state(enabled, t, last_power);

                if matches!(state, HeaterState::Fault) && enabled {
                    enabled = false;
                    last_power = 0.0;
                    controller.on_fault(t as f64 - T_AMB);

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
                        const WATER_DIST: [f64; 2] = [-0.0072949, -0.4026258];
                        let pump = PUMP_ON.load(Ordering::Relaxed);
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
                    let setpoint_dev = SETPOINT as f64 - T_AMB;
                    controller.start_preheat(Instant::now(), measured_dev, setpoint_dev);
                } else {
                    controller.on_disable(measured_dev);
                }
                HEATER_CMD_CH.send(HeaterCommand::SetEnabled(enabled)).await;
                let state = determine_heater_state(enabled, last_temp, last_power);
                let _ = HEATER_STATE_CH.try_send(state);
                let _ = LED2_STATE_CH.try_send(state);
            }
        }
    }
}
