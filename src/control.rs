use crate::channels::*;
use crate::config::SETPOINT;
use crate::heater::determine_heater_state;
use crate::pid::Pid;
use core::sync::atomic::Ordering;
use defmt::{debug, info, warn};
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

#[embassy_executor::task]
pub async fn control_task() {
    // SIMC PI tuning matching THIS model (lambda=12s): Kc=50.2756447862,
    // Ti=66.6790624681 -> Kp=Kc, Ki=Kc/Ti.
    let pid = Pid::new(70., 15., 0.0); // Kp, Ki, Kd

    // Discretized at Ts=CONTROL_TS_S=0.5s from the ACTUAL fitted gray-box
    // parameters (C_h=249.9899 J/K, C_s=572.4037 J/K, R_hs=0.0547 K/W,
    // R_h_amb=1.8929 K/W, R_s_amb=0.9957 K/W, theta=13.8224s) via the
    // Python cont2discrete pipeline. Regenerate all of these together if
    // you re-fit -- do not hand-edit one without the others, and do not
    // substitute values from a different fit run or model structure.
    let model_a = [[0.9633608564, 0.0355862980], [0.0155418546, 0.9835798425]];
    let model_b = [0.0019631157, 0.0000156836];
    let model_c = [0.0, 1.0];
    let model_d = 0.0;

    let dead_time = 13.8224; // s, from the same fit run as model_a/b above
    let delay_steps = (dead_time / CONTROL_TS_S) as usize; // 28

    // MAX_DELAY_BUF: compile-time buffer capacity, chosen here to give
    // headroom above delay_steps (28) in case theta is re-identified
    // larger later without a smith.rs edit. Bump if delay_steps ever
    // exceeds this.
    const MAX_DELAY_BUF: usize = 32;
    let mut smith: DiscreteSmithPredictor<Pid, MAX_DELAY_BUF> =
        DiscreteSmithPredictor::new(pid, model_a, model_b, model_c, model_d, delay_steps);

    let mut enabled = false;
    let mut last_temp = 25.0_f32;
    let mut last_power = 0.0_f32;
    let mut y_hat = 0.0_f32;
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

                let state = determine_heater_state(enabled, t, last_power);

                if matches!(state, HeaterState::Fault) && enabled {
                    enabled = false;
                    last_power = 0.0;
                    smith.reset();

                    HEATER_CMD_CH.send(HeaterCommand::SetEnabled(false)).await;
                    HEATER_CMD_CH.send(HeaterCommand::Power(0.0)).await;
                    info!("FAULT: temperature out of range ({} C), heater disabled", t);
                } else {
                    (last_power, y_hat) = if enabled {
                        let setpoint_dev = SETPOINT as f64 - T_AMB;
                        let measured_dev = t as f64 - T_AMB;

                        // Feedforward MUST be added before the internal
                        // model advances, or the model's state silently
                        // diverges from what's actually applied to the
                        // real plant every time the pump is on.
                        let ff = if PUMP_ON.load(Ordering::Relaxed) {
                            530.
                        } else {
                            0.0
                        };

                        let (u_out, y_hat) =
                            smith.step_with_feedforward(setpoint_dev, measured_dev, ff);
                        let u_normalized = u_out / 1000.;
                        HEATER_CMD_CH
                            .send(HeaterCommand::Power(u_normalized as f32))
                            .await;
                        (u_normalized as f32, y_hat as f32)
                    } else {
                        (0.0, 0.0)
                    };
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
                    y_hat,
                    flags,
                });
            }
            Either::Second(SwitchEvent::Toggle(e)) => {
                info!("toggle {}", e);
                enabled = e;
                if !enabled {
                    smith.reset();
                }
                HEATER_CMD_CH.send(HeaterCommand::SetEnabled(enabled)).await;
                let state = determine_heater_state(enabled, last_temp, last_power);
                let _ = HEATER_STATE_CH.try_send(state);
                let _ = LED2_STATE_CH.try_send(state);
            }
        }
    }
}
