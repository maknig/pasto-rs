//! System Identification Task
//!
//! Enabled via `--features sysid`.  Replaces the normal PID control task
//! with a fixed step-excitation sequence and emits CSV data over RTT so the
//! thermoblock thermal model can be identified offline.
//!
//! # Excitation sequence
//!
//! | Phase | Duration          | Power        | Purpose                        |
//! |-------|-------------------|--------------|--------------------------------|
//! | 0     | `SYSID_PHASE0_S`  | 0.0          | Let system reach thermal equil.|
//! | 1     | `SYSID_PHASE1_S`  | `SYSID_POWER`| Step-up — capture heating curve|
//! | 2     | `SYSID_PHASE2_S`  | 0.0          | Step-down — capture cooling    |
//!
//! # Capturing data on the host
//!
//! ```sh
//! cargo run --features sysid 2>&1 | tee sysid_raw.log
//! # Then Ctrl-C after the sequence completes (≈ 11 minutes with defaults).
//! ```
//!
//! # Fitting the model
//!
//! ```sh
//! python sim/sysid_fit.py sysid_raw.log
//! ```
//!
//! The script prints `MODEL_A`, `MODEL_B`, `MODEL_C` ready to paste into
//! `src/config.rs`.
//!
//! # Log format
//!
//! Each RTT line looks like (after defmt decoding):
//! ```
//! 0.123456 INFO  sysid,<time_ms>,<temp_cdeg>,<power_mpct>
//! ```
//! where `temp_cdeg` = temperature × 100 (integer, centidegrees) and
//! `power_mpct` = power × 1000 (integer, milli-percent).

use crate::channels::{CONTROL_CH, ControlEvent, HEATER_CMD_CH, HeaterCommand, SWITCH_CH};
use crate::config::{SYSID_PHASE0_S, SYSID_PHASE1_S, SYSID_PHASE2_S, SYSID_POWER};
use defmt::info;
use embassy_time::Instant;
#[cfg(feature = "monitor")]
use core::sync::atomic::Ordering;
#[cfg(feature = "monitor")]
use crate::channels::{TELEM_PUB, TelemetryFrame, VALVE_OPEN, PUMP_ON};

#[embassy_executor::task]
pub async fn sysid_task() {
    // Enable heater driver, start at zero power
    HEATER_CMD_CH.send(HeaterCommand::SetEnabled(true)).await;
    HEATER_CMD_CH.send(HeaterCommand::Power(0.0)).await;

    let start = Instant::now();
    let mut current_power: f32 = 0.0;
    #[cfg(feature = "monitor")]
    let telem_pub = TELEM_PUB.immediate_publisher();

    // Phase boundary timestamps [ms]
    let t1_ms: u64 = SYSID_PHASE0_S as u64 * 1_000;
    let t2_ms: u64 = (SYSID_PHASE0_S + SYSID_PHASE1_S) as u64 * 1_000;
    let t3_ms: u64 = (SYSID_PHASE0_S + SYSID_PHASE1_S + SYSID_PHASE2_S) as u64 * 1_000;

    info!(
        "sysid start: phase0={}s phase1={}s@{} phase2={}s",
        SYSID_PHASE0_S, SYSID_PHASE1_S, SYSID_POWER, SYSID_PHASE2_S,
    );

    loop {
        // Drain switch events so switch_task never blocks
        while SWITCH_CH.try_receive().is_ok() {}

        let ControlEvent::TempUpdate(temp) = CONTROL_CH.receive().await;

        let time_ms = start.elapsed().as_millis();

        // Determine desired power for this phase
        let desired: f32 = if time_ms < t1_ms {
            0.0
        } else if time_ms < t2_ms {
            SYSID_POWER
        } else if time_ms < t3_ms {
            0.0
        } else {
            // Sequence complete — stay off forever, keep logging
            0.0
        };

        // Send power command only on transitions
        let diff = desired - current_power;
        if diff > 1e-4 || diff < -1e-4 {
            HEATER_CMD_CH.send(HeaterCommand::Power(desired)).await;
            current_power = desired;
            info!("sysid phase transition -> power={}", desired);
        }

        // Emit CSV sample (integers to avoid defmt float-format ambiguity)
        //   temp_cdeg  = temp  × 100   (centidegrees, i32)
        //   power_mpct = power × 1000  (milli-fraction, i32)
        let temp_cdeg = (temp * 100.0) as i32;
        let power_mpct = (current_power * 1000.0) as i32;
        info!("sysid,{},{},{}", time_ms, temp_cdeg, power_mpct);

        #[cfg(feature = "monitor")]
        {
            let flags = 1u8  // heater always enabled in sysid
                | ((VALVE_OPEN.load(Ordering::Relaxed) as u8) << 1)
                | ((PUMP_ON.load(Ordering::Relaxed) as u8) << 2);
            telem_pub.publish_immediate(TelemetryFrame {
                time_ms: time_ms as u32,
                temp,
                setpoint: 0.0,
                power: current_power,
                flags,
            });
        }
    }
}
