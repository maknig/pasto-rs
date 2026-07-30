//! System Identification Task (switch-driven)
//!
//! Enabled via `--features sysid`.  Replaces the normal PID control task with a
//! manual, switch-driven excitation: the heater switch (PA2) gates full power on
//! and off, and temperature is logged continuously over RTT so the thermoblock
//! thermal model can be identified offline. There is **no fixed timing** — you
//! drive the steps by hand and cut power whenever you decide the run is done.
//!
//! # How to run
//!
//! Preferred capture is over the **monitor** (binary telemetry on USART1/PA9),
//! which logs full-resolution `f32` temp/power straight to a fit-ready CSV and
//! survives a long cooldown better than a streamed RTT pipe:
//!
//! ```sh
//! cargo run --features "sysid monitor"
//! python sysid/monitor_rerun.py /dev/ttyUSB0 --csv run.csv --no-viz
//! # drop --no-viz to also get the live Rerun view
//! ```
//!
//! The monitor CSV columns (`time,temp,setpoint,power,...`, time in seconds) are
//! directly consumable by `sysid/sysid_fit.py`; `setpoint`/`y_hat` are 0 in
//! sysid frames and ignored by the fitter. Fallback with no extra wiring is the
//! raw RTT log: `cargo run --features sysid 2>&1 | tee sysid_raw.log`.
//!
//! Then, with the machine plumbed and attended:
//!   1. Let it log a bit at power off first to capture the ambient baseline.
//!   2. Flip the heater switch ON  -> heater goes to `SYSID_POWER` (full power).
//!   3. Flip the heater switch OFF -> heater cuts to 0, capture the cooldown.
//!   4. Repeat for as many steps as you like (see "Multiple steps" below), then
//!      Ctrl-C once the final cooldown tail has flattened toward ambient.
//!   5. Fit: `python sysid/sysid_fit.py run.csv`.
//!
//! # Multiple steps
//!
//! Fitting more than one step in a single log is worthwhile here because the
//! plant is nonlinear (block->ambient loss differs between warm-up and a hot
//! hold — see the two-regime notes in `control.rs`). Stepping full power on/off
//! from several different starting temperatures spans the operating range and
//! exposes that loss-vs-temperature behaviour, which a single mid-power step
//! cannot. `sim/sysid_fit.py` drives the model with the recorded `power` column
//! (`lfilter` over the actual input), so an arbitrary on/off sequence fits
//! unchanged — to capture the nonlinearity, segment the log and fit the
//! cold-transient and near-setpoint regimes separately.
//!
//! # Safety
//!
//! `SYSID_POWER` defaults to full power (1.0) and there is **no automatic
//! over-temp cutoff in this mode** — the fault trip lives in `control_task`,
//! which this task replaces, so the heater fires exactly what it is commanded.
//! Run only on a fully plumbed machine, attended, with the heater switch in
//! reach.
//!
//! # Log format
//!
//! Each RTT line looks like (after defmt decoding):
//! ```
//! 0.123456 INFO  sysid,<time_ms>,<temp_cdeg>,<power_mpct>
//! ```
//! where `temp_cdeg` = temperature × 100 (integer, centidegrees) and
//! `power_mpct` = power × 1000 (integer, milli-fraction).

use crate::channels::{
    CONTROL_CH, ControlEvent, HEATER_CMD_CH, HeaterCommand, SWITCH_CH, SwitchEvent,
};
#[cfg(feature = "monitor")]
use crate::channels::{PUMP_ON, TELEM_PUB, TelemetryFrame, VALVE_OPEN};
use crate::config::SYSID_POWER;
#[cfg(feature = "monitor")]
use core::sync::atomic::Ordering;
use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_time::Instant;

#[embassy_executor::task]
pub async fn sysid_task() {
    // Enable heater driver, start at zero power.
    HEATER_CMD_CH.send(HeaterCommand::SetEnabled(true)).await;
    HEATER_CMD_CH.send(HeaterCommand::Power(0.0)).await;

    let start = Instant::now();
    let mut current_power: f32 = 0.0;
    #[cfg(feature = "monitor")]
    let telem_pub = TELEM_PUB.immediate_publisher();

    // Drop any switch event queued before we started listening.
    while SWITCH_CH.try_receive().is_ok() {}

    info!(
        "sysid start (switch-driven): flip heater switch ON for {} power, OFF to cut. Logging continuously.",
        SYSID_POWER,
    );

    loop {
        match select(CONTROL_CH.receive(), SWITCH_CH.receive()).await {
            // Temperature sample -> log a CSV row at the current power.
            Either::First(ControlEvent::TempUpdate(temp)) => {
                let time_ms = start.elapsed().as_millis();

                // Integers to avoid defmt float-format ambiguity:
                //   temp_cdeg  = temp  × 100   (centidegrees, i32)
                //   power_mpct = power × 1000  (milli-fraction, i32)
                let temp_cdeg = (temp * 100.0) as i32;
                let power_mpct = (current_power * 1000.0) as i32;
                info!("sysid,{},{},{}", time_ms, temp_cdeg, power_mpct);

                #[cfg(feature = "monitor")]
                {
                    let flags = 1u8 // heater always enabled in sysid
                        | ((VALVE_OPEN.load(Ordering::Relaxed) as u8) << 1)
                        | ((PUMP_ON.load(Ordering::Relaxed) as u8) << 2);
                    telem_pub.publish_immediate(TelemetryFrame {
                        time_ms: time_ms as u32,
                        temp,
                        setpoint: 0.0,
                        power: current_power,
                        y_hat: 0.0,
                        flags,
                    });
                }
            }
            Either::First(ControlEvent::TimeoutReset) => {}

            // Heater switch toggled -> full power on / off. This is the step edge.
            Either::Second(SwitchEvent::Toggle(on)) => {
                current_power = if on { SYSID_POWER } else { 0.0 };
                HEATER_CMD_CH
                    .send(HeaterCommand::Power(current_power))
                    .await;
                info!("sysid step -> power={}", current_power);
            }
        }
    }
}
