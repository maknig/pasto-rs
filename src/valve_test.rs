//! Valve Test Task
//!
//! Enabled via `--features valve_test`.  Replaces the normal control task
//! with a simple valve open/close cycle (5 s each) while logging temperature.
//!
//! # Usage
//!
//! ```sh
//! cargo run --features valve_test --release 2>&1 | tee valve_test.log
//! ```

use crate::channels::{CONTROL_CH, ControlEvent, SWITCH_CH, VALVE_CMD_CH, ValveCommand};
use defmt::info;
use embassy_time::{Duration, Timer};

const VALVE_OPEN_S: u64 = 5;
const VALVE_CLOSE_S: u64 = 5;

#[embassy_executor::task]
pub async fn valve_test_task() {
    let mut cycle: u32 = 0;

    info!(
        "valve_test: starting — {}s open / {}s closed",
        VALVE_OPEN_S, VALVE_CLOSE_S
    );

    loop {
        cycle += 1;

        // --- Open valve ---
        VALVE_CMD_CH.send(ValveCommand::SetOpen(true)).await;
        info!("valve_test: cycle {} — OPEN", cycle);
        drain_and_log_temp(VALVE_OPEN_S, true).await;

        // --- Close valve ---
        VALVE_CMD_CH.send(ValveCommand::SetOpen(false)).await;
        info!("valve_test: cycle {} — CLOSED", cycle);
        drain_and_log_temp(VALVE_CLOSE_S, false).await;
    }
}

/// Drain control channel for `duration_s`, logging each temperature sample.
/// Also drains switch events so switch_task never blocks.
async fn drain_and_log_temp(duration_s: u64, valve_open: bool) {
    let deadline = embassy_time::Instant::now() + Duration::from_secs(duration_s);

    loop {
        // Drain switch events
        while SWITCH_CH.try_receive().is_ok() {}

        // Wait for next temp update (or timeout)
        let remaining = deadline.saturating_duration_since(embassy_time::Instant::now());
        if remaining == Duration::from_ticks(0) {
            break;
        }

        match embassy_time::with_timeout(remaining, CONTROL_CH.receive()).await {
            Ok(ControlEvent::TempUpdate(temp)) => {
                let temp_cdeg = (temp * 100.0) as i32;
                let open_flag: u8 = if valve_open { 1 } else { 0 };
                info!("valve_test,{},{}", temp_cdeg, open_flag);
            }
            Err(_timeout) => break,
        }
    }
}
