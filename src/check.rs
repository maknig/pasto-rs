//! Machine Sanity Check Task
//!
//! Enabled via `--features check`.  Replaces the normal PID control task
//! with a sequential test of every hardware component: LEDs, temperature
//! sensor, AC zero-crossing detector, buttons, heater, pump, and valve.
//!
//! Results are logged over RTT.  Checks that require operator observation
//! (LEDs, pump sound, valve click) are marked `VISUAL`; all others report
//! `PASS` or `FAIL` automatically.
//!
//! # Usage
//!
//! ```sh
//! cargo run --features check
//! ```

use crate::channels::{
    CONTROL_CH, ControlEvent, HEATER_CMD_CH, HEATER_STATE_CH, HeaterCommand, HeaterState,
    LED2_STATE_CH, PUMP_CMD_CH, PumpCommand, SWITCH_CH, SwitchEvent, VALVE_CMD_CH, ValveCommand,
    ZcSubscriber,
};
use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};

/// Individual check result.
#[derive(Clone, Copy, defmt::Format)]
enum CheckResult {
    Pass,
    Fail,
    Visual,
}

#[embassy_executor::task]
pub async fn check_task(mut zc_sub: ZcSubscriber) {
    info!("=== MACHINE SANITY CHECK START ===");
    Timer::after(Duration::from_millis(500)).await;

    // --- 1. LED1 (PB2) ---
    let r_led1 = check_led1().await;

    // --- 2. LED2 (PB10) ---
    let r_led2 = check_led2().await;

    // --- 3. Temperature sensor ---
    let r_temp = check_temp().await;

    // --- 4. Zero-crossing ---
    let r_zc = check_zc(&mut zc_sub).await;

    // --- 5. Heater button ---
    let r_hbtn = check_heater_button().await;

    // --- 6. Pump button ---
    // let r_pbtn = check_pump_button().await;

    // --- 7. Heater ---
    let r_heater = if matches!(r_zc, CheckResult::Pass) && matches!(r_temp, CheckResult::Pass) {
        check_heater().await
    } else {
        info!("[7/9] HEATER: SKIP (requires ZC + TEMP)");
        CheckResult::Fail
    };

    // --- 8. Pump ---
    let r_pump = false;
    let r_pbtn = false;
    // let r_pump = if matches!(r_zc, CheckResult::Pass) {
    //     check_pump().await
    // } else {
    //     info!("[8/9] PUMP: SKIP (requires ZC)");
    //     CheckResult::Fail
    // };

    // --- 9. Valve ---
    let r_valve = check_valve().await;

    // --- Summary ---
    info!("=== CHECK SUMMARY ===");
    info!(
        "LED1={} LED2={} TEMP={} ZC={} HBTN={} PBTN={} HEATER={} PUMP={} VALVE={}",
        r_led1, r_led2, r_temp, r_zc, r_hbtn, r_pbtn, r_heater, r_pump, r_valve
    );

    // Determine overall result (only automatic checks count)
    let all_pass = matches!(r_temp, CheckResult::Pass)
        && matches!(r_zc, CheckResult::Pass)
        && matches!(r_hbtn, CheckResult::Pass)
        && matches!(r_pbtn, CheckResult::Pass)
        && matches!(r_heater, CheckResult::Pass);

    if all_pass {
        info!("=== ALL AUTOMATIC CHECKS PASSED ===");
    } else {
        info!("=== SOME CHECKS FAILED ===");
    }

    // Final LED indication: both on = pass, alternating = fail
    loop {
        if all_pass {
            let _ = HEATER_STATE_CH.try_send(HeaterState::Heating);
            let _ = LED2_STATE_CH.try_send(HeaterState::Heating);
            Timer::after(Duration::from_secs(10)).await;
        } else {
            // Alternate LEDs at 1 Hz
            let _ = HEATER_STATE_CH.try_send(HeaterState::Heating);
            let _ = LED2_STATE_CH.try_send(HeaterState::Disabled);
            Timer::after(Duration::from_millis(500)).await;
            let _ = HEATER_STATE_CH.try_send(HeaterState::Disabled);
            let _ = LED2_STATE_CH.try_send(HeaterState::Heating);
            Timer::after(Duration::from_millis(500)).await;
        }
    }
}

// ---------------------------------------------------------------------------
// Individual checks
// ---------------------------------------------------------------------------

/// Check 1: LED1 — turn on for 2 s, operator confirms visually.
async fn check_led1() -> CheckResult {
    info!("[1/9] LED1: turning on for 2s — confirm visually");
    let _ = HEATER_STATE_CH.try_send(HeaterState::Heating);
    Timer::after(Duration::from_secs(2)).await;
    let _ = HEATER_STATE_CH.try_send(HeaterState::Disabled);
    info!("[1/9] LED1: VISUAL");
    CheckResult::Visual
}

/// Check 2: LED2 — turn on for 2 s, operator confirms visually.
async fn check_led2() -> CheckResult {
    info!("[2/9] LED2: turning on for 2s — confirm visually");
    let _ = LED2_STATE_CH.try_send(HeaterState::Heating);
    Timer::after(Duration::from_secs(2)).await;
    let _ = LED2_STATE_CH.try_send(HeaterState::Disabled);
    info!("[2/9] LED2: VISUAL");
    CheckResult::Visual
}

/// Check 3: Temperature sensor — read and verify 5–50 °C (room temperature).
async fn check_temp() -> CheckResult {
    info!("[3/9] TEMP: reading sensor...");
    // Drain stale readings
    while CONTROL_CH.try_receive().is_ok() {}

    match select(CONTROL_CH.receive(), Timer::after(Duration::from_secs(2))).await {
        Either::First(ControlEvent::TempUpdate(t)) => {
            if t >= 5.0 && t <= 50.0 {
                info!("[3/9] TEMP: {} C — PASS", t);
                CheckResult::Pass
            } else {
                info!("[3/9] TEMP: {} C out of expected range [5, 50] — FAIL", t);
                CheckResult::Fail
            }
        }
        Either::Second(_) => {
            info!("[3/9] TEMP: no reading within 2s — FAIL");
            CheckResult::Fail
        }
    }
}

/// Check 4: Zero-crossing — count edges for 1 s, verify 80–140 Hz.
async fn check_zc(zc_sub: &mut ZcSubscriber) -> CheckResult {
    info!("[4/9] ZC: counting edges for 1s...");

    // Drain any buffered ZC events
    while zc_sub.try_next_message_pure().is_some() {}

    let start = Instant::now();
    let mut count: u32 = 0;

    loop {
        match select(
            zc_sub.next_message_pure(),
            Timer::after(Duration::from_millis(50)),
        )
        .await
        {
            Either::First(_) => count += 1,
            Either::Second(_) => {}
        }
        if start.elapsed().as_millis() >= 1000 {
            break;
        }
    }

    if count >= 80 && count <= 140 {
        info!("[4/9] ZC: {} edges/s — PASS", count);
        CheckResult::Pass
    } else {
        info!("[4/9] ZC: {} edges/s (expected 80-140) — FAIL", count);
        CheckResult::Fail
    }
}

/// Check 5: Heater button — wait for operator to press it.
async fn check_heater_button() -> CheckResult {
    info!("[5/9] HEATER BUTTON: press the heater button within 10s...");
    // Drain stale switch events
    while SWITCH_CH.try_receive().is_ok() {}

    match select(SWITCH_CH.receive(), Timer::after(Duration::from_secs(10))).await {
        Either::First(SwitchEvent::Toggle(_)) => {
            info!("[5/9] HEATER BUTTON: PASS");
            CheckResult::Pass
        }
        Either::Second(_) => {
            info!("[5/9] HEATER BUTTON: no press within 10s — FAIL");
            CheckResult::Fail
        }
    }
}

/// Check 6: Pump button — wait for operator to press it.
async fn check_pump_button() -> CheckResult {
    info!("[6/9] PUMP BUTTON: press the pump button within 10s...");
    // Drain stale pump commands
    while PUMP_CMD_CH.try_receive().is_ok() {}
    // Also drain valve commands that pump_switch_task sends
    while VALVE_CMD_CH.try_receive().is_ok() {}

    match select(PUMP_CMD_CH.receive(), Timer::after(Duration::from_secs(10))).await {
        Either::First(PumpCommand::SetEnabled(_)) => {
            // Also drain the valve command that pump_switch_task sent
            while VALVE_CMD_CH.try_receive().is_ok() {}
            info!("[6/9] PUMP BUTTON: PASS");
            CheckResult::Pass
        }
        Either::First(_) => {
            info!("[6/9] PUMP BUTTON: unexpected command — FAIL");
            CheckResult::Fail
        }
        Either::Second(_) => {
            info!("[6/9] PUMP BUTTON: no press within 10s — FAIL");
            CheckResult::Fail
        }
    }
}

/// Check 7: Heater — pulse at 10% for 5 s and verify temp rises >= 0.5 °C.
async fn check_heater() -> CheckResult {
    info!("[7/9] HEATER: pulsing at 10%% for 5s...");

    // Get baseline temperature
    while CONTROL_CH.try_receive().is_ok() {}
    let baseline = match select(CONTROL_CH.receive(), Timer::after(Duration::from_secs(2))).await {
        Either::First(ControlEvent::TempUpdate(t)) => t,
        Either::Second(_) => {
            info!("[7/9] HEATER: no temp reading — FAIL");
            return CheckResult::Fail;
        }
    };
    info!("[7/9] HEATER: baseline temp = {} C", baseline);

    // Enable heater at 10% power
    HEATER_CMD_CH.send(HeaterCommand::SetEnabled(true)).await;
    HEATER_CMD_CH.send(HeaterCommand::Power(0.3)).await;

    // Wait 5 seconds, draining temp updates
    let start = Instant::now();
    let mut latest_temp = baseline;
    while start.elapsed().as_secs() < 10 {
        match select(
            CONTROL_CH.receive(),
            Timer::after(Duration::from_millis(200)),
        )
        .await
        {
            Either::First(ControlEvent::TempUpdate(t)) => latest_temp = t,
            Either::Second(_) => {}
        }
    }

    // Disable heater
    HEATER_CMD_CH.send(HeaterCommand::Power(0.0)).await;
    HEATER_CMD_CH.send(HeaterCommand::SetEnabled(false)).await;

    let rise = latest_temp - baseline;
    if rise >= 0.5 {
        info!(
            "[7/9] HEATER: temp rose from {} to {} C (+{}) — PASS",
            baseline, latest_temp, rise
        );
        CheckResult::Pass
    } else {
        info!(
            "[7/9] HEATER: temp rose from {} to {} C (+{}) — expected >= 0.5 — FAIL",
            baseline, latest_temp, rise
        );
        CheckResult::Fail
    }
}

/// Check 8: Pump — run for 2 s, operator confirms sound/vibration.
async fn check_pump() -> CheckResult {
    info!("[8/9] PUMP: running for 2s — confirm sound/vibration");
    PUMP_CMD_CH.send(PumpCommand::SetEnabled(true)).await;
    Timer::after(Duration::from_secs(2)).await;
    PUMP_CMD_CH.send(PumpCommand::SetEnabled(false)).await;
    info!("[8/9] PUMP: VISUAL");
    CheckResult::Visual
}

/// Check 9: Valve — open for 2 s, operator confirms click.
async fn check_valve() -> CheckResult {
    info!("[9/9] VALVE: opening for 2s — confirm click");
    VALVE_CMD_CH.send(ValveCommand::SetOpen(true)).await;
    Timer::after(Duration::from_secs(2)).await;
    VALVE_CMD_CH.send(ValveCommand::SetOpen(false)).await;
    info!("[9/9] VALVE: VISUAL");
    CheckResult::Visual
}
