//! Independent hardware watchdog (IWDG).
//!
//! Resets the whole MCU if the firmware stops petting it -- a last-resort safety
//! net for a mains-connected heater. On reset the heater gate (PA12) reverts to
//! low, so the SSR opens and the heater turns off. `watchdog_task` pets on its
//! own timer, so only a genuine executor hang (every task wedged) trips it.

use embassy_stm32::peripherals::IWDG;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_time::{Duration, Timer};

#[embassy_executor::task]
pub async fn watchdog_task(mut wdg: IndependentWatchdog<'static, IWDG>) {
    // Start the countdown, then feed it at ~4x margin below the timeout window.
    wdg.unleash();
    loop {
        wdg.pet();
        Timer::after(Duration::from_millis(500)).await;
    }
}
