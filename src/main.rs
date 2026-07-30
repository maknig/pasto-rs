#![no_std]
#![no_main]

use defmt::{debug, info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::Config;
use embassy_stm32::adc::AdcChannel;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use crate::channels::{
    CONTROL_CH, ControlEvent, HEATER_CMD_CH, HEATER_STATE_CH, HeaterCommand, HeaterState,
    LED2_STATE_CH, ZC_PUB, ZcSubscriber,
};
use crate::heater::Heater;
use crate::pump::Pump;
use crate::switch::Switch;
use crate::temp_probe::TempProbe;
use crate::valve::Valve;

mod channels;
#[cfg(feature = "check")]
mod check;
mod config;
mod control;
mod heater;
mod kalman;
#[cfg(feature = "monitor")]
mod monitor;
mod pid;
mod pump;
mod smith;
mod switch;
#[cfg(feature = "sysid")]
mod sysid;
mod temp_probe;
mod valve;
#[cfg(feature = "valve_test")]
mod valve_test;
mod watchdog;

#[cfg(all(feature = "sysid", feature = "check"))]
compile_error!("Features `sysid` and `check` are mutually exclusive");
#[cfg(all(feature = "sysid", feature = "valve_test"))]
compile_error!("Features `sysid` and `valve_test` are mutually exclusive");
#[cfg(all(feature = "check", feature = "valve_test"))]
compile_error!("Features `check` and `valve_test` are mutually exclusive");

#[embassy_executor::task]
pub async fn zc_task(mut zc: ExtiInput<'static>) {
    let pub_ = ZC_PUB.immediate_publisher();
    loop {
        zc.wait_for_falling_edge().await;
        pub_.publish_immediate(());
    }
}

#[embassy_executor::task]
pub async fn temp_task(mut probe: TempProbe) {
    loop {
        let t = probe.read_celsius().await;
        CONTROL_CH.send(ControlEvent::TempUpdate(t)).await;
        Timer::after(Duration::from_millis(100)).await;
    }
}

/// LED1 task (PB2): indicates heater enabled / fault.
///
/// | HeaterState    | LED1 behaviour      |
/// |----------------|---------------------|
/// | Disabled       | off                 |
/// | Heating        | solid on            |
/// | AtSetpoint     | solid on            |
/// | AboveSetpoint  | solid on            |
/// | Preheating     | solid on            |
/// | Fault          | toggle every 250 ms | (2 Hz blink)
#[embassy_executor::task]
async fn led1_task(mut led: Output<'static>) {
    let mut state = HeaterState::Disabled;
    led.set_low();

    loop {
        while let Ok(s) = HEATER_STATE_CH.try_receive() {
            state = s;
        }

        match state {
            HeaterState::Disabled => {
                led.set_low();
                state = HEATER_STATE_CH.receive().await;
            }
            HeaterState::Heating
            | HeaterState::AtSetpoint
            | HeaterState::AboveSetpoint
            | HeaterState::Preheating => {
                led.set_high();
                state = HEATER_STATE_CH.receive().await;
            }
            HeaterState::Fault => {
                led.toggle();
                match select(
                    HEATER_STATE_CH.receive(),
                    Timer::after(Duration::from_millis(250)),
                )
                .await
                {
                    Either::First(s) => state = s,
                    Either::Second(_) => {}
                }
            }
        }
    }
}

/// LED2 task (PB10): indicates temperature vs setpoint.
///
/// | HeaterState    | LED2 behaviour      |
/// |----------------|---------------------|
/// | Disabled       | off                 |
/// | Heating        | solid on            | (below setpoint)
/// | AtSetpoint     | off                 |
/// | AboveSetpoint  | toggle every 250 ms | (2 Hz blink)
/// | Preheating     | toggle every 100 ms | (5 Hz blink)
/// | Fault          | off                 |
#[embassy_executor::task]
async fn led2_task(mut led: Output<'static>) {
    let mut state = HeaterState::Disabled;
    led.set_low();

    loop {
        while let Ok(s) = LED2_STATE_CH.try_receive() {
            state = s;
        }

        match state {
            HeaterState::Disabled | HeaterState::AtSetpoint | HeaterState::Fault => {
                led.set_low();
                state = LED2_STATE_CH.receive().await;
            }
            HeaterState::Heating => {
                led.set_high();
                state = LED2_STATE_CH.receive().await;
            }
            HeaterState::AboveSetpoint => {
                led.toggle();
                match select(
                    LED2_STATE_CH.receive(),
                    Timer::after(Duration::from_millis(250)),
                )
                .await
                {
                    Either::First(s) => state = s,
                    Either::Second(_) => {}
                }
            }
            HeaterState::Preheating => {
                led.toggle();
                match select(
                    LED2_STATE_CH.receive(),
                    Timer::after(Duration::from_millis(500)),
                )
                .await
                {
                    Either::First(s) => state = s,
                    Either::Second(_) => {}
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn heater_task(mut heater: Heater, mut zc_sub: ZcSubscriber) {
    loop {
        let result = select(zc_sub.next_message_pure(), HEATER_CMD_CH.receive()).await;
        match result {
            Either::First(_) => {
                if heater.next_halfwave() {
                    heater.set_high();
                } else {
                    heater.set_low();
                }
            }
            Either::Second(HeaterCommand::Power(p)) => {
                heater.set_power(p);
                // info!("setpow {}", p)
            }
            Either::Second(HeaterCommand::SetEnabled(e)) => {
                heater.set_enabled(e);
                info!("heater enabled: {}", heater.enabled())
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.mux.adcsel = mux::Adcsel::SYS;
    }
    let p = embassy_stm32::init(config);

    // Heater SSR gate — PA12, starts low / off
    let heater_pin = Output::new(p.PA12, Level::Low, Speed::Low);

    // Pump SSR gate — PA8, starts low / off
    let pump_pin = Output::new(p.PA8, Level::Low, Speed::Low);

    // AC zero-crossing detector — PA11 / EXTI11
    let zc_input = ExtiInput::new(p.PA11, p.EXTI11, Pull::Down);

    // Heater switch — PA2 / EXTI2 (falling edge triggers toggle)
    let heater_switch_input = ExtiInput::new(p.PA2, p.EXTI2, Pull::None);

    // Pump switch — PA5 / EXTI5 (falling edge triggers toggle)
    let pump_switch_input = ExtiInput::new(p.PA5, p.EXTI5, Pull::None);

    // Status LED1 — PB2, starts off
    let led1 = Output::new(p.PB2, Level::Low, Speed::Low);

    // Status LED2 — PB10, starts off
    let led2 = Output::new(p.PB10, Level::Low, Speed::Low);

    // 3-way brew valve solenoid — PB15, starts closed
    let valve_pin = Output::new(p.PB15, Level::Low, Speed::Low);

    // Temperature probe (ADC1 / PA6)
    let temp_probe = TempProbe::new(p.ADC1, p.PA6.degrade_adc());

    // Independent hardware watchdog (2 s) -- resets the MCU (heater gate -> off)
    // if the executor ever hangs. Pet by watchdog_task every 500 ms.
    let wdg = IndependentWatchdog::new(p.IWDG, 2_000_000);

    // USART1 TX (PA9) for telemetry monitor
    #[cfg(feature = "monitor")]
    let monitor_tx = {
        use embassy_stm32::usart::{Config as UartConfig, UartTx};
        let uart_cfg = UartConfig::default(); // 115200 8N1
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH4, uart_cfg).unwrap()
    };

    // Create ZC subscribers before spawning tasks
    let zc_sub_heater: ZcSubscriber = ZC_PUB.subscriber().unwrap();
    let zc_sub_pump: ZcSubscriber = ZC_PUB.subscriber().unwrap();
    #[cfg(feature = "check")]
    let zc_sub_check: ZcSubscriber = ZC_PUB.subscriber().unwrap();

    // Spawn tasks
    spawner
        .spawn(heater_task(Heater::new(heater_pin), zc_sub_heater))
        .unwrap();
    spawner
        .spawn(pump::pump_task(Pump::new(pump_pin), zc_sub_pump))
        .unwrap();
    spawner
        .spawn(valve::valve_task(Valve::new(valve_pin)))
        .unwrap();
    spawner.spawn(zc_task(zc_input)).unwrap();
    #[cfg(not(any(feature = "sysid", feature = "check", feature = "valve_test")))]
    spawner.spawn(control::control_task()).unwrap();
    #[cfg(feature = "sysid")]
    spawner.spawn(sysid::sysid_task()).unwrap();
    #[cfg(feature = "check")]
    spawner.spawn(check::check_task(zc_sub_check)).unwrap();
    #[cfg(feature = "valve_test")]
    spawner.spawn(valve_test::valve_test_task()).unwrap();
    #[cfg(feature = "monitor")]
    spawner.spawn(monitor::monitor_task(monitor_tx)).unwrap();
    spawner.spawn(temp_task(temp_probe)).unwrap();
    spawner
        .spawn(switch::heater_switch_task(Switch::new(heater_switch_input)))
        .unwrap();
    spawner
        .spawn(switch::pump_switch_task(Switch::new(pump_switch_input)))
        .unwrap();
    spawner.spawn(led1_task(led1)).unwrap();
    spawner.spawn(led2_task(led2)).unwrap();
    spawner.spawn(watchdog::watchdog_task(wdg)).unwrap();
}
