#![no_std]
#![no_main]

//#[cfg(feature = "defmt")]

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};

use embassy_stm32::Config;
use embassy_stm32::peripherals;
use embassy_stm32::wdg::IndependentWatchdog;

use crate::channels::{CONTROL_CH, ControlEvent, HEATER_CMD_CH, HeaterCommand, ZC_CH};
use crate::heater::Heater;
use crate::pid::Pid;
use crate::temp_probe::TempProbe;
use defmt::{debug, error, info, write};
use embassy_stm32::adc::{Adc, AdcChannel, Resolution, SampleTime};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

mod channels;
mod control;
mod heater;
mod kalman;
mod pid;
mod temp_probe;
mod watchdog;

#[embassy_executor::task]
pub async fn zc_task(mut zc: ExtiInput<'static>) {
    loop {
        zc.wait_for_falling_edge().await;
        let _ = ZC_CH.try_send(());
    }
}
// #[embassy_executor::task]
// pub async fn watchdog_task(mut wdg: IndependentWatchdog<'static, peripherals::IWDG>) {
//     loop {
//         wdg.pet();
//         Timer::after(Duration::from_millis(500)).await;
//     }
// }

#[embassy_executor::task]
pub async fn temp_task(mut probe: TempProbe) {
    loop {
        let t = probe.read_celsius().await;

        info!("Temp: {}", t);
        CONTROL_CH.send(ControlEvent::TempUpdate(t)).await;

        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
pub async fn heater_task(mut heater: Heater) {
    loop {
        let result = select(ZC_CH.receive(), HEATER_CMD_CH.receive()).await;

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
            }
        }
    }
}
#[embassy_executor::task]
pub async fn control_task() {
    let mut pid = Pid::new(0.05, 0.01, 0.0);
    let setpoint = 93.0;

    loop {
        let ControlEvent::TempUpdate(t) = CONTROL_CH.receive().await;
        let power = pid.update(setpoint, t, 0.05);

        HEATER_CMD_CH.send(HeaterCommand::Power(power)).await;
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

    // -------- Pins directly in main --------

    let heater_pin = Output::new(p.PA5, Level::Low, Speed::Low);

    let zc_input = ExtiInput::new(p.PA0, p.EXTI0, Pull::Down);

    // -------- Watchdog --------
    // let wdg = IndependentWatchdog::new(p.IWDG, 2_000_000); // ~2s timeout
    //
    let temp_probe = TempProbe::new(p.ADC1, p.PA6.degrade_adc());

    // -------- Spawn Tasks --------
    spawner.spawn(heater_task(Heater::new(heater_pin))).unwrap();

    spawner.spawn(zc_task(zc_input)).unwrap();

    spawner.spawn(control_task()).unwrap();

    spawner.spawn(temp_task(temp_probe)).unwrap();

    // spawner.spawn(watchdog_task(wdg)).unwrap();
}
