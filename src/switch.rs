use crate::channels::{SWITCH_CH, SwitchEvent};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};

const TEMP_MIN: f32 = 0.0;
const TEMP_MAX: f32 = 130.0;
const SETPOINT: f32 = 93.0;

pub enum LedState {
    Off,
    On,
    Heating,
    Error,
}

pub struct Switch {
    input: ExtiInput<'static>,
    led: Output<'static>,
    enabled: bool,
}

impl Switch {
    pub fn new(input: ExtiInput<'static>, led: Output<'static>) -> Self {
        Self {
            input,
            led,
            enabled: false,
        }
    }

    pub fn led_off(&mut self) {
        self.led.set_low();
    }

    pub fn led_on(&mut self) {
        self.led.set_high();
    }

    pub fn led_toggle(&mut self) {
        self.led.toggle();
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub fn toggle(&mut self) {
        self.enabled = !self.enabled;
        if !self.enabled {
            self.led_off();
        }
    }
}

fn determine_led_state(enabled: bool, temp: f32, power: f32) -> LedState {
    if !enabled {
        return LedState::Off;
    }

    if temp < TEMP_MIN || temp > TEMP_MAX {
        return LedState::Error;
    }

    let error = (SETPOINT - temp).abs();
    if error < 2.0 && power < 0.05 {
        LedState::On
    } else {
        LedState::Heating
    }
}

#[embassy_executor::task]
pub async fn switch_task(mut switch: Switch) {
    let mut last_temp = 0.0_f32;
    let mut last_power = 0.0_f32;

    loop {
        embassy_futures::select::select(
            switch.input.wait_for_falling_edge(),
            Timer::after(Duration::from_millis(100)),
        )
        .await;

        Timer::after(Duration::from_millis(20)).await;
        switch.toggle();
        SWITCH_CH
            .send(SwitchEvent::Toggle(switch.is_enabled()))
            .await;

        match SWITCH_CH.try_receive() {
            Ok(SwitchEvent::TempUpdate(temp)) => {
                last_temp = temp;
            }
            Ok(SwitchEvent::PowerUpdate(power)) => {
                last_power = power;
            }
            Ok(SwitchEvent::Toggle(enabled)) => {
                switch.enabled = enabled;
                if !enabled {
                    switch.led_off();
                }
            }
            Err(_) => {}
        }

        if switch.is_enabled() {
            let state = determine_led_state(switch.is_enabled(), last_temp, last_power);
            match state {
                LedState::Off => switch.led_off(),
                LedState::On => switch.led_on(),
                LedState::Heating => switch.led_on(),
                LedState::Error => switch.led_toggle(),
            }
        }
    }
}
