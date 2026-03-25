use crate::channels::{HEATER_CMD_CH, HeaterCommand, ZC_CH};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;

pub struct Heater {
    gate: Output<'static>,
    power: f32,
    accum: f32,
}

impl Heater {
    pub fn new(gate: Output<'static>) -> Self {
        Self {
            gate,
            power: 0.0,
            accum: 0.0,
        }
    }
    pub fn set_high(&mut self) {
        self.gate.set_high();
    }
    pub fn set_low(&mut self) {
        self.gate.set_low();
    }

    pub fn next_halfwave(&mut self) -> bool {
        self.accum += self.power;
        if self.accum >= 1.0 {
            self.accum -= 1.0;
            true
        } else {
            false
        }
    }

    pub fn set_power(&mut self, p: f32) {
        self.power = p.clamp(0.0, 1.0);
    }
}
