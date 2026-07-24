use crate::config::{SETPOINT, TEMP_MAX, TEMP_MIN};
use defmt::info;
use embassy_stm32::gpio::Output;

#[derive(Clone, Copy, defmt::Format)]
pub enum HeaterState {
    /// Heater disabled by user
    Disabled,
    /// Actively heating toward setpoint
    Heating,
    /// At setpoint (within ±2 °C), heater idle
    AtSetpoint,
    /// Temperature above setpoint + 2 °C (overshoot)
    AboveSetpoint,
    /// Actively warming open-loop before closed-loop trim engages -- covers
    /// both the preheat (full power) and coast (power off, waiting for the
    /// thermal lag to carry temp near setpoint) sub-phases
    Preheating,
    /// Temperature reading out of safe range
    Fault,
}

pub fn determine_heater_state(enabled: bool, temp: f32, _power: f32) -> HeaterState {
    if !enabled {
        return HeaterState::Disabled;
    }
    if temp < TEMP_MIN || temp > TEMP_MAX {
        return HeaterState::Fault;
    }
    let error = SETPOINT - temp;
    if error.abs() < 2.0 {
        HeaterState::AtSetpoint
    } else if error < 0.0 {
        HeaterState::AboveSetpoint
    } else {
        HeaterState::Heating
    }
}

pub struct Heater {
    gate: Output<'static>,
    power: f32,
    accum: f32,
    enabled: bool,
}

impl Heater {
    pub fn new(gate: Output<'static>) -> Self {
        Self {
            gate,
            power: 0.0,
            accum: 0.0,
            enabled: false,
        }
    }
    pub fn set_high(&mut self) {
        self.gate.set_high();
    }
    pub fn set_low(&mut self) {
        self.gate.set_low();
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        info!("set enebled {}", enabled);
        self.enabled = enabled;
        if !enabled {
            self.power = 0.0;
            self.set_low();
        }
    }
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn next_halfwave(&mut self) -> bool {
        if !self.enabled {
            return false;
        }
        self.accum += self.power;
        if self.accum >= 1.0 {
            self.accum -= 1.0;
            true
        } else {
            false
        }
    }

    pub fn set_power(&mut self, p: f32) {
        self.power = if self.enabled { p.clamp(0.0, 1.0) } else { 0.0 };
    }
}
