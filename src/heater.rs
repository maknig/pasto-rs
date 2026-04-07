use embassy_stm32::gpio::Output;

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
        self.enabled = enabled;
        if !enabled {
            self.power = 0.0;
            self.set_low();
        }
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
