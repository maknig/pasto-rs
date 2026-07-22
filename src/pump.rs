use crate::channels::{PUMP_CMD_CH, PumpCommand, ZcSubscriber};
use crate::config::PUMP_DEFAULT_SPEED;
use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_stm32::gpio::Output;

#[derive(Clone, Copy, defmt::Format)]
pub enum PumpState {
    /// Pump disabled by user
    Disabled,
    /// Pump running
    Running,
}

pub struct Pump {
    gate: Output<'static>,
    /// Desired power fraction [0.0, 1.0]
    power: f32,
    /// Sigma-delta accumulator for burst-fire
    accum: f32,
    enabled: bool,
    /// Count of AC half-cycles the gate was active (flow estimation)
    active_halfcycles: u32,
}

impl Pump {
    pub fn new(gate: Output<'static>) -> Self {
        Self {
            gate,
            power: PUMP_DEFAULT_SPEED,
            accum: 0.0,
            enabled: false,
            active_halfcycles: 0,
        }
    }

    pub fn set_high(&mut self) {
        self.gate.set_high();
    }

    pub fn set_low(&mut self) {
        self.gate.set_low();
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        info!("pump enabled: {}", enabled);
        self.enabled = enabled;
        if !enabled {
            self.power = 0.0;
            self.set_low();
        } else {
            self.power = PUMP_DEFAULT_SPEED;
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
            self.active_halfcycles += 1;
            true
        } else {
            false
        }
    }

    /// Set the power fraction [0.0, 1.0]. Only applies when enabled.
    pub fn set_power(&mut self, p: f32) {
        self.power = if self.enabled { p.clamp(0.0, 1.0) } else { 0.0 };
    }

    /// Number of active half-cycles since last reset — proportional to flow volume.
    pub fn active_halfcycles(&self) -> u32 {
        self.active_halfcycles
    }

    /// Reset the flow estimation counter (call at shot start).
    pub fn reset_halfcycles(&mut self) {
        self.active_halfcycles = 0;
    }
}

#[embassy_executor::task]
pub async fn pump_task(mut pump: Pump, mut zc_sub: ZcSubscriber) {
    loop {
        match select(zc_sub.next_message_pure(), PUMP_CMD_CH.receive()).await {
            Either::First(_) => {
                if pump.next_halfwave() {
                    pump.set_high();
                } else {
                    pump.set_low();
                }
            }
            Either::Second(PumpCommand::SetEnabled(e)) => {
                pump.set_enabled(e);
            }
            Either::Second(PumpCommand::SetSpeed(s)) => {
                pump.set_power(s);
                info!("pump speed: {}", s);
            }
        }
    }
}
