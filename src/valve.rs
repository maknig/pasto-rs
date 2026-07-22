use crate::channels::{VALVE_CMD_CH, ValveCommand};
use defmt::info;
use embassy_stm32::gpio::Output;

pub struct Valve {
    gate: Output<'static>,
    open: bool,
}

impl Valve {
    pub fn new(gate: Output<'static>) -> Self {
        Self { gate, open: false }
    }

    pub fn set_open(&mut self, open: bool) {
        info!("valve open: {}", open);
        self.open = open;
        if open {
            self.gate.set_high();
        } else {
            self.gate.set_low();
        }
    }

    pub fn is_open(&self) -> bool {
        self.open
    }
}

#[embassy_executor::task]
pub async fn valve_task(mut valve: Valve) {
    loop {
        match VALVE_CMD_CH.receive().await {
            ValveCommand::SetOpen(open) => {
                valve.set_open(open);
            }
        }
    }
}
