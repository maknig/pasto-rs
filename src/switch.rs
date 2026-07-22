use crate::channels::{
    PUMP_CMD_CH, PUMP_ON, PumpCommand, SWITCH_CH, SwitchEvent, VALVE_CMD_CH, VALVE_OPEN,
    ValveCommand,
};
use core::sync::atomic::Ordering;
use embassy_stm32::exti::ExtiInput;
use embassy_time::{Duration, Timer};

pub struct Switch {
    input: ExtiInput<'static>,
    enabled: bool,
}

impl Switch {
    pub fn new(input: ExtiInput<'static>) -> Self {
        Self {
            input,
            enabled: false,
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub fn toggle(&mut self) {
        self.enabled = !self.enabled;
    }
}

#[embassy_executor::task]
pub async fn heater_switch_task(mut switch: Switch) {
    loop {
        // Wait for button press (falling edge)
        switch.input.wait_for_falling_edge().await;
        // Debounce
        Timer::after(Duration::from_millis(20)).await;

        switch.toggle();
        SWITCH_CH
            .send(SwitchEvent::Toggle(switch.is_enabled()))
            .await;
    }
}

/// Pump switch task — toggles both pump and 3-way brew valve together.
#[embassy_executor::task]
pub async fn pump_switch_task(mut switch: Switch) {
    loop {
        switch.input.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(20)).await;

        switch.toggle();
        let on = switch.is_enabled();
        PUMP_ON.store(on, Ordering::Relaxed);
        VALVE_OPEN.store(on, Ordering::Relaxed);
        PUMP_CMD_CH.send(PumpCommand::SetEnabled(on)).await;
        VALVE_CMD_CH.send(ValveCommand::SetOpen(on)).await;
        CONTROL_CH.send(crate::channels::ControlEvent::TimeoutReset).await;
    }
}
