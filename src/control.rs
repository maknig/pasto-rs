use crate::channels::*;
use crate::pid::Pid;
use embassy_futures::select::{Either, select};

#[embassy_executor::task]
pub async fn control_task() {
    let mut pid = Pid::new(0.05, 0.01, 0.0);
    let setpoint = 93.0;
    let mut enabled = false;

    loop {
        match select(CONTROL_CH.receive(), SWITCH_CH.receive()).await {
            Either::First(ControlEvent::TempUpdate(t)) => {
                SWITCH_CH.send(SwitchEvent::TempUpdate(t)).await;

                if enabled {
                    let power = pid.update(setpoint, t, 0.05);
                    SWITCH_CH.send(SwitchEvent::PowerUpdate(power)).await;
                    HEATER_CMD_CH.send(HeaterCommand::Power(power)).await;
                } else {
                    SWITCH_CH.send(SwitchEvent::PowerUpdate(0.0)).await;
                    HEATER_CMD_CH.send(HeaterCommand::Power(0.0)).await;
                }
            }
            Either::Second(SwitchEvent::Toggle(e)) => {
                enabled = e;
                HEATER_CMD_CH.send(HeaterCommand::SetEnabled(enabled)).await;
            }
            Either::Second(_) => {}
        }
    }
}
