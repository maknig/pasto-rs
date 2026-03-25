use crate::channels::*;
use crate::pid::Pid;

#[embassy_executor::task]
pub async fn control_task() {
    let mut pid = Pid::new(0.05, 0.01, 0.0);
    let setpoint = 93.0;

    loop {
        match CONTROL_CH.receive().await {
            ControlEvent::TempUpdate(t) => {
                let power = pid.update(setpoint, t, 0.05);

                HEATER_CMD_CH.send(HeaterCommand::Power(power)).await;
            }

            _ => {}
        }
    }
}
