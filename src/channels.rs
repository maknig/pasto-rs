use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

#[derive(Clone, Copy)]
pub enum ControlEvent {
    TempUpdate(f32),
}

#[derive(Clone, Copy)]
pub enum HeaterCommand {
    Power(f32),
    SetEnabled(bool),
}

#[derive(Clone, Copy)]
pub enum SwitchEvent {
    Toggle(bool),
    TempUpdate(f32),
    PowerUpdate(f32),
}

pub static CONTROL_CH: Channel<CriticalSectionRawMutex, ControlEvent, 8> = Channel::new();
pub static HEATER_CMD_CH: Channel<CriticalSectionRawMutex, HeaterCommand, 4> = Channel::new();
pub static ZC_CH: Channel<CriticalSectionRawMutex, (), 8> = Channel::new();
pub static SWITCH_CH: Channel<CriticalSectionRawMutex, SwitchEvent, 8> = Channel::new();
