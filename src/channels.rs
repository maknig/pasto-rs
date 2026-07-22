use core::sync::atomic::AtomicBool;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pubsub::{PubSubChannel, Subscriber};

pub use crate::heater::HeaterState;

// --- ZC broadcast (1 producer, 2 subscribers: heater + pump) ---
// CAP=4, SUBS=3, PUBS=0 — uses immediate_publisher (no slot needed).
pub static ZC_PUB: PubSubChannel<CriticalSectionRawMutex, (), 4, 3, 0> = PubSubChannel::new();

/// Convenience alias used by heater_task and pump_task.
pub type ZcSubscriber = Subscriber<'static, CriticalSectionRawMutex, (), 4, 3, 0>;

// --- Control events (temp_task → control_task / sysid_task) ---
#[derive(Clone, Copy)]
pub enum ControlEvent {
    TempUpdate(f32),
    TimeoutReset,
}

// --- Heater commands (control_task → heater_task) ---
#[derive(Clone, Copy)]
pub enum HeaterCommand {
    Power(f32),
    SetEnabled(bool),
}

// --- Pump commands (pump_switch_task → pump_task) ---
#[derive(Clone, Copy)]
pub enum PumpCommand {
    SetEnabled(bool),
    SetSpeed(f32),
}

// --- Valve commands (pump_switch_task → valve_task) ---
#[derive(Clone, Copy)]
pub enum ValveCommand {
    SetOpen(bool),
}

// --- Switch events (switch_task → control_task) ---
#[derive(Clone, Copy)]
pub enum SwitchEvent {
    Toggle(bool),
}

pub static CONTROL_CH: Channel<CriticalSectionRawMutex, ControlEvent, 8> = Channel::new();
pub static HEATER_CMD_CH: Channel<CriticalSectionRawMutex, HeaterCommand, 4> = Channel::new();
pub static PUMP_CMD_CH: Channel<CriticalSectionRawMutex, PumpCommand, 4> = Channel::new();
pub static VALVE_CMD_CH: Channel<CriticalSectionRawMutex, ValveCommand, 4> = Channel::new();
pub static SWITCH_CH: Channel<CriticalSectionRawMutex, SwitchEvent, 4> = Channel::new();
pub static HEATER_STATE_CH: Channel<CriticalSectionRawMutex, HeaterState, 4> = Channel::new();
pub static LED2_STATE_CH: Channel<CriticalSectionRawMutex, HeaterState, 4> = Channel::new();

// --- Telemetry (control_task → monitor_task) ---

/// Atomic flags for valve/pump state, set by pump_switch_task.
pub static VALVE_OPEN: AtomicBool = AtomicBool::new(false);
pub static PUMP_ON: AtomicBool = AtomicBool::new(false);

#[derive(Clone, Copy)]
pub struct TelemetryFrame {
    pub time_ms: u32,
    pub temp: f32,
    pub setpoint: f32,
    pub power: f32,
    pub y_hat: f32,
    pub flags: u8, // bit0 = heater enabled, bit1 = valve open, bit2 = pump on
}

/// Telemetry PubSub: 1 publisher (control_task), 1 subscriber (monitor_task).
/// CAP=4 keeps a small buffer so the monitor can lag slightly without dropping.
pub static TELEM_PUB: PubSubChannel<CriticalSectionRawMutex, TelemetryFrame, 4, 1, 1> =
    PubSubChannel::new();
