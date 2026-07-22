/// Target brew temperature in °C
pub const SETPOINT: f32 = 94.0;

// ---------------------------------------------------------------------------
// Pump
// ---------------------------------------------------------------------------

/// Default pump power fraction [0.0, 1.0] applied when the pump is enabled.
/// Vibratory pumps are typically run at full power; reduce only if needed.
pub const PUMP_DEFAULT_SPEED: f32 = 1.0;

/// Fault threshold — below this temperature something is wrong
pub const TEMP_MIN: f32 = 0.0;

/// Fault threshold — above this temperature something is wrong
pub const TEMP_MAX: f32 = 130.0;

// ---------------------------------------------------------------------------
// System Identification (sysid feature)
// ---------------------------------------------------------------------------

/// Heater power fraction applied during the step-up phase [0.0, 1.0].
/// 0.5 is recommended: safe for bench testing while still giving clear dynamics.
/// Use 1.0 only on a fully plumbed machine with proper thermal limits in place.
#[cfg(feature = "sysid")]
pub const SYSID_POWER: f32 = 0.5;

/// Phase 0 duration (seconds): heater off, wait for thermal equilibrium.
#[cfg(feature = "sysid")]
pub const SYSID_PHASE0_S: u32 = 60;

/// Phase 1 duration (seconds): heater at SYSID_POWER — capture heating response.
/// Should cover ≥ 3× the expected thermal time constant (τ ≈ 30–120 s for thermoblock).
#[cfg(feature = "sysid")]
pub const SYSID_PHASE1_S: u32 = 90;

/// Phase 2 duration (seconds): heater off — capture cooling response.
#[cfg(feature = "sysid")]
pub const SYSID_PHASE2_S: u32 = 60;

// ---------------------------------------------------------------------------
// Thermoblock model:  T[k+1] = MODEL_A * T[k] + MODEL_B * u[k] + MODEL_C
// Discrete-time, Ts = 0.1 s.
//
// Derived from first-order continuous-time model:
//   dT/dt = -(T - T_amb) / tau  +  K * u / tau
//
// Parameters:
//   MODEL_A = exp(-Ts / tau)
//   MODEL_B = K * (1 - MODEL_A)
//   MODEL_C = T_amb * (1 - MODEL_A)
//
// Typical espresso thermoblock (starting point before sysid):
//   tau   ≈ 30 s    → MODEL_A ≈ 0.9967
//   K     ≈ 180 °C  (max temp above ambient at full power)
//   T_amb ≈ 20 °C   → MODEL_C ≈ 0.067
//
// Fill in after running: python sim/sysid_fit.py sysid_raw.log
// ---------------------------------------------------------------------------
pub const MODEL_A: f32 = 0.9967; // placeholder — replace after sysid
pub const MODEL_B: f32 = 0.5940; // placeholder — replace after sysid
pub const MODEL_C: f32 = 0.0660; // placeholder — replace after sysid
