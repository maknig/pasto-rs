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
// Preheat / coast (model-predictive open-loop warmup, see control.rs)
// ---------------------------------------------------------------------------

/// Below this setpoint deviation on enable, skip preheat/coast entirely and
/// go straight to closed-loop control -- small errors don't need it.
pub const PREHEAT_MIN_ERROR_C: f32 = 10.0;

/// Fixed open-loop power (0..1 fraction) applied during the preheat phase.
/// 1.0 = full power, matching what the PID already does today under
/// saturation at a large cold-start error -- this just makes it explicit
/// and boundable by the model-predictive cutoff instead of open-ended PID
/// saturation. Same 0..1 scale as HeaterCommand::Power and the identified
/// model's input (see control.rs).
pub const PREHEAT_POWER: f32 = 1.0;

/// Safety-cap fallback (seconds): force the Preheat -> Coast transition
/// even if the model-predictive cutoff never predicts crossing setpoint.
pub const PREHEAT_MAX_S: u32 = 180;

/// Hand off from Coast to closed-loop control once measured temp is within
/// this many °C of setpoint.
pub const COAST_EXIT_MARGIN_C: f32 = 3.0;

/// Safety-cap fallback (seconds): force the Coast -> Closed transition even
/// if temp never gets within COAST_EXIT_MARGIN_C of setpoint.
pub const COAST_MAX_S: u32 = 120;

// ---------------------------------------------------------------------------
// Brew feedforward shaping (pump on -- see control.rs)
// ---------------------------------------------------------------------------

/// Steady brew feedforward (0..1 fraction) once the start-of-brew boost has
/// decayed -- rejects the water's continuous heat draw.
pub const FF_BREW_STEADY: f32 = 0.3;

/// Feedforward at the instant brewing starts. Front-loads the core so it heats
/// fast and the block-temp dip is minimized, instead of waiting out the
/// core->block thermal lag. Relaxes to FF_BREW_STEADY.
pub const FF_BREW_MAX: f32 = 1.0;

/// Per-control-tick relaxation of the brew feedforward from FF_BREW_MAX toward
/// FF_BREW_STEADY (first-order): ff += (steady - ff) * FF_BREW_DECAY.
/// ~0.06 gives an ~8 s boost time constant at CONTROL_TS_S = 0.5 s.
pub const FF_BREW_DECAY: f32 = 0.06;

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
/// Must run ≈ 3× the slow time constant (τ_slow ≈ 13 min) so the cooldown tail
/// flattens toward ambient — that decay rate is what pins the slow pole and
/// hence the DC gain / block→ambient loss. A short cooldown leaves the gain
/// unidentifiable (the heat-up alone looks like an integrator).
#[cfg(feature = "sysid")]
pub const SYSID_PHASE2_S: u32 = 2400;

