/// Target brew temperature in °C
pub const SETPOINT: f32 = 92.0;

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

/// Idle auto-off: disable the heater this long after the last activity (heater
/// enable or pump use), exactly like a manual switch-off. Pump use resets the
/// timer; re-enabling is heater-switch only. 720 s = 12 min.
pub const HEATER_TIMEOUT_S: u32 = 720;

// ---------------------------------------------------------------------------
// Preheat / coast (model-predictive open-loop warmup, see control.rs)
// ---------------------------------------------------------------------------

/// Below this setpoint deviation on enable, skip preheat/coast entirely and
/// go straight to closed-loop control -- small errors don't need it.
pub const PREHEAT_MIN_ERROR_C: f32 = 12.0;

/// Fixed open-loop power (0..1 fraction) applied during the preheat phase.
/// 1.0 = full power, matching what the PID already does today under
/// saturation at a large cold-start error -- this just makes it explicit
/// and boundable by the model-predictive cutoff instead of open-ended PID
/// saturation. Same 0..1 scale as HeaterCommand::Power and the identified
/// model's input (see control.rs).
pub const PREHEAT_POWER: f32 = 1.0;

/// Safety-cap fallback (seconds): force the Preheat -> Coast transition
/// even if the measured-lead cutoff never fires.
pub const PREHEAT_MAX_S: u32 = 180;

/// Preheat cuts heater power when measured temp reaches SETPOINT - this margin,
/// then coasts the residual thermal lag up the rest of the way. Model-
/// INDEPENDENT (replaces the old peak_free_response prediction, which fired late
/// because of the ~14 s sensor dead time). Larger = cut earlier / lower peak;
/// smaller = cut later / higher peak. Tune against the real warmup peak: raise
/// if it overshoots 94, lower if it falls short.
pub const PREHEAT_LEAD_C: f32 = 14.0;

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
/// decayed -- rejects the water's continuous heat draw. Set near the physical
/// draw (~0.465 = 90 ml/min * 4.186 J/gK * 74 K = 465 W of 1000 W); lower under-
/// compensates and lets temp dip, higher risks a bump.
pub const FF_BREW_STEADY: f32 = 0.45;

/// Feedforward at the instant brewing starts. Front-loads the core so it heats
/// fast and the block-temp dip is minimized, instead of waiting out the
/// core->block thermal lag. Relaxes to FF_BREW_STEADY.
pub const FF_BREW_MAX: f32 = 1.0;

/// Per-control-tick relaxation of the brew feedforward from FF_BREW_MAX toward
/// FF_BREW_STEADY (first-order): ff += (steady - ff) * FF_BREW_DECAY.
/// ~0.06 gives an ~8 s boost time constant at CONTROL_TS_S = 0.5 s.
pub const FF_BREW_DECAY: f32 = 0.03;

// ---------------------------------------------------------------------------
// System Identification (sysid feature)
// ---------------------------------------------------------------------------

/// Heater power fraction [0.0, 1.0] applied while the heater switch is ON during
/// a switch-driven sysid run (see sysid.rs). 1.0 = full power for the richest
/// dynamics. There is NO automatic over-temp cutoff in sysid mode (the fault
/// trip lives in control_task, which sysid replaces), so run only on a fully
/// plumbed machine, attended, with the switch in reach.
#[cfg(feature = "sysid")]
pub const SYSID_POWER: f32 = 1.0;
