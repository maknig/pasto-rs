#!/usr/bin/env python3
"""Grey-box 2-lump thermal model identification for the Quickmill thermoblock.

Fits a physically-structured second-order model to a step-response log and emits
the discretized state-space constants for `src/control.rs` (Smith predictor).

Model
-----
Two thermal lumps -- node 1 = core/heater (heat capacity C1, temp T1, receives
the heater power P), node 2 = aluminum block (C2, T2, what the PA6 sensor reads):

    C1 * dT1/dt = P            - k12*(T1 - T2)
    C2 * dT2/dt = k12*(T1 - T2) - k2a*T2          (y = T2, measured)

k12 = core<->block conductance, k2a = block->ambient conductance. Temperatures
are deviations from ambient. The transfer function P -> T2 is second-order with
NO zero:

                    k12
    G(s) = --------------------------------------------------
            C1*C2 s^2 + (C1*k12 + C2*k12 + C1*k2a) s + k12*k2a

Identifiability
---------------
A step/cool experiment on T2 alone yields only THREE numbers -- the two poles
and the DC gain -- but the physics has FOUR unknowns (C1, C2, k12, k2a). So the
model is structurally non-identifiable from block-temp + power alone: exactly one
unobservable degree of freedom. What IS recoverable directly:

    k2a  = 1 / DC_gain            (block->ambient loss, from steady state)
    poles / time constants        (fast = core<->block, slow = (C1+C2)/k2a)

To pin the missing DOF and get absolute C1 / k12 (=1/R12), supply ONE physical
anchor: the block thermal mass  C2 = m_Al * c_Al  (c_Al ~ 897 J/(kg*K)). Weigh
the block. Everything else then falls out analytically. Without an anchor the
tool still emits valid discrete matrices (the I/O behavior -- all the Smith
predictor needs -- is identical for the whole family); it just picks a canonical
balanced mass split and flags the internal-state scale as assumption-dependent.

Usage
-----
    python sysid/sysid_fit.py sysid/sysid_resampled.csv
    python sysid/sysid_fit.py sysid/sysid_resampled.csv --block-mass 550   # grams Al
    python sysid/sysid_fit.py sysid/sysid_resampled.csv --c2 490           # J/K directly
    python sysid/sysid_fit.py sysid/sysid_resampled.csv --plot fit.png
    python sysid/sysid_fit.py sysid/sysid_resampled.csv --no-dead-time --ts 0.5

The emitted `model_a/model_b/model_c/model_d`, `dead_time` and `delay_steps` are
ready to paste into control.rs -- regenerate them all together (see the warning
comments there); never hand-edit one constant in isolation.
"""

from __future__ import annotations

import argparse
import csv
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import cont2discrete, lfilter

C_AL = 897.0  # J/(kg*K), specific heat of aluminum


# --------------------------------------------------------------------------- #
# Data loading
# --------------------------------------------------------------------------- #
def load_csv(path: str, target_dt: float | None = None):
    """Load (t, temp, u) from a CSV. Column names are matched case-insensitively
    against known aliases; a leading unnamed index column is ignored.

    If target_dt is given, always resample onto that uniform grid (regardless
    of whether the native sampling is already uniform) -- useful to coarsen a
    fast native sensor rate (e.g. 0.1s) down to a chosen step size."""
    time_keys = {"time", "t", "time_s", "t_s", "time_ms"}
    temp_keys = {"temp", "temperature", "y", "temp_c"}
    u_keys = {"u", "power", "p", "heater"}

    with open(path, newline="") as fh:
        reader = csv.reader(fh)
        header = next(reader)
        cols = {name.strip().lower(): i for i, name in enumerate(header)}

        def find(keys):
            for k in keys:
                if k in cols:
                    return cols[k], k
            return None, None

        ti, tkey = find(time_keys)
        yi, _ = find(temp_keys)
        ui, _ = find(u_keys)
        if ti is None or yi is None or ui is None:
            sys.exit(f"could not find time/temp/u columns in header: {header}")

        rows = [r for r in reader if r and r[ti].strip() != ""]

    t = np.array([float(r[ti]) for r in rows])
    y = np.array([float(r[yi]) for r in rows])
    u = np.array([float(r[ui]) for r in rows])

    # Drop stale leading frame(s): a serial buffer can hand back bytes left
    # over from a prior session before the real, freshly-reset device clock
    # starts -- shows up as a big backwards jump in time. Keep only the
    # final contiguous monotonic run.
    resets = np.where(np.diff(t) < -1.0)[0]
    if len(resets):
        start = int(resets[-1]) + 1
        print(f"[warn] dropped {start} leading stale row(s) before a time reset")
        t, y, u = t[start:], y[start:], u[start:]

    if tkey == "time_ms":
        t = t / 1000.0

    # Enforce a uniform sample interval (the model discretization assumes it).
    dt = np.diff(t)
    dt_med = float(np.median(dt))
    if target_dt is not None:
        span = t[-1] - t[0]
        n = int(round(span / target_dt)) + 1
        tu = t[0] + target_dt * np.arange(n)
        y = np.interp(tu, t, y)
        u = np.interp(tu, t, u)
        t = tu
        dt_med = target_dt
        print(f"[info] resampled from native dt={float(np.median(dt)):g}s to "
              f"dt={target_dt:g}s ({n} pts)")
    elif np.any(np.abs(dt - dt_med) > 1e-6 * max(1.0, dt_med) + 1e-9):
        span = t[-1] - t[0]
        n = int(round(span / dt_med)) + 1
        tu = t[0] + dt_med * np.arange(n)
        y = np.interp(tu, t, y)
        u = np.interp(tu, t, u)
        t = tu
        print(f"[warn] non-uniform sampling; resampled to dt={dt_med:g}s ({n} pts)")
    return t, y, u, dt_med


# --------------------------------------------------------------------------- #
# Model simulation (I/O only -- realization-invariant, used for the fit)
# --------------------------------------------------------------------------- #
def simulate(theta, u, dt, fit_delay, tau_slow_fixed=None):
    """Simulate y = G(s)*u for G = g/((tau_fast s+1)(tau_slow s+1)) * exp(-L s).

    theta = [log g, log tau_fast, (log tau_slow if tau_slow_fixed is None),
             (log L if fit_delay)]
    """
    g = np.exp(theta[0])
    tau_fast = np.exp(theta[1])
    if tau_slow_fixed is not None:
        tau_slow = tau_slow_fixed
        idx_L = 2
    else:
        tau_slow = np.exp(theta[2])
        idx_L = 3
    L = np.exp(theta[idx_L]) if fit_delay else 0.0

    a1 = 1.0 / tau_fast + 1.0 / tau_slow
    a0 = 1.0 / (tau_fast * tau_slow)
    b0 = g * a0
    num_d, den_d, _ = cont2discrete(([b0], [1.0, a1, a0]), dt, method="zoh")
    y = lfilter(num_d.ravel(), den_d.ravel(), shift_input(u, L, dt))
    return y


def shift_input(u, L, dt):
    """Delay input by L seconds (integer-sample shift, holding the first value)."""
    d = int(round(L / dt))
    if d <= 0:
        return u
    return np.concatenate([np.full(d, u[0]), u[:-d]])


def fit(t, y, u, dt, fit_delay, tau_slow_fixed=None):
    # Initial guess from crude data features.
    g0 = max((y.max() - y.min()) / max(u.max(), 1e-3), 1e-3)
    span = t[-1] - t[0]
    if tau_slow_fixed is not None:
        theta0 = [np.log(g0), np.log(span / 30.0)]
        lb = [np.log(1e-4), np.log(dt)]
        ub = [np.log(1e4), np.log(tau_slow_fixed)]
    else:
        theta0 = [np.log(g0), np.log(span / 6.0), np.log(span / 30.0)]
        lb = [np.log(1e-4), np.log(dt), np.log(dt)]
        ub = [np.log(1e4), np.log(span * 5), np.log(span * 5)]
    if fit_delay:
        theta0.append(np.log(max(dt, 1.0)))
        lb.append(np.log(dt / 10))
        ub.append(np.log(span / 3))

    res = least_squares(
        lambda th: simulate(th, u, dt, fit_delay, tau_slow_fixed) - y,
        theta0, bounds=(lb, ub), method="trf", x_scale="jac",
    )
    return res


def estimate_tail_tau(t, y, u, min_duration=600.0, min_points=30):
    """Estimate tau_slow directly from the deep cooldown tail via a
    log-linear regression on the back half of the post-transition decay --
    avoids the compromise a single global 2-lump fit is forced into when the
    data has more timescales than 2 poles can represent. Returns
    (tau_slow, residual_std_degC), or None if there isn't enough tail."""
    trans_idx = np.where(np.abs(np.diff(u)) > 1e-6)[0] + 1
    if len(trans_idx) == 0:
        return None
    last_trans_t = t[trans_idx[-1]]
    tail_start = last_trans_t + 0.5 * (t[-1] - last_trans_t)
    mask = (t >= tail_start) & (y > 1.0)
    if (t[-1] - tail_start) < min_duration or mask.sum() < min_points:
        return None
    tt, yy = t[mask], y[mask]
    A = np.vstack([tt, np.ones_like(tt)]).T
    coef, *_ = np.linalg.lstsq(A, np.log(yy), rcond=None)
    slope = coef[0]
    if slope >= 0:
        return None
    tau_slow = -1.0 / slope
    resid_std = float((yy - np.exp(A @ coef)).std())
    return tau_slow, resid_std


# --------------------------------------------------------------------------- #
# Physical-parameter recovery + discretization for control.rs
# --------------------------------------------------------------------------- #
def physical_realization(g, tau1, tau2, c2=None):
    """Recover (C1, C2, k12, k2a) from the fitted G. If c2 is None, use the
    canonical balanced split C2* = sqrt(k2a/beta) that maximizes the C1>0 margin
    (feasible whenever the poles are distinct, i.e. an overdamped real-pole pair).
    Returns (C1, C2, k12, k2a) and a bool 'anchored'."""
    a1 = 1.0 / tau1 + 1.0 / tau2      # sum of pole rates
    a0 = 1.0 / (tau1 * tau2)          # product of pole rates
    k2a = 1.0 / g
    beta = g * a0                     # = k12/(C1*C2)

    anchored = c2 is not None
    if c2 is None:
        c2 = np.sqrt(k2a / beta)      # canonical balanced split

    c1 = (a1 - beta * c2 - k2a / c2) / beta
    if c1 <= 0:
        raise ValueError(
            f"infeasible split: C1={c1:.4g}<=0 for C2={c2:.4g} J/K.\n"
            f"       Feasible C2 lies near {np.sqrt(k2a / beta):.4g} J/K "
            f"(check the block-mass anchor)."
        )
    k12 = beta * c1 * c2
    return c1, c2, k12, k2a, anchored


def discretize(c1, c2, k12, k2a, ts):
    """Continuous physical 2-lump -> discrete (Ad, Bd, C, D) at sample time ts,
    with the sensor node as the SECOND state so C = [0, 1] (matches control.rs)."""
    A = np.array([[-k12 / c1, k12 / c1],
                  [k12 / c2, -(k12 + k2a) / c2]])
    B = np.array([[1.0 / c1], [0.0]])
    C = np.array([[0.0, 1.0]])
    D = np.array([[0.0]])
    Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, D), ts, method="zoh")
    return Ad, Bd.ravel(), Cd.ravel(), float(Dd.ravel()[0])


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #
def overshoot_features(t, y, u):
    """Locate heater-off, then the post-cutoff peak (the 2-lump fingerprint)."""
    du = np.diff(u)
    downs = np.where(du < -1e-6)[0]
    if len(downs) == 0:
        return None
    off = downs[-1] + 1
    seg = slice(off, len(y))
    pk = off + int(np.argmax(y[seg]))
    return dict(
        t_off=t[off], y_off=y[off],
        t_peak=t[pk], y_peak=y[pk],
        dt_peak=t[pk] - t[off],
        overshoot=y[pk] - y[off],
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="step-response CSV (columns: time, temp, u)")
    ap.add_argument("--block-mass", type=float, default=None,
                    metavar="GRAMS", help="aluminum block mass in grams (anchor C2)")
    ap.add_argument("--c2", type=float, default=None, metavar="J_PER_K",
                    help="block heat capacity C2 in J/K (overrides --block-mass)")
    ap.add_argument("--ts", type=float, default=0.5,
                    help="control-loop sample time for discretization (default 0.5)")
    ap.add_argument("--no-dead-time", action="store_true",
                    help="fix pure dead time L=0 instead of fitting it")
    ap.add_argument("--resample-dt", type=float, default=1.0, metavar="SECONDS",
                    help="resample the log onto this uniform step size before "
                         "fitting (default 1.0; pass 0 to keep native rate)")
    ap.add_argument("--tail-window", type=float, default=300.0, metavar="SECONDS",
                    help="keep only this much cooldown after the last on/off "
                         "transition for fitting (default 300; a long tail "
                         "swamps the fast/overshoot dynamics -- pass 0 to "
                         "keep the whole recording)")
    ap.add_argument("--plot", metavar="PNG", default=None,
                    help="save a fit-vs-data plot to this path")
    args = ap.parse_args()

    resample_dt = args.resample_dt if args.resample_dt > 0 else None
    t, y, u, dt = load_csv(args.csv, target_dt=resample_dt)
    fit_delay = not args.no_dead_time

    # The model represents temperature as a deviation from ambient with zero
    # initial state -- baseline against the pre-heat segment (samples before
    # u first goes nonzero) rather than requiring pre-subtracted input data.
    heating = u > 1e-6
    first_heat = int(np.argmax(heating)) if heating.any() else len(u)
    baseline = float(y[:first_heat].mean()) if first_heat > 0 else float(y[0])
    print(f"[info] baselining against pre-heat ambient estimate {baseline:.2f} degC")
    y = y - baseline

    # Estimate tau_slow from the deep tail of the FULL record (before any
    # truncation below) -- a clean log-linear regression on the back half of
    # the cooldown. Informational only: feeding this into the truncated-
    # window fit below was tried and made the overshoot fit worse (the short
    # window is still under the influence of a timescale in between tau_fast
    # and this asymptotic one, which a 2-lump model can't represent
    # alongside it) -- so tau_slow stays free in the actual fit.
    tail_est = estimate_tail_tau(t, y, u)
    if tail_est is not None:
        tau_slow_tail, tail_resid_std = tail_est
        print(f"[info] tau_slow(tail-only, informational)={tau_slow_tail:.1f}s "
              f"(residual std={tail_resid_std:.3f} degC) -- not used in the fit")
    else:
        print("[info] insufficient tail for tau_slow regression")

    # A long cooldown tail (needed to pin the slow pole/DC gain) has more
    # timescales in it than a 2-lump model can represent at once, and its
    # sheer point count swamps the fast, information-dense samples right
    # after each on/off transition -- fitting the whole recording blurs the
    # fast pole/overshoot shape no matter how it's weighted. Keep only a
    # limited cooldown after the last transition instead.
    if args.tail_window > 0:
        trans_idx = np.where(np.abs(np.diff(u)) > 1e-6)[0] + 1
        if len(trans_idx):
            last_trans_t = t[trans_idx[-1]]
            keep = t <= last_trans_t + args.tail_window
            if keep.sum() < len(t):
                t, y, u = t[keep], y[keep], u[keep]
                print(f"[info] truncated to t<={t[-1]:.0f}s ({len(t)} pts) for fitting")

    res = fit(t, y, u, dt, fit_delay)
    g = float(np.exp(res.x[0]))
    # theta[1]/theta[2] aren't constrained to any particular order -- sort
    # so tau_slow/tau_fast are labeled correctly regardless of which one the
    # optimizer landed on (a1/a0 are symmetric in the two, so this doesn't
    # affect the fit itself, only the reported labels).
    tau_a, tau_b = float(np.exp(res.x[1])), float(np.exp(res.x[2]))
    tau_slow, tau_fast = max(tau_a, tau_b), min(tau_a, tau_b)
    L = float(np.exp(res.x[3])) if fit_delay else 0.0

    y_hat = simulate(res.x, u, dt, fit_delay)
    resid = y_hat - y
    rms = float(np.sqrt(np.mean(resid**2)))
    denom = float(np.sqrt(np.mean((y - y.mean())**2)))
    nrms = rms / denom if denom > 0 else float("nan")

    print("=" * 68)
    print("FIT (identifiable I/O model)")
    print("=" * 68)
    print(f"  DC gain g        = {g:.5g}  degC per unit power")
    print(f"  tau_slow         = {tau_slow:8.2f} s   ({tau_slow/60:.2f} min)  "
          f"pole {-1/tau_slow:+.5g}")
    print(f"  tau_fast         = {tau_fast:8.2f} s              "
          f"pole {-1/tau_fast:+.5g}")
    if fit_delay:
        print(f"  dead time L      = {L:8.2f} s")
    print(f"  k2a = 1/g        = {1/g:.5g}  (block->ambient loss)")
    print(f"  fit RMS          = {rms:.4f} degC   (NRMS {nrms*100:.2f}% of signal)")

    feat = overshoot_features(t, y, u)
    if feat:
        y_hat_feat = overshoot_features(t, y_hat, u)
        print("\n  post-cutoff overshoot (2-lump fingerprint):")
        print(f"    heater off @ t={feat['t_off']:.0f}s, T={feat['y_off']:.2f}")
        print(f"    measured peak: +{feat['overshoot']:.2f} degC after "
              f"{feat['dt_peak']:.0f}s")
        print(f"    model    peak: +{y_hat_feat['overshoot']:.2f} degC after "
              f"{y_hat_feat['dt_peak']:.0f}s")

    # Physical parameters.
    c2 = args.c2
    if c2 is None and args.block_mass is not None:
        c2 = args.block_mass / 1000.0 * C_AL
    try:
        c1, c2, k12, k2a, anchored = physical_realization(g, tau_slow, tau_fast, c2)
    except ValueError as e:
        sys.exit(f"[error] {e}")

    print("\n" + "=" * 68)
    print("PHYSICAL PARAMETERS " + ("(anchored on block mass)" if anchored
                                    else "(CANONICAL SPLIT -- supply --block-mass "
                                         "for true C1/k12)"))
    print("=" * 68)
    if not anchored:
        print("  [warn] no anchor: absolute C1/k12 and node-1 scale are an")
        print("         assumption; the I/O model + discrete matrices below are")
        print("         still exact (all realizations share the same y).")
    print(f"  C1 (core)        = {c1:8.2f} J/K")
    print(f"  C2 (block)       = {c2:8.2f} J/K"
          + (f"  (= {c2/C_AL*1000:.0f} g Al)" if anchored else ""))
    print(f"  k12 (core<->blk) = {k12:8.4f} W/K   -> R12 = {1/k12:.4f} K/W")
    print(f"  k2a (blk->amb)   = {k2a:8.4f} W/K   -> R2a = {1/k2a:.4f} K/W")

    Ad, Bd, Cd, Dd = discretize(c1, c2, k12, k2a, args.ts)
    delay_steps = int(round(L / args.ts))

    print("\n" + "=" * 68)
    print(f"CONTROL.RS CONSTANTS  (discretized ZOH at Ts={args.ts}s)")
    print("=" * 68)
    print(f"    let model_a = [[{Ad[0,0]:.10f}, {Ad[0,1]:.10f}], "
          f"[{Ad[1,0]:.10f}, {Ad[1,1]:.10f}]];")
    print(f"    let model_b = [{Bd[0]:.10f}, {Bd[1]:.10f}];")
    print(f"    let model_c = [{Cd[0]:.1f}, {Cd[1]:.1f}];")
    print(f"    let model_d = {Dd:.1f};")
    print(f"    let dead_time = {L:.4f}; // s")
    print(f"    let delay_steps = {delay_steps}; // = dead_time / Ts")

    if args.plot:
        make_plot(t, y, y_hat, u, args.plot)
        print(f"\n[plot] wrote {args.plot}")


def make_plot(t, y, y_hat, u, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True,
                                   gridspec_kw={"height_ratios": [3, 1]})
    ax1.plot(t, y, ".", ms=3, alpha=0.5, label="measured")
    ax1.plot(t, y_hat, "-", lw=1.5, label="2-lump fit")
    ax1.set_ylabel("temp (degC, ambient-ref)")
    ax1.legend()
    ax1.grid(alpha=0.3)
    ax2.plot(t, u, drawstyle="steps-post")
    ax2.set_ylabel("power")
    ax2.set_xlabel("time (s)")
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


if __name__ == "__main__":
    main()
