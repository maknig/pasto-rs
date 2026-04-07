# pasto-rs

Rust/Embassy firmware for a Quickmill espresso machine using the [re-leva](https://github.com/maknig/re-leva) controller.

> **This project is under active development.** APIs, control parameters, and hardware mappings may change without notice.

## Overview

pasto-rs is a bare-metal, async Rust firmware targeting the **STM32L431CC** microcontroller on the [re-leva controller board](https://maknig.github.io/re-leva/). It replaces the stock electronics of a Quickmill espresso machine with PID-based temperature control, zero-crossing heater switching, and a physical power switch with LED status feedback.

The firmware is built on the [Embassy](https://embassy.dev/) async embedded framework and runs entirely without an OS or allocator.

## Features

- **PID temperature control** targeting 93 &deg;C (espresso brewing temperature)
- **Zero-crossing SSR/TRIAC control** using integral cycle (burst-firing) for clean AC heater switching
- **Analog temperature sensing** via ADC with 1D Kalman filtering for noise rejection
- **Power button** with debounce and multi-state **status LED** (off / heating / at temperature / error)
- **Fully async architecture** with 5 Embassy tasks communicating over typed channels

## Hardware

| Component | Details |
|-----------|---------|
| **MCU** | STM32L431CC (ARM Cortex-M4, 256 KB flash, 64 KB SRAM) |
| **Controller** | [re-leva](https://github.com/maknig/re-leva) custom board ([docs](https://maknig.github.io/re-leva/)) |
| **Heater** | AC heater driven via solid-state relay with zero-cross detection |
| **Temperature sensor** | Analog probe with op-amp signal conditioning (0--150 &deg;C range) |

### Pin assignments

| Pin | Function | Direction | Description |
|-----|----------|-----------|-------------|
| PA0 | Zero-crossing detector | Input (EXTI, falling edge) | AC mains zero-cross signal |
| PA3 | Power switch | Input (EXTI, falling edge) | Momentary push button |
| PA4 | Status LED | Output | Visual feedback (off / on / heating / error) |
| PA5 | Heater SSR gate | Output | Drives the solid-state relay |
| PA6 | Temperature probe | Analog input (ADC1) | Analog temperature sensor |

## Architecture

The firmware runs 5 concurrent async tasks on the Embassy executor, communicating via typed async channels:

```
                        CONTROL_CH                          HEATER_CMD_CH
 [PA6 ADC] --> temp_task -----------> control_task -------> heater_task --> [PA5 SSR]
                                        ^       |                ^
                                        |       |                |
                                   SWITCH_CH    |             ZC_CH
                                        |       v                |
               [PA3 Button] --> switch_task --> [PA4 LED]    zc_task <-- [PA0 ZC]
```

| Task | Role |
|------|------|
| `zc_task` | Detects AC zero-crossing edges, signals `heater_task` |
| `temp_task` | Reads temperature at 10 Hz, applies Kalman filter, feeds `control_task` |
| `control_task` | Runs the PID loop, sends power commands to heater and state updates to switch |
| `heater_task` | Accumulator-based burst-firing: decides each half-wave whether to fire the SSR |
| `switch_task` | Handles button presses (with 20 ms debounce), manages LED state |

### LED states

| State | Meaning |
|-------|---------|
| Off | System disabled |
| Steady on | At setpoint (within 2 &deg;C, power < 5%) |
| On (heating) | Actively heating toward setpoint |
| Error | Temperature out of range (< 0 &deg;C or > 130 &deg;C) |

## Prerequisites

- **Rust 1.84+** (stable) with the `thumbv7em-none-eabi` target
- **probe-rs** for flashing and debugging
- An **SWD debug probe** (ST-Link or compatible)

The repository includes a Nix flake for a reproducible development environment:

```sh
nix develop   # or use direnv with the included .envrc
```

This provides `probe-rs`, the Rust toolchain with the ARM target, and other build tools.

## Building and flashing

Build the firmware:

```sh
cargo build
```

Build and flash to the target (via probe-rs):

```sh
cargo run
```

`cargo run` will flash the binary and stream `defmt` log output over RTT. Debug logging is enabled by default (the `debug` feature). To build without it:

```sh
cargo build --no-default-features
```

## Configuration

Key parameters and where to find them in the source:

| Parameter | Location | Default |
|-----------|----------|---------|
| Brew setpoint | `src/control.rs` | 93.0 &deg;C |
| PID gains (Kp, Ki, Kd) | `src/control.rs` | 0.05, 0.01, 0.0 |
| PID sample time (dt) | `src/control.rs` | 0.05 s |
| Kalman process noise (q) | `src/temp_probe.rs` | 0.02 |
| Kalman measurement noise (r) | `src/temp_probe.rs` | 0.5 |
| Temp conversion (linear) | `src/temp_probe.rs` | raw * (150 / 4096) - 0.3 |
| Temp read interval | `src/main.rs` | 100 ms |

## Status

This firmware is **under active development**. It is intended for experimentation and personal use. It is not certified for commercial or safety-critical applications.

**Safety:** This project involves AC mains voltage. Improper wiring or control of mains-connected heaters can cause fire, electric shock, or death. If you build or modify this system, ensure all mains-voltage wiring is performed by a qualified person and that appropriate safety measures (fusing, grounding, enclosure) are in place.
