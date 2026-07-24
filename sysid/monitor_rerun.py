#!/usr/bin/env python3
"""Read binary telemetry frames from UART: live Rerun view and/or CSV capture.

Frame format (23 bytes):
    [0xAA] [time_ms:u32] [temp:f32] [setpoint:f32] [power:f32] [y_hat:f32]
           [flags:u8] [xor:u8]
    flags: bit0=heater enabled, bit1=valve open, bit2=pump on
    xor:   XOR of bytes 1..22

Usage:
    python monitor_rerun.py /dev/ttyUSB0                       # live Rerun view
    python monitor_rerun.py /dev/ttyUSB0 --csv run.csv         # view + log CSV
    python monitor_rerun.py /dev/ttyUSB0 --csv run.csv --no-viz  # headless capture
    python monitor_rerun.py /dev/ttyUSB0 --baud 9600

The CSV columns (time in seconds) are directly consumable by sysid_fit.py:
    time,temp,setpoint,power,y_hat,enabled,valve,pump

Requirements:
    pip install pyserial            # always
    pip install rerun-sdk           # only for the live view (omit with --no-viz)
"""

import argparse
import csv
import struct
import sys

import serial

SYNC = 0xAA
# time_ms(u32), temp(f32), setpoint(f32), power(f32), y_hat(f32), flags(u8)
PAYLOAD_FMT = "<IffffB"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)  # 21 bytes
FRAME_LEN = 1 + PAYLOAD_SIZE + 1  # sync + payload + checksum = 23

CSV_COLUMNS = ["time", "temp", "setpoint", "power", "y_hat",
               "enabled", "valve", "pump"]


def parse_frame(buf: bytes):
    """Validate checksum and unpack a 23-byte frame. Returns None on error."""
    if len(buf) != FRAME_LEN or buf[0] != SYNC:
        return None
    # XOR checksum over payload bytes 1..22 (everything but sync and checksum).
    chk = 0
    for b in buf[1:FRAME_LEN - 1]:
        chk ^= b
    if chk != buf[FRAME_LEN - 1]:
        return None
    time_ms, temp, setpoint, power, y_hat, flags = struct.unpack_from(
        PAYLOAD_FMT, buf, 1)
    return {
        "time_ms": time_ms,
        "temp": temp,
        "setpoint": setpoint,
        "power": power,
        "y_hat": y_hat,
        "enabled": bool(flags & 0x01),
        "valve": bool(flags & 0x02),
        "pump": bool(flags & 0x04),
    }


def main():
    parser = argparse.ArgumentParser(description="Pasto-rs telemetry monitor")
    parser.add_argument("port", help="Serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default 115200)")
    parser.add_argument("--csv", metavar="PATH", default=None,
                        help="also log frames to this CSV file")
    parser.add_argument("--no-viz", action="store_true",
                        help="disable the live Rerun view (headless capture)")
    args = parser.parse_args()

    viz = not args.no_viz
    if not viz and args.csv is None:
        sys.exit("nothing to do: --no-viz given without --csv "
                 "(enable one output)")

    # Lazy Rerun import so --no-viz capture doesn't require rerun-sdk.
    rr = None
    if viz:
        try:
            import rerun as rr
        except ImportError:
            sys.exit("rerun-sdk not installed; install it or pass --no-viz "
                     "for CSV-only capture")
        rr.init("pasto-rs-monitor", spawn=True)

    csv_file = None
    csv_writer = None
    if args.csv:
        # Line-buffered so each row hits disk -> a long run survives Ctrl-C.
        csv_file = open(args.csv, "w", newline="", buffering=1)
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(CSV_COLUMNS)

    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    print(f"Listening on {args.port} @ {args.baud} baud "
          f"(viz={'on' if viz else 'off'}, csv={args.csv or 'off'}) …")

    frame_count = 0
    err_count = 0

    try:
        while True:
            # Synchronise: scan for the sync byte.
            b = ser.read(1)
            if len(b) == 0:
                continue
            if b[0] != SYNC:
                continue

            rest = ser.read(FRAME_LEN - 1)
            if len(rest) != FRAME_LEN - 1:
                err_count += 1
                continue

            f = parse_frame(bytes([SYNC]) + rest)
            if f is None:
                err_count += 1
                continue

            frame_count += 1
            t = f["time_ms"] / 1000.0  # seconds

            if rr is not None:
                rr.set_time("time", duration=t)
                rr.log("temperature", rr.Scalars(f["temp"]))
                rr.log("setpoint", rr.Scalars(f["setpoint"]))
                rr.log("heater/power", rr.Scalars(f["power"]))
                rr.log("model/y_hat", rr.Scalars(f["y_hat"]))
                rr.log("heater/enabled", rr.Scalars(float(f["enabled"])))
                rr.log("valve/open", rr.Scalars(float(f["valve"])))
                rr.log("pump/on", rr.Scalars(float(f["pump"])))

            if csv_writer is not None:
                csv_writer.writerow([
                    f"{t:.3f}", f["temp"], f["setpoint"], f["power"],
                    f["y_hat"], int(f["enabled"]), int(f["valve"]),
                    int(f["pump"]),
                ])

            if frame_count % 100 == 0:
                print(f"  {frame_count} frames, {err_count} errors, "
                      f"t={t:.1f}s, T={f['temp']:.1f}°C")

    except KeyboardInterrupt:
        print(f"\nStopped. {frame_count} frames, {err_count} errors.")
    finally:
        ser.close()
        if csv_file is not None:
            csv_file.close()


if __name__ == "__main__":
    main()
