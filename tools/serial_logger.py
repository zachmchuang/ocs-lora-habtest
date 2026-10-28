#!/usr/bin/env -S uv run
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "pyserial>=3.5",
# ]
# ///
"""Capture ground-node USB serial: echo pretty text, save CSV rows to disk.

Usage:
  uv run tools/serial_logger.py [/dev/ttyUSB0] [logs/dir]
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Install deps with uv:  uv run tools/serial_logger.py", file=sys.stderr)
    sys.exit(1)


CSV_PREFIX = "CSV,"


def pick_port(preferred: str | None) -> str:
    if preferred:
        return preferred
    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("No serial ports found. Pass the device path explicitly.")
    # Prefer USB CDC / ACM / Heltec-looking descriptions
    for p in ports:
        desc = (p.description or "").lower()
        if "usb" in desc or "uart" in desc or "serial" in desc or "acm" in p.device.lower():
            return p.device
    return ports[0].device


def main() -> int:
    ap = argparse.ArgumentParser(description="HAB ground serial logger")
    ap.add_argument("port", nargs="?", help="Serial device (default: auto)")
    ap.add_argument(
        "log_dir",
        nargs="?",
        default="logs",
        help="Directory for CSV logs (default: logs)",
    )
    ap.add_argument("-b", "--baud", type=int, default=115200)
    args = ap.parse_args()

    port = pick_port(args.port)
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    csv_path = log_dir / f"ground_{stamp}.csv"

    print(f"Opening {port} @ {args.baud}")
    print(f"Writing CSV to {csv_path}")
    print("Ctrl+C to stop\n")

    header = (
        "rx_utc,rssi_dbm,snr_db,type,seq,lat,lon,alt_m,tilt_deg,"
        "heading_deg,gps_ok,imu_ok,mag_ok,rtt_ms\n"
    )

    with serial.Serial(port, args.baud, timeout=1) as ser, csv_path.open(
        "w", encoding="utf-8", newline=""
    ) as csv_f:
        csv_f.write(header)
        csv_f.flush()
        while True:
            try:
                raw = ser.readline()
            except KeyboardInterrupt:
                print("\nStopped.")
                break
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            except Exception:
                continue

            # Host wall-clock for data rows (firmware uses t+millis)
            if line.startswith(CSV_PREFIX):
                body = line[len(CSV_PREFIX) :]
                # Skip firmware header echo
                if body.startswith("rx_utc,"):
                    print(line)
                    continue
                now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
                parts = body.split(",", 1)
                row = f"{now},{parts[1]}" if len(parts) == 2 else f"{now},{body}"
                csv_f.write(row + "\n")
                csv_f.flush()
                print(f"CSV,{row}")
            else:
                print(line)

    print(f"Saved {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
