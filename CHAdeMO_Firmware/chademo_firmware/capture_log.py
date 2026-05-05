#!/usr/bin/env python3
"""
Serial log capture for CHAdeMO firmware debugging.

Usage:
    python3 capture_log.py [PORT] [BAUD]

Defaults:
    PORT = /dev/ttyACM0
    BAUD = 115200

Output:
    Writes to chademo_log_YYYYmmdd_HHMMSS.txt with timestamps.
    Also prints to stdout in real time.

Requirements:
    pip install pyserial
"""

import sys
import os
import time
from datetime import datetime
from serial import Serial, SerialException

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200


def get_log_filename():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"chademo_log_{ts}.txt"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    logfile = get_log_filename()
    print(f"[CAPTURE] Logging to: {os.path.abspath(logfile)}")
    print(f"[CAPTURE] Port: {port} @ {baud} baud")
    print("[CAPTURE] Press Ctrl+C to stop\n")

    with open(logfile, "w", buffering=1) as f:
        f.write(f"# CHAdeMO serial log started {datetime.now().isoformat()}\n")
        f.write(f"# Port: {port} @ {baud} baud\n")
        f.write("# ---\n")

        while True:
            try:
                with Serial(port, baud, timeout=0.1) as ser:
                    print(f"[CAPTURE] Connected to {port}")
                    while True:
                        try:
                            line = ser.readline()
                            if not line:
                                continue
                            # Decode with fallback for garbage bytes
                            text = line.decode("utf-8", errors="replace").rstrip("\r\n")
                            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            out = f"[{ts}] {text}"
                            print(out)
                            f.write(out + "\n")
                        except SerialException:
                            print(f"[CAPTURE] Lost connection to {port}")
                            break
            except SerialException as e:
                print(f"[CAPTURE] Cannot open {port}: {e}")
                print(f"[CAPTURE] Retrying in 3 seconds...")
                time.sleep(3)
            except KeyboardInterrupt:
                print("\n[CAPTURE] Stopped.")
                break


if __name__ == "__main__":
    main()
