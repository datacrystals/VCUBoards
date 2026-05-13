#!/usr/bin/env python3
"""
energica_screen_hunt.py — Slow sweep to find the screen-lock command

Usage:
    sudo python3 energica_screen_hunt.py /dev/ttyACM0

Watches the display and prints one ID every 5 seconds.
When the display stops flickering and locks to one screen,
note the ID and hit Ctrl+C. Tell us that ID!
"""

import sys
import serial
import time

# ---- config ----
DWELL_S = 5          # seconds per ID
START_ID = 0x000
END_ID = 0x1FF
KEEPALIVE_MS = 100

# ---- open serial ----
port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
ser = serial.Serial(port, 115200, timeout=0.05)
ser.reset_input_buffer()
time.sleep(0.3)


def send(can_id: int, data: list[int]):
    hex_data = "".join(f"{b:02X}" for b in data)
    ser.write(f"{can_id:03X}#{hex_data}\r\n".encode())
    ser.flush()


def keepalive():
    # 0x100
    send(0x100, [0x00, 0x00, 0x00, 0x00, 0xA4, 0x01, 0x64, 0x00])
    # 0x101
    send(0x101, [0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xB0, 0x04, 0x00])
    # 0x102
    send(0x102, [0x02, 0x5E, 0x01, 0x00, 0x00, 0x08, 0x4B, 0x00])


# ---- main sweep ----
print(f"Connected to {port}")
print("Starting slow sweep. WATCH THE DISPLAY!")
print(f"Each ID is hammered for {DWELL_S} seconds.")
print("Note the ID when the display locks / stops flickering.\n")

try:
    for cid in range(START_ID, END_ID + 1):
        print(f"\n{'='*40}")
        print(f"  TESTING ID 0x{cid:03X}")
        print(f"{'='*40}")

        start = time.time()
        last_keepalive = 0

        while (time.time() - start) < DWELL_S:
            now = time.time()

            # background keepalive
            if (now - last_keepalive) * 1000 >= KEEPALIVE_MS:
                keepalive()
                last_keepalive = now

            # hammer the test ID with a few payloads
            send(cid, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            time.sleep(0.05)
            send(cid, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
            time.sleep(0.05)
            send(cid, [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            time.sleep(0.05)
            send(cid, [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
            time.sleep(0.05)

            # drain RX
            while ser.in_waiting:
                ser.read(ser.in_waiting)

except KeyboardInterrupt:
    print(f"\n\nStopped at ID 0x{cid:03X}")

ser.close()
print("Done.")
