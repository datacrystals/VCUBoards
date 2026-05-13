#!/usr/bin/env python3
"""
energica_logger.py — Logging CAN bridge for Energica display reverse engineering

Usage:
    sudo python3 energica_logger.py /dev/ttyACM0

All TX, RX, and timestamps are logged to:
    logs/energica_YYYY-MM-DD_HH-MM-SS.log

Interactive commands (same as bridge.py plus logging):
    >>> send(0x102, [0x02, 0x5E, 0x01, 0x00, 0x00, 0x08, 0x4B, 0x00])
    >>> repeat(0x002, 100, [0x00]*8)
    >>> sweep(0x000, 0x100, dwell=5.0)
    >>> stop_all()
    >>> log("Display locked to speedometer at ID 0x002")
"""

import sys
import os
import serial
import threading
import time
import queue
import datetime


class EnergicaLogger:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ser.reset_input_buffer()
        time.sleep(0.3)

        # Create log file
        os.makedirs("logs", exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_path = f"logs/energica_{ts}.log"
        self.log_file = open(self.log_path, "w")
        self._log_raw(f"# Energica CAN Log started {ts}\n")
        self._log_raw(f"# Port: {port}  Baud: {baud}\n")
        self._log_raw("# Format: [TIME] TX/RX ID#DATA  (or user notes)\n\n")

        self.rx_queue = queue.Queue()
        self._running = True
        self._repeaters = {}
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _log_raw(self, text: str):
        self.log_file.write(text)
        self.log_file.flush()

    def _log(self, direction: str, can_id: int, data: list[int]):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hex_data = "".join(f"{b:02X}" for b in data)
        line = f"[{ts}] {direction} {can_id:03X}#{hex_data}\n"
        self._log_raw(line)
        print(line, end="")

    def _read_loop(self):
        line = b""
        while self._running:
            try:
                data = self.ser.read(256)
            except serial.SerialException as e:
                print(f"[SERIAL ERR] {e}")
                time.sleep(0.5)
                continue
            if data:
                line += data
                while b"\n" in line:
                    chunk, line = line.split(b"\n", 1)
                    chunk = chunk.decode("ascii", errors="ignore").strip()
                    if chunk and not chunk.startswith("#"):
                        self.rx_queue.put(chunk)
                        # Log RX
                        try:
                            cid, dbytes = self._parse_quick(chunk)
                            self._log("RX", cid, dbytes)
                        except Exception:
                            pass

    def _parse_quick(self, msg: str):
        id_hex, data_hex = msg.split("#", 1)
        return int(id_hex, 16), [int(data_hex[i:i+2], 16) for i in range(0, len(data_hex), 2)]

    def send(self, can_id: int, data: list[int]):
        hex_data = "".join(f"{b:02X}" for b in data)
        self.ser.write(f"{can_id:03X}#{hex_data}\r\n".encode())
        self.ser.flush()
        self._log("TX", can_id, data)

    def repeat(self, can_id: int, period_ms: int, data: list[int]):
        self.stop(can_id)
        ev = threading.Event()
        def loop():
            while not ev.is_set():
                self.send(can_id, data)
                time.sleep(period_ms / 1000.0)
        t = threading.Thread(target=loop, daemon=True)
        t.start()
        self._repeaters[can_id] = (t, ev)
        print(f"[REP] 0x{can_id:03X} every {period_ms} ms")

    def stop(self, can_id: int):
        if can_id in self._repeaters:
            self._repeaters[can_id][1].set()
            del self._repeaters[can_id]

    def stop_all(self):
        for cid in list(self._repeaters.keys()):
            self.stop(cid)

    def sweep(self, start_id: int, end_id: int, dwell: float = 5.0,
              payloads=None, keepalive_ids=None):
        """Slow sweep logging everything. Hit Ctrl+C to stop early."""
        if payloads is None:
            payloads = [
                [0x00]*8,
                [0xFF]*8,
                [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
            ]
        try:
            for cid in range(start_id, end_id + 1):
                banner = f"\n{'='*40}\n  DWELL ID 0x{cid:03X}\n{'='*40}\n"
                self._log_raw(banner)
                print(banner, end="")

                t0 = time.time()
                while (time.time() - t0) < dwell:
                    for p in payloads:
                        self.send(cid, p)
                        time.sleep(0.05)
                    if keepalive_ids:
                        for k_id, k_data in keepalive_ids.items():
                            self.send(k_id, k_data)
                            time.sleep(0.02)
                    # drain any RX that piled up
                    time.sleep(0.1)
        except KeyboardInterrupt:
            print(f"\n[Sweep stopped at 0x{cid:03X}]")
            self._log_raw(f"# Sweep stopped at 0x{cid:03X}\n")

    def log(self, note: str):
        """Log a human observation."""
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] NOTE: {note}\n"
        self._log_raw(line)
        print(line, end="")

    def close(self):
        self._running = False
        self.stop_all()
        self.log_file.close()
        self.ser.close()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    b = EnergicaLogger(port)
    print(f"Connected to {port}")
    print(f"Logging to: {b.log_path}")
    print("Commands:  send(id, data), repeat(id, ms, data), stop(id), stop_all()")
    print("           sweep(start, end, dwell=5.0)")
    print("           log(\"your note here\")")

    # Inject into local namespace for interactive use
    send = b.send
    repeat = b.repeat
    stop = b.stop
    stop_all = b.stop_all
    sweep = b.sweep
    log = b.log

    try:
        from IPython import embed
        embed(using=False)
    except ImportError:
        import code
        code.interact(local=locals())

    b.close()


if __name__ == "__main__":
    main()
