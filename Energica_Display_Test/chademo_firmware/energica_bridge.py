#!/usr/bin/env python3
"""
energica_bridge.py — Python side of the USB-CAN bridge

Usage:
    python3 energica_bridge.py /dev/ttyACM0

Then in the Python REPL you have:
    >>> send(0x102, [0x02, 0x5E, 0x01, 0x00, 0x00, 0x08, 0x4B, 0x00])
    >>> send(0x100, [0x00, 0x00, 0x00, 0x00, 0xA4, 0x01, 0x64, 0x00])
    >>> repeat(0x102, 100, [0x02, 0x5E, 0x01, 0x00, 0x00, 0x08, 0x4B, 0x00])
    >>> stop_all()
    >>> for msg in read_all(timeout=2.0):
    ...     print(msg)

Or just run the script and type interactively:
    >>> lock_screen(0x82)   # experiment with 0x400 echo
"""

import sys
import serial
import threading
import time
import queue


class EnergicaBridge:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ser.reset_input_buffer()
        self.rx_queue = queue.Queue()
        self._running = True
        self._repeaters = {}   # id -> (thread, event)
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        time.sleep(0.5)  # let bridge boot message pass
        self.ser.reset_input_buffer()

    def _read_loop(self):
        line = b""
        while self._running:
            data = self.ser.read(256)
            if data:
                line += data
                while b"\n" in line:
                    chunk, line = line.split(b"\n", 1)
                    chunk = chunk.decode("ascii", errors="ignore").strip()
                    if chunk and not chunk.startswith("#"):
                        self.rx_queue.put(chunk)

    def send(self, can_id: int, data: list[int]):
        """Send a single CAN frame."""
        hex_data = "".join(f"{b:02X}" for b in data)
        line = f"{can_id:03X}#{hex_data}\r\n"
        self.ser.write(line.encode())
        self.ser.flush()

    def repeat(self, can_id: int, period_ms: int, data: list[int]):
        """Start a background thread that sends a frame every period_ms."""
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
        """Stop a repeating frame."""
        if can_id in self._repeaters:
            self._repeaters[can_id][1].set()
            del self._repeaters[can_id]

    def stop_all(self):
        """Stop all repeating frames."""
        for can_id in list(self._repeaters.keys()):
            self.stop(can_id)

    def read_all(self, timeout: float = 0.0):
        """Drain the RX queue. Returns list of '123#010203...' strings."""
        msgs = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                msgs.append(self.rx_queue.get(timeout=0.05))
            except queue.Empty:
                break
        return msgs

    def parse(self, msg: str):
        """Parse '123#01020304' → (0x123, [0x01, 0x02, 0x03, 0x04])."""
        id_hex, data_hex = msg.split("#", 1)
        can_id = int(id_hex, 16)
        data = [int(data_hex[i:i+2], 16) for i in range(0, len(data_hex), 2)]
        return can_id, data

    def close(self):
        self._running = False
        self.stop_all()
        self.ser.close()


def keepalive_data(voltage: int = 350, current: int = 0, soc: int = 75,
                   status: int = 0x08, max_v: int = 420, cap: int = 100):
    """Build the three CHAdeMO-style keep-alive frames."""
    h100 = [0x00, 0x00, 0x00, 0x00, max_v & 0xFF, (max_v >> 8) & 0xFF, cap, 0x00]
    h101 = [0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xB0, 0x04, 0x00]
    h102 = [0x02, voltage & 0xFF, (voltage >> 8) & 0xFF, current, 0x00, status, soc, 0x00]
    return {0x100: h100, 0x101: h101, 0x102: h102}


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    b = EnergicaBridge(port)
    print(f"Connected to {port}")
    print("Available:  b.send(id, data), b.repeat(id, ms, data), b.stop(id)")
    print("            b.read_all(timeout), b.parse(msg), keepalive_data(...)")
    print("            b.stop_all(), lock_screen(byte5)")

    # convenience functions injected into local namespace
    send = b.send
    repeat = b.repeat
    stop = b.stop
    stop_all = b.stop_all
    read_all = b.read_all
    parse = b.parse

    def lock_screen(byte5: int):
        """Echo 0x400 back with a fixed byte 5 to try locking the screen."""
        repeat(0x400, 55, [0x00, 0x6F, 0x0B, 0xCE, 0x00, byte5, 0x00, 0x80])

    def start_keepalive(v=350, a=0, soc=75, status=0x08, max_v=420, cap=100):
        frames = keepalive_data(v, a, soc, status, max_v, cap)
        for cid, data in frames.items():
            repeat(cid, 100, data)

    # drop into interactive mode if called directly
    try:
        from IPython import embed
        embed(using=False)
    except ImportError:
        import code
        code.interact(local=locals())

    b.close()


if __name__ == "__main__":
    main()
