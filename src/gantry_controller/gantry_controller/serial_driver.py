from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Callable, Optional

import serial


@dataclass
class GantryState:
    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0
    busy: bool = False
    last_error: str = ""


class SerialGantryDriver:
    """Threaded line-based serial driver for the gantry MCU."""

    def __init__(
        self,
        port: str,
        baud: int,
        line_cb: Optional[Callable[[str], None]] = None,
        read_timeout_s: float = 0.1,
    ) -> None:
        self._port = port
        self._baud = baud
        self._read_timeout_s = read_timeout_s
        self._line_cb = line_cb

        self._ser: Optional[serial.Serial] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._state_lock = threading.Lock()
        self._last_ok_time = 0.0
        self.state = GantryState()

    def open(self) -> None:
        self._ser = serial.Serial(self._port, self._baud, timeout=self._read_timeout_s)
        time.sleep(0.25)
        self._stop_evt.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        self._stop_evt.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
        if self._ser is not None and self._ser.is_open:
            self._ser.close()

    def move_linear(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        feed_mm_s: float,
    ) -> None:
        self.send_line(f"G0 X{x_mm:.3f} Y{y_mm:.3f} Z{z_mm:.3f} F{feed_mm_s:.3f}")

    def home(self) -> None:
        self.send_line("HOME")

    def stop(self) -> None:
        self.send_line("STOP")

    def ping(self) -> None:
        self.send_line("PING")

    def send_line(self, line: str) -> None:
        if self._ser is None or not self._ser.is_open:
            raise RuntimeError("Serial port is not open")
        if not line.endswith("\n"):
            line += "\n"
        with self._state_lock:
            self.state.last_error = ""
        self._ser.write(line.encode("utf-8"))

    def get_state_copy(self) -> GantryState:
        with self._state_lock:
            return GantryState(
                x_mm=self.state.x_mm,
                y_mm=self.state.y_mm,
                z_mm=self.state.z_mm,
                busy=self.state.busy,
                last_error=self.state.last_error,
            )

    def last_ok_age_s(self) -> float:
        if self._last_ok_time <= 0.0:
            return float("inf")
        return time.time() - self._last_ok_time

    def _rx_loop(self) -> None:
        assert self._ser is not None
        buffer = ""
        while not self._stop_evt.is_set():
            try:
                chunk = self._ser.read(256)
            except Exception:
                time.sleep(0.05)
                continue

            if not chunk:
                continue

            buffer += chunk.decode("utf-8", errors="ignore")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                self._handle_line(line)
                if self._line_cb is not None:
                    self._line_cb(line)

    def _handle_line(self, line: str) -> None:
        with self._state_lock:
            if line == "OK":
                self._last_ok_time = time.time()
                self.state.last_error = ""
                return

            if line.startswith("BUSY"):
                parts = line.split()
                if len(parts) == 2:
                    self.state.busy = parts[1] == "1"
                return

            if line.startswith("POS"):
                for token in line.split()[1:]:
                    try:
                        if token.startswith("X"):
                            self.state.x_mm = float(token[1:])
                        elif token.startswith("Y"):
                            self.state.y_mm = float(token[1:])
                        elif token.startswith("Z"):
                            self.state.z_mm = float(token[1:])
                    except ValueError:
                        continue
                return

            if line.startswith("ERR"):
                self.state.last_error = line[4:] if len(line) > 4 else "unknown"
