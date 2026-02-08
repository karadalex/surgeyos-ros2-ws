import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable

import serial


@dataclass
class GantryState:
    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0
    busy: bool = False
    last_error: str = ""


class SerialGantryDriver:
    """
    Threaded serial driver for a NodeMCU running a simple text protocol.
    """

    def __init__(
        self,
        port: str,
        baud: int,
        line_cb: Optional[Callable[[str], None]] = None,
        read_timeout_s: float = 0.1,
    ):
        self._port = port
        self._baud = baud
        self._read_timeout_s = read_timeout_s
        self._line_cb = line_cb

        self._ser: Optional[serial.Serial] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

        self.state = GantryState()
        self._state_lock = threading.Lock()

        # Used by action server to wait for acceptance or completion
        self._last_ok_time = 0.0

    def open(self) -> None:
        self._ser = serial.Serial(self._port, self._baud, timeout=self._read_timeout_s)
        time.sleep(0.2)  # allow MCU reset
        self._stop_evt.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        self._stop_evt.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        if self._ser and self._ser.is_open:
            self._ser.close()

    def send_line(self, line: str) -> None:
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Serial port not open")
        if not line.endswith("\n"):
            line += "\n"
        self._ser.write(line.encode("utf-8"))

    def move_linear(self, x_mm: float, y_mm: float, z_mm: float, feed_mm_s: float) -> None:
        cmd = f"G0 X{x_mm:.3f} Y{y_mm:.3f} Z{z_mm:.3f} F{feed_mm_s:.3f}"
        self.send_line(cmd)

    def home(self) -> None:
        self.send_line("HOME")

    def stop(self) -> None:
        self.send_line("STOP")

    def ping(self) -> None:
        self.send_line("PING")

    def get_state_copy(self) -> GantryState:
        with self._state_lock:
            return GantryState(
                x_mm=self.state.x_mm,
                y_mm=self.state.y_mm,
                z_mm=self.state.z_mm,
                busy=self.state.busy,
                last_error=self.state.last_error,
            )

    def _rx_loop(self) -> None:
        assert self._ser is not None
        buf = ""
        while not self._stop_evt.is_set():
            try:
                chunk = self._ser.read(256)
            except Exception:
                time.sleep(0.05)
                continue

            if not chunk:
                continue

            buf += chunk.decode("utf-8", errors="ignore")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if line:
                    self._handle_line(line)
                    if self._line_cb:
                        self._line_cb(line)

    def _handle_line(self, line: str) -> None:
        # Examples:
        # OK
        # BUSY 1
        # POS X10.000 Y20.000 Z5.000
        # ERR limit_hit
        with self._state_lock:
            if line == "OK":
                self._last_ok_time = time.time()
                return

            if line.startswith("BUSY"):
                parts = line.split()
                if len(parts) == 2:
                    self.state.busy = (parts[1] == "1")
                return

            if line.startswith("POS"):
                # POS X.. Y.. Z..
                parts = line.split()
                for p in parts[1:]:
                    if p.startswith("X"):
                        self.state.x_mm = float(p[1:])
                    elif p.startswith("Y"):
                        self.state.y_mm = float(p[1:])
                    elif p.startswith("Z"):
                        self.state.z_mm = float(p[1:])
                return

            if line.startswith("ERR"):
                self.state.last_error = line[4:] if len(line) > 4 else "unknown"
                return

    def last_ok_age_s(self) -> float:
        t = self._last_ok_time
        return float("inf") if t <= 0 else (time.time() - t)
