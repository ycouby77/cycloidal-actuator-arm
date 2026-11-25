"""Host-side client for the actuator firmware's line protocol (docs/protocol.md).

Layers, bottom up:
    Transport   moves text lines (serial today, CAN later, or the simulator)
    Actuator    commands, replies, telemetry, heartbeat, CSV recording
    GUI         gui.py, only ever talks to Actuator

Swapping serial for CAN later means writing one new Transport. Nothing above it changes.
"""

from __future__ import annotations

import csv
import threading
import time
import queue
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

MODES = ("torque", "vel", "pos")          # index = mode number in telemetry
MODE_UNITS = {"torque": "V", "vel": "deg/s", "pos": "deg"}

HEARTBEAT_S = 0.25        # ping period. Firmware watchdog is WATCHDOG_MS
WATCHDOG_MS = 1000        # firmware disables the motor if the host is quiet this long


# =====================================================================
# Transports
# =====================================================================

class Transport(ABC):
    """Anything that can carry newline-terminated text lines both ways."""

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def write_line(self, line: str) -> None:
        """Send one line. Must be safe to call from several threads."""

    @abstractmethod
    def read_line(self, timeout: float) -> Optional[str]:
        """Return the next complete line without its newline, or None on timeout."""


class SerialTransport(Transport):
    """USB serial through the Nucleo's ST-LINK virtual COM port."""

    def __init__(self, port: str, baud: int = 460800):
        self.port = port
        self.baud = baud
        self._ser = None
        self._buf = bytearray()
        self._write_lock = threading.Lock()

    def open(self) -> None:
        import serial  # imported here so the simulator works without pyserial
        # The ST-LINK VCP does not reset the MCU on open, so firmware state survives reconnects
        self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
        self._ser.reset_input_buffer()

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def write_line(self, line: str) -> None:
        with self._write_lock:   # whole lines only, never interleaved bytes
            self._ser.write((line + "\n").encode("ascii"))

    def read_line(self, timeout: float) -> Optional[str]:
        deadline = time.monotonic() + timeout
        while True:
            nl = self._buf.find(b"\n")
            if nl >= 0:
                raw = bytes(self._buf[:nl])
                del self._buf[:nl + 1]
                return raw.decode("ascii", errors="replace").rstrip("\r")
            if time.monotonic() > deadline:
                return None
            chunk = self._ser.read(max(1, self._ser.in_waiting))   # waits up to 50 ms
            if chunk:
                self._buf += chunk


def list_serial_ports() -> list[tuple[str, str]]:
    """(device, description) for every serial port, ST-LINK ports first."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    ports = [(p.device, p.description or "") for p in list_ports.comports()]
    ports.sort(key=lambda p: ("stlink" not in p[1].lower().replace("-", ""), p[0]))
    return ports


# =====================================================================
# Data types
# =====================================================================

@dataclass
class Sample:
    """One telemetry line. Output-side units."""
    t: float          # s, device clock, unwrapped
    mode: int         # 0 torque, 1 vel, 2 pos
    enabled: bool
    target: float     # V, deg/s or deg depending on mode
    pos: float        # deg
    vel: float        # deg/s
    uq: float         # V
    host_t: float     # s, host clock when received (time.time())


@dataclass
class Reply:
    ok: bool
    cmd: str
    text: str                                   # whole line
    fields: dict = field(default_factory=dict)  # key=value pairs, numbers converted

    def __str__(self) -> str:
        return self.text


class ActuatorError(Exception):
    pass


def parse_reply(line: str) -> Optional[Reply]:
    """'ok status init=1 pos=3.2' -> Reply. Returns None for non-reply lines."""
    parts = line.split()
    if len(parts) < 2 or parts[0] not in ("ok", "err"):
        return None
    fields = {}
    for tok in parts[2:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            try:
                fields[k] = float(v)
            except ValueError:
                fields[k] = v
    return Reply(ok=parts[0] == "ok", cmd=parts[1], text=line, fields=fields)


# =====================================================================
# Actuator client
# =====================================================================

class Actuator:
    """One actuator over one transport.

    Threads:
      reader     routes every incoming line: replies, telemetry, log text
      heartbeat  pings so the firmware watchdog stays happy while we're connected

    on_sample(Sample) and on_log(str) are called from the reader thread.
    Keep them short (the GUI just drops things into a queue).
    """

    def __init__(self, transport: Transport,
                 on_sample: Optional[Callable[[Sample], None]] = None,
                 on_log: Optional[Callable[[str], None]] = None):
        self.transport = transport
        self.on_sample = on_sample or (lambda s: None)
        self.on_log = on_log or (lambda s: None)

        self._replies: "queue.Queue[Reply]" = queue.Queue()
        self._cmd_lock = threading.Lock()      # one command in flight at a time
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._last_cmd_time = 0.0

        self.connected = False
        self.link_ok = False

        # device clock unwrap (micros() wraps every ~71 min)
        self._t_last_raw = None
        self._t_offset = 0

        # recording
        self._rec_lock = threading.Lock()
        self._rec_file = None
        self._rec_writer = None
        self.rec_path: Optional[Path] = None
        self.rec_rows = 0

    # ---------- connection ----------

    def connect(self, stream_hz: int = 100) -> Reply:
        """Open the transport, find the firmware, arm the watchdog, start streaming."""
        self.transport.open()
        self._stop.clear()
        self._start_thread(self._reader_loop, "reader")

        # The board may be mid-boot or printing; give it a few tries
        last_err = None
        for _ in range(8):
            try:
                self.command("ping", timeout=0.3)
                break
            except ActuatorError as e:
                last_err = e
        else:
            self._shutdown_threads()
            self.transport.close()
            raise ActuatorError(f"no reply to ping ({last_err}). Right port? Firmware flashed? Baud 460800?")

        self.connected = True
        self.link_ok = True
        self.command(f"wd {WATCHDOG_MS}")
        self.command(f"stream {int(stream_hz)}")
        self._start_thread(self._heartbeat_loop, "heartbeat")
        return self.command("status")

    def close(self) -> None:
        """Stop the motor, stop streaming, disarm the watchdog, close the port."""
        if self.connected:
            for cmd in ("disable", "stream 0", "wd 0"):
                try:
                    self.command(cmd, timeout=0.3)
                except ActuatorError:
                    pass
        self.stop_recording()
        self.connected = False
        self._shutdown_threads()
        self.transport.close()

    # ---------- commands ----------

    def command(self, line: str, timeout: float = 1.0) -> Reply:
        """Send one command and wait for ITS reply. Raises ActuatorError on err or timeout."""
        name = line.split()[0].lower() if line.split() else ""
        expect = "disable" if name == "stop" else name
        with self._cmd_lock:
            self._drain_replies()
            self.transport.write_line(line)
            self._last_cmd_time = time.monotonic()
            deadline = time.monotonic() + timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ActuatorError(f"timeout waiting for reply to '{line}'")
                try:
                    r = self._replies.get(timeout=remaining)
                except queue.Empty:
                    continue
                # A reply to someone else's line (e.g. an emergency stop) is not ours
                if r.cmd != expect and not (r.cmd == "line" and not r.ok):
                    continue
                if not r.ok:
                    raise ActuatorError(r.text)
                return r

    def emergency_stop(self) -> None:
        """Write 'disable' immediately, without waiting behind a command in flight."""
        try:
            self.transport.write_line("disable")
        except Exception as e:  # port gone: nothing else we can do from here
            self.on_log(f"! stop write failed: {e}")

    def init(self) -> Reply:
        return self.command("init", timeout=10.0)   # initFOC takes a few seconds

    def enable(self) -> Reply:
        return self.command("enable")

    def disable(self) -> Reply:
        return self.command("disable")

    def set_mode(self, mode: str) -> Reply:
        return self.command(f"mode {mode}")

    def set_target(self, value: float) -> Reply:
        return self.command(f"set {value:.4f}")

    def zero(self) -> Reply:
        return self.command("zero")

    def set_stream(self, hz: int) -> Reply:
        return self.command(f"stream {int(hz)}")

    def status(self) -> dict:
        return self.command("status").fields

    def params(self) -> dict:
        return self.command("params").fields

    def set_param(self, name: str, value: float) -> float:
        """Returns the value the firmware actually applied (after clamping)."""
        r = self.command(f"param {name} {value:.6g}")
        return float(r.fields[name])

    # ---------- recording ----------

    def start_recording(self, path: Path, meta: Optional[dict] = None) -> None:
        """Write every telemetry sample to a CSV until stop_recording()."""
        path.parent.mkdir(parents=True, exist_ok=True)
        f = open(path, "w", newline="")
        f.write(f"# recorded {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        for k, v in (meta or {}).items():
            f.write(f"# {k}: {v}\n")
        w = csv.writer(f)
        w.writerow(["host_time", "t_s", "mode", "enabled", "target", "pos_deg", "vel_dps", "uq_v"])
        with self._rec_lock:
            self._rec_file, self._rec_writer = f, w
            self.rec_path, self.rec_rows = path, 0

    def stop_recording(self) -> Optional[Path]:
        with self._rec_lock:
            path = self.rec_path
            if self._rec_file:
                self._rec_file.close()
            self._rec_file = self._rec_writer = None
            self.rec_path = None
        return path

    @property
    def recording(self) -> bool:
        return self._rec_file is not None

    # ---------- threads ----------

    def _start_thread(self, target, name):
        t = threading.Thread(target=target, name=name, daemon=True)
        t.start()
        self._threads.append(t)

    def _shutdown_threads(self):
        self._stop.set()
        for t in self._threads:
            if t is not threading.current_thread():
                t.join(timeout=1.0)
        self._threads.clear()

    def _drain_replies(self):
        while True:
            try:
                self._replies.get_nowait()
            except queue.Empty:
                return

    def _reader_loop(self):
        while not self._stop.is_set():
            try:
                line = self.transport.read_line(timeout=0.1)
            except Exception as e:   # unplugged cable, port closed under us
                self.link_ok = False
                self.on_log(f"! link lost: {e}")
                return
            if line is None or not line.strip():
                continue
            if line.startswith("d,"):
                s = self._parse_sample(line)
                if s:
                    self.on_sample(s)
                    self._record(s)
                continue
            r = parse_reply(line)
            if r:
                self._replies.put(r)
                if r.cmd not in ("ping", "status"):   # keep the console readable
                    self.on_log(("<< " if r.ok else "!! ") + line)
            else:
                self.on_log(line)   # "# ..." info, SimpleFOC "MOT: ..." debug, anything else

    def _heartbeat_loop(self):
        misses = 0
        while not self._stop.wait(HEARTBEAT_S):
            if time.monotonic() - self._last_cmd_time < HEARTBEAT_S:
                continue   # a real command just went out, that counts
            if not self._cmd_lock.acquire(timeout=0):
                continue   # a command is in flight (maybe init), it refreshes the watchdog when done
            self._cmd_lock.release()
            try:
                self.command("ping", timeout=0.5)
                if not self.link_ok:
                    self.on_log("# link back")
                misses, self.link_ok = 0, True
            except ActuatorError:
                misses += 1
                if misses == 3:
                    self.link_ok = False
                    self.on_log("! no reply to 3 pings. Firmware watchdog will have disabled the motor")
            except Exception:
                self.link_ok = False
                return

    def _parse_sample(self, line: str) -> Optional[Sample]:
        try:
            _, t_us, mode, en, tgt, pos, vel, uq = line.split(",")
            raw = int(t_us)
        except ValueError:
            return None
        if self._t_last_raw is not None and raw < self._t_last_raw - 2**31:
            self._t_offset += 2**32     # micros() wrapped
        self._t_last_raw = raw
        return Sample(t=(raw + self._t_offset) * 1e-6, mode=int(mode), enabled=en == "1",
                      target=float(tgt), pos=float(pos), vel=float(vel), uq=float(uq),
                      host_t=time.time())

    def _record(self, s: Sample):
        with self._rec_lock:
            if self._rec_writer:
                self._rec_writer.writerow([f"{s.host_t:.4f}", f"{s.t:.6f}", s.mode, int(s.enabled),
                                           s.target, s.pos, s.vel, s.uq])
                self.rec_rows += 1
