"""Simulated actuator: speaks the same line protocol as firmware/actuator.

Use it to try the GUI with no hardware (python gui.py --sim), and for tests.
The physics are rough on purpose: a stepper in voltage mode with back-EMF and
friction, through a 25:1 reversing gearbox. Good enough to see the loops work,
not for tuning gains you'll use on the real thing.
"""

from __future__ import annotations

import math
import queue
import threading
import time
from typing import Optional

from actuator import Transport

MODES = ("torque", "vel", "pos")


class _PID:
    """Same structure as SimpleFOC's PIDController: P + trapezoidal I + D, output ramp and clamp."""

    def __init__(self, P, I, D, ramp, limit):
        self.P, self.I, self.D, self.ramp, self.limit = P, I, D, ramp, limit
        self.reset()

    def reset(self):
        self.i_prev = self.e_prev = self.out_prev = 0.0

    def __call__(self, e, dt):
        p = self.P * e
        i = self.i_prev + self.I * dt * 0.5 * (e + self.e_prev)
        i = max(-self.limit, min(self.limit, i))
        d = self.D * (e - self.e_prev) / dt
        out = max(-self.limit, min(self.limit, p + i + d))
        if self.ramp > 0:   # limit how fast the output can change
            rate = (out - self.out_prev) / dt
            if rate > self.ramp:
                out = self.out_prev + self.ramp * dt
            elif rate < -self.ramp:
                out = self.out_prev - self.ramp * dt
        self.i_prev, self.e_prev, self.out_prev = i, e, out
        return out


class SimTransport(Transport):
    # Motor-side plant
    J = 4e-5          # kg m^2, rotor + reflected gearbox
    KT = 0.30         # N m / A
    KE = 0.30         # V / (rad/s)
    R = 1.48          # ohm per coil
    COULOMB = 0.012   # N m, gearbox friction at the motor
    VISC = 2e-4       # N m / (rad/s)
    DT = 0.0005       # s, physics + control step

    def __init__(self):
        self._out: "queue.Queue[str]" = queue.Queue()
        self._in: "queue.Queue[str]" = queue.Queue()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._reset_state()

    def _reset_state(self):
        self.theta = 0.0      # motor rad
        self.omega = 0.0      # motor rad/s
        self.omega_f = 0.0    # filtered velocity
        self.uq = 0.0
        self.initialized = False
        self.enabled = False
        self.mode = 0
        self.target = 0.0
        self.zero_deg = 0.0
        self.ratio, self.dir = 25.0, -1
        self.ulim, self.vlim = 2.5, 12.0
        self.valign, self.vf, self.ap = 2.0, 0.01, 10.0
        self.pid = _PID(0.2, 5.0, 0.0, 1000.0, self.ulim)
        self.wd_ms, self.wd_trip, self.last_rx = 0, False, time.monotonic()
        self.stream_hz, self.last_stream = 0, 0.0
        self.t0 = time.monotonic()

    # ---------- Transport ----------

    def open(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="sim", daemon=True)
        self._thread.start()
        self._out.put("# actuator ready (SIMULATOR). Supply on, then send: init")

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def write_line(self, line: str):
        self._in.put(line)

    def read_line(self, timeout: float):
        try:
            return self._out.get(timeout=timeout)
        except queue.Empty:
            return None

    # ---------- conversions (mirror the firmware) ----------

    def out_pos(self):
        return math.degrees(self.dir * self.theta / self.ratio) - self.zero_deg

    def out_vel(self):
        return math.degrees(self.dir * self.omega_f / self.ratio)

    def motor_target(self):
        if self.mode == 0:
            self.target = max(-self.ulim, min(self.ulim, self.target))
            return self.dir * self.target
        if self.mode == 1:
            self.target = max(-self.vlim, min(self.vlim, self.target))
            return self.dir * self.ratio * math.radians(self.target)
        return self.dir * self.ratio * math.radians(self.target + self.zero_deg)

    def set_mode(self, m):
        self.pid.reset()
        self.mode = m
        self.target = self.out_pos() if m == 2 else 0.0

    # ---------- main loop ----------

    def _run(self):
        last = time.perf_counter()
        while not self._stop.is_set():
            while not self._in.empty():
                line = self._in.get()
                self._handle(line)
                self.last_rx = time.monotonic()

            now = time.perf_counter()
            steps = min(int((now - last) / self.DT), 200)   # catch up, but never spiral
            for _ in range(steps):
                self._step()
            last += steps * self.DT
            if steps == 200:
                last = now

            if self.wd_ms and self.enabled and (time.monotonic() - self.last_rx) * 1000 > self.wd_ms:
                self.enabled, self.wd_trip = False, True
                self._out.put("# wd_trip: no command from host, driver disabled")

            if self.stream_hz and self.initialized and time.monotonic() - self.last_stream >= 1.0 / self.stream_hz:
                self.last_stream = time.monotonic()
                t_us = int((time.monotonic() - self.t0) * 1e6) & 0xFFFFFFFF
                self._out.put(f"d,{t_us},{self.mode},{int(self.enabled)},{self.target:.3f},"
                              f"{self.out_pos():.3f},{self.out_vel():.2f},{self.dir * self.uq:.3f}")
            time.sleep(0.001)

    def _step(self):
        dt = self.DT
        a = dt / (self.vf + dt) if self.vf > 0 else 1.0
        self.omega_f += a * (self.omega - self.omega_f)

        if self.initialized and self.enabled:
            tgt = self.motor_target()
            if self.mode == 0:
                u = tgt
            else:
                if self.mode == 2:
                    vmax = self.ratio * math.radians(self.vlim)
                    tgt = max(-vmax, min(vmax, self.ap * (tgt - self.theta)))
                u = self.pid(tgt - self.omega_f, dt)
            self.uq = max(-self.ulim, min(self.ulim, u))
        else:
            self.uq = 0.0

        current = (self.uq - self.KE * self.omega) / self.R if self.enabled else 0.0
        torque = self.KT * current - self.VISC * self.omega
        # Coulomb friction: sticks below breakaway, opposes motion above it
        if abs(self.omega) < 1e-3 and abs(torque) <= self.COULOMB:
            self.omega = 0.0
            return
        torque -= math.copysign(self.COULOMB, self.omega if abs(self.omega) > 1e-3 else torque)
        self.omega += torque / self.J * dt
        self.theta += self.omega * dt

    # ---------- protocol (same commands and replies as the firmware) ----------

    def _params(self):
        return {"vp": self.pid.P, "vi": self.pid.I, "vd": self.pid.D, "vf": self.vf,
                "ramp": self.pid.ramp, "ap": self.ap, "ulim": self.ulim, "vlim": self.vlim,
                "valign": self.valign, "ratio": self.ratio, "dir": float(self.dir)}

    def _set_param(self, name, v):
        if name == "vp": self.pid.P = max(v, 0)
        elif name == "vi": self.pid.I = max(v, 0)
        elif name == "vd": self.pid.D = max(v, 0)
        elif name == "vf": self.vf = min(max(v, 0), 0.2)
        elif name == "ramp": self.pid.ramp = max(v, 0)
        elif name == "ap": self.ap = max(v, 0)
        elif name == "ulim": self.ulim = min(max(v, 0.1), 2.8); self.pid.limit = self.ulim
        elif name == "vlim": self.vlim = min(max(v, 0.5), 60.0)
        elif name == "valign": self.valign = min(max(v, 0.5), 2.8)
        elif name == "ratio":
            if v >= 1: self.ratio = v
        elif name == "dir": self.dir = -1 if v < 0 else 1

    def _handle(self, line):
        o = self._out.put
        tok = line.strip().lower().split()
        if not tok:
            return
        cmd, args = tok[0], tok[1:]
        need_init = cmd in ("enable", "mode", "set", "zero")
        if need_init and not self.initialized:
            o(f"err {cmd} not_initialized"); return

        def num(i):
            try:
                return float(args[i])
            except (IndexError, ValueError):
                return None

        if cmd == "ping": o("ok ping")
        elif cmd == "init":
            if self.initialized:
                o("ok init already=1"); return
            o("# initFOC: the shaft will twitch. Hands off.")
            o("MOT: Align sensor."); time.sleep(0.8); o("MOT: sensor_direction==CW"); o("MOT: Ready.")
            self.initialized = True
            self.set_mode(0)
            self.zero_deg = self.out_pos() + self.zero_deg
            self.enabled = False
            o("ok init sensor_dir=cw zero_el=1.2345")
        elif cmd == "enable":
            self.set_mode(self.mode); self.enabled = True; self.wd_trip = False; o("ok enable")
        elif cmd in ("disable", "stop"):
            self.enabled = False; o("ok disable")
        elif cmd == "mode":
            m = {"torque": 0, "vel": 1, "pos": 2}.get(args[0] if args else "")
            if m is None:
                o("err mode usage: mode torque|vel|pos"); return
            self.set_mode(m); o(f"ok mode mode={MODES[m]} target={self.target:.3f}")
        elif cmd == "set":
            v = num(0)
            if v is None:
                o("err set usage: set <value>"); return
            self.target = v; self.motor_target(); o(f"ok set target={self.target:.3f}")
        elif cmd == "zero":
            self.zero_deg += self.out_pos()
            if self.mode == 2:
                self.target = 0.0
            o("ok zero")
        elif cmd == "stream":
            v = num(0)
            if v is None or not 0 <= v <= 500:
                o("err stream usage: stream <0..500 Hz>"); return
            self.stream_hz = int(v); o(f"ok stream hz={self.stream_hz}")
        elif cmd == "wd":
            v = num(0)
            if v is None or v < 0 or 0 < v < 100:
                o("err wd usage: wd <0 off, or >=100 ms>"); return
            self.wd_ms, self.wd_trip = int(v), False; o(f"ok wd ms={self.wd_ms}")
        elif cmd == "param":
            p = self._params()
            if not args or args[0] not in p:
                o("err param unknown_name"); return
            if len(args) > 1:
                v = num(1)
                if v is None:
                    o("err param bad_value"); return
                self._set_param(args[0], v)
            o(f"ok param {args[0]}={self._params()[args[0]]:.5f}")
        elif cmd == "params":
            o("ok params " + " ".join(f"{k}={v:.5f}" for k, v in self._params().items()))
        elif cmd == "status":
            o(f"ok status init={int(self.initialized)} en={int(self.enabled)} mode={MODES[self.mode]} "
              f"target={self.target:.3f} pos={self.out_pos():.3f} vel={self.out_vel():.3f} "
              f"uq={self.dir * self.uq:.3f} wd={self.wd_ms} wd_trip={int(self.wd_trip)} "
              f"stream={self.stream_hz} loop_hz=9999 dropped=0")
        elif cmd == "rate": o("ok rate loop_hz=9999")
        elif cmd == "help": o("# (simulator) same commands as the firmware"); o("ok help")
        else: o(f"err {cmd} unknown_command")
