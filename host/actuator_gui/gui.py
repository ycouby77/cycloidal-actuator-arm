"""Actuator test GUI.

    python gui.py              pick the Nucleo's COM port in the window
    python gui.py --port COM5  connect straight away
    python gui.py --sim        no hardware, simulated actuator

Esc = STOP (disables the driver) from anywhere in the window.
"""

from __future__ import annotations

import argparse
import math
import queue
import sys
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtGui import QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPlainTextEdit,
    QPushButton, QRadioButton, QSizePolicy, QSlider, QSpinBox, QSplitter, QVBoxLayout, QWidget,
)

from actuator import MODES, MODE_UNITS, Actuator, ActuatorError, SerialTransport, list_serial_ports

LOG_DIR = Path(__file__).resolve().parent.parent / "logs"   # host/logs
SIM_PORT = "Simulator"
BUFFER_S = 120          # seconds of telemetry kept for the plots

# Setpoint ranges per mode: (min, max, default step)
SETPOINT_RANGE = {"torque": (-2.8, 2.8, 0.1), "vel": (-60.0, 60.0, 5.0), "pos": (-3600.0, 3600.0, 10.0)}

# Tunable firmware params: name -> (label, min, max, decimals, step)
PARAM_SPECS = {
    "ulim":   ("Voltage limit (V)",          0.1, 2.8,  2, 0.1),
    "vlim":   ("Output vel limit (deg/s)",   0.5, 60.0, 1, 1.0),
    "vp":     ("Velocity P",                 0.0, 10.0, 4, 0.01),
    "vi":     ("Velocity I",                 0.0, 200.0, 3, 0.5),
    "vd":     ("Velocity D",                 0.0, 1.0,  5, 0.0001),
    "vf":     ("Velocity filter Tf (s)",     0.0, 0.2,  4, 0.001),
    "ramp":   ("Voltage ramp (V/s, 0=off)",  0.0, 10000, 0, 50),
    "ap":     ("Angle P",                    0.0, 200.0, 2, 1.0),
    "valign": ("Align voltage (V)",          0.5, 2.8,  2, 0.1),
}


class Bridge(QObject):
    """Carries results from the worker thread back to the GUI thread."""
    done = Signal(object)


class MainWindow(QMainWindow):
    def __init__(self, port: str | None = None, sim: bool = False):
        super().__init__()
        self.setWindowTitle("Cycloidal actuator")
        self.resize(1400, 900)

        self.act: Actuator | None = None
        self.pool = ThreadPoolExecutor(max_workers=1)   # every command runs here, in order
        self.bridge = Bridge()
        self.bridge.done.connect(self._on_done)
        self.sample_q: "queue.Queue" = queue.Queue()
        self.log_q: "queue.Queue[str]" = queue.Queue()
        self.buf: deque = deque(maxlen=BUFFER_S * 500)
        self.last_sample = None
        self.status: dict = {}
        self.param_values: dict = {}
        self.mode = "torque"
        self._status_busy = False
        self._suppress_mode_signal = False

        self._build_ui()
        self._refresh_ports(select=SIM_PORT if sim else port)

        # GUI timers: plots + queues at ~30 Hz, status poll at 2 Hz
        self.ui_timer = QTimer(self, interval=33, timeout=self._tick)
        self.ui_timer.start()
        self.status_timer = QTimer(self, interval=500, timeout=self._poll_status)
        self.status_timer.start()

        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self._stop,
                  context=Qt.ApplicationShortcut)

        self._set_connected_ui(False)
        if sim or port:
            QTimer.singleShot(100, self._toggle_connect)

    # =================================================================
    # UI construction
    # =================================================================

    def _build_ui(self):
        root = QSplitter(Qt.Horizontal)
        self.setCentralWidget(root)

        # ---------- left: controls ----------
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(6, 6, 6, 6)

        # Connection
        g = QGroupBox("Connection")
        gl = QGridLayout(g)
        self.port_combo = QComboBox()
        self.port_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        refresh = QPushButton("↻")
        refresh.setFixedWidth(32)
        refresh.clicked.connect(lambda: self._refresh_ports())
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connect)
        self.stream_spin = QSpinBox(minimum=10, maximum=500, value=100, suffix=" Hz")
        self.stream_spin.setToolTip("Telemetry rate. 100 Hz is plenty; 200+ for step responses")
        self.stream_spin.editingFinished.connect(self._apply_stream)
        gl.addWidget(self.port_combo, 0, 0, 1, 2)
        gl.addWidget(refresh, 0, 2)
        gl.addWidget(QLabel("Telemetry"), 1, 0)
        gl.addWidget(self.stream_spin, 1, 1, 1, 2)
        gl.addWidget(self.connect_btn, 2, 0, 1, 3)
        lv.addWidget(g)

        # Actuator state + big buttons
        g = QGroupBox("Actuator")
        gl = QGridLayout(g)
        self.state_label = QLabel("—")
        self.state_label.setWordWrap(True)
        self.init_btn = QPushButton("Init FOC")
        self.init_btn.setToolTip("Bench supply ON first. Runs initFOC: the shaft twitches.")
        self.init_btn.clicked.connect(self._init)
        self.enable_btn = QPushButton("Enable")
        self.enable_btn.clicked.connect(lambda: self._run(self.act.enable))
        self.zero_btn = QPushButton("Zero here")
        self.zero_btn.setToolTip("Current output position becomes 0 deg")
        self.zero_btn.clicked.connect(lambda: self._run(self.act.zero))
        self.stop_btn = QPushButton("STOP  (Esc)")
        self.stop_btn.setMinimumHeight(56)
        self.stop_btn.setStyleSheet("QPushButton { background:#c0392b; color:white; font-size:18px; font-weight:bold; }"
                                    "QPushButton:disabled { background:#8a8a8a; }")
        self.stop_btn.clicked.connect(self._stop)
        gl.addWidget(self.state_label, 0, 0, 1, 3)
        gl.addWidget(self.init_btn, 1, 0)
        gl.addWidget(self.enable_btn, 1, 1)
        gl.addWidget(self.zero_btn, 1, 2)
        gl.addWidget(self.stop_btn, 2, 0, 1, 3)
        lv.addWidget(g)

        # Mode
        g = QGroupBox("Mode")
        hl = QHBoxLayout(g)
        self.mode_group = QButtonGroup(self)
        for i, (name, label) in enumerate([("torque", "Torque (V)"), ("vel", "Velocity"), ("pos", "Position")]):
            rb = QRadioButton(label)
            rb.setProperty("mode", name)
            self.mode_group.addButton(rb, i)
            hl.addWidget(rb)
        self.mode_group.button(0).setChecked(True)
        self.mode_group.idToggled.connect(self._mode_toggled)
        lv.addWidget(g)

        # Setpoint
        g = QGroupBox("Setpoint (output side)")
        gl = QGridLayout(g)
        self.sp_spin = QDoubleSpinBox(decimals=3, keyboardTracking=False)
        self.sp_spin.setMinimumWidth(110)
        self.sp_send = QPushButton("Send")
        self.sp_send.clicked.connect(lambda: self._send_target(self.sp_spin.value()))
        self.sp_spin.lineEdit().returnPressed.connect(lambda: self._send_target(self.sp_spin.value()))
        self.sp_slider = QSlider(Qt.Horizontal)
        self.sp_slider.setToolTip("Sends on release")
        self.sp_slider.valueChanged.connect(self._slider_moved)
        self.sp_slider.sliderReleased.connect(lambda: self._send_target(self.sp_spin.value()))
        self.step_spin = QDoubleSpinBox(decimals=3, minimum=0.001, maximum=3600, value=1.0)
        minus = QPushButton("− step")
        plus = QPushButton("+ step")
        minus.clicked.connect(lambda: self._send_target(self._current_target() - self.step_spin.value()))
        plus.clicked.connect(lambda: self._send_target(self._current_target() + self.step_spin.value()))
        self.hold_btn = QPushButton("Hold here")
        self.hold_btn.setToolTip("Position mode: target = where the output is now")
        self.hold_btn.clicked.connect(self._hold_here)
        zero_t = QPushButton("Target 0")
        zero_t.clicked.connect(lambda: self._send_target(0.0))
        self.sp_units = QLabel("V")
        gl.addWidget(self.sp_spin, 0, 0)
        gl.addWidget(self.sp_units, 0, 1)
        gl.addWidget(self.sp_send, 0, 2)
        gl.addWidget(self.sp_slider, 1, 0, 1, 3)
        gl.addWidget(QLabel("Step size"), 2, 0)
        gl.addWidget(self.step_spin, 2, 1, 1, 2)
        gl.addWidget(minus, 3, 0)
        gl.addWidget(plus, 3, 1)
        gl.addWidget(zero_t, 3, 2)
        gl.addWidget(self.hold_btn, 4, 0, 1, 3)
        lv.addWidget(g)
        self.setpoint_widgets = [self.sp_spin, self.sp_send, self.sp_slider, minus, plus, zero_t, self.hold_btn]

        # Params
        g = QGroupBox("Parameters")
        fl = QFormLayout(g)
        self.param_spins: dict[str, QDoubleSpinBox] = {}
        for name, (label, lo, hi, dec, step) in PARAM_SPECS.items():
            s = QDoubleSpinBox(minimum=lo, maximum=hi, decimals=dec, singleStep=step, keyboardTracking=False)
            s.valueChanged.connect(self._mark_param_dirty)
            self.param_spins[name] = s
            fl.addRow(label, s)
        hl = QHBoxLayout()
        read_btn = QPushButton("Read")
        read_btn.clicked.connect(self._read_params)
        self.apply_btn = QPushButton("Apply changes")
        self.apply_btn.clicked.connect(self._apply_params)
        hl.addWidget(read_btn)
        hl.addWidget(self.apply_btn)
        fl.addRow(hl)
        lv.addWidget(g)
        self.param_widgets = list(self.param_spins.values()) + [read_btn, self.apply_btn]

        # Recording
        g = QGroupBox("Record to CSV")
        gl = QGridLayout(g)
        self.rec_label = QLineEdit(placeholderText="label, e.g. stall_2V_r150")
        self.rec_btn = QPushButton("● Record")
        self.rec_btn.setCheckable(True)
        self.rec_btn.toggled.connect(self._toggle_record)
        self.rec_info = QLabel(f"Saves to {LOG_DIR}")
        self.rec_info.setWordWrap(True)
        self.rec_info.setTextInteractionFlags(Qt.TextSelectableByMouse)
        gl.addWidget(self.rec_label, 0, 0)
        gl.addWidget(self.rec_btn, 0, 1)
        gl.addWidget(self.rec_info, 1, 0, 1, 2)
        lv.addWidget(g)
        lv.addStretch(1)

        self.control_widgets = ([self.init_btn, self.enable_btn, self.zero_btn, self.stop_btn, self.stream_spin,
                                 self.rec_btn, self.rec_label] + list(self.mode_group.buttons())
                                + self.setpoint_widgets + self.param_widgets)

        left.setMaximumWidth(420)
        root.addWidget(left)

        # ---------- right: readouts, plots, console ----------
        right = QSplitter(Qt.Vertical)

        top = QWidget()
        tv = QVBoxLayout(top)
        tv.setContentsMargins(0, 0, 0, 0)

        ro = QHBoxLayout()
        big = QFont()
        big.setPointSize(18)
        big.setBold(True)
        self.readouts = {}
        for key, label in [("pos", "Position"), ("vel", "Velocity"), ("uq", "Uq"), ("target", "Target")]:
            box = QVBoxLayout()
            cap = QLabel(label)
            cap.setAlignment(Qt.AlignCenter)
            val = QLabel("—")
            val.setFont(big)
            val.setAlignment(Qt.AlignCenter)
            box.addWidget(cap)
            box.addWidget(val)
            ro.addLayout(box)
            self.readouts[key] = val
        tv.addLayout(ro)

        ph = QHBoxLayout()
        ph.addWidget(QLabel("Window"))
        self.window_spin = QSpinBox(minimum=2, maximum=BUFFER_S, value=10, suffix=" s")
        ph.addWidget(self.window_spin)
        self.pause_chk = QCheckBox("Pause plots")
        ph.addWidget(self.pause_chk)
        clear = QPushButton("Clear")
        clear.clicked.connect(self.buf.clear)
        ph.addWidget(clear)
        ph.addStretch(1)
        tv.addLayout(ph)

        pg.setConfigOptions(antialias=True)
        self.plots = pg.GraphicsLayoutWidget()
        pen_act = pg.mkPen("#2e86de", width=2)
        pen_tgt = pg.mkPen("#e67e22", width=2, style=Qt.DashLine)
        self.curves = {}
        prev = None
        for row, (key, title, unit) in enumerate([("pos", "Position", "deg"),
                                                  ("vel", "Velocity", "deg/s"),
                                                  ("uq", "Uq", "V")]):
            p = self.plots.addPlot(row=row, col=0)
            p.setLabel("left", title, units=unit)
            p.showGrid(x=True, y=True, alpha=0.3)
            p.addLegend(offset=(5, 5))
            if prev is not None:
                p.setXLink(prev)
            prev = p
            self.curves[key] = (p.plot(pen=pen_act, name="actual"),
                                p.plot(pen=pen_tgt, name="target", connect="finite"))
        prev.setLabel("bottom", "time", units="s")
        tv.addWidget(self.plots, 1)
        right.addWidget(top)

        # Console
        con = QWidget()
        cv = QVBoxLayout(con)
        cv.setContentsMargins(0, 0, 0, 0)
        self.console = QPlainTextEdit(readOnly=True)
        self.console.setMaximumBlockCount(3000)
        self.console.setFont(QFont("Consolas", 9))
        self.cmd_line = QLineEdit(placeholderText="raw command, e.g.  help   param vp 0.15   status")
        self.cmd_line.returnPressed.connect(self._send_raw)
        cv.addWidget(self.console)
        cv.addWidget(self.cmd_line)
        right.addWidget(con)
        right.setSizes([650, 220])

        root.addWidget(right)
        root.setSizes([400, 1000])
        self._apply_mode_ui("torque")

    # =================================================================
    # Worker plumbing: run actuator calls off the GUI thread
    # =================================================================

    def _run(self, fn, *args, on_ok=None, on_err=None, label=None):
        """Run fn(*args) on the worker thread, in order with every other command.
        on_ok(result) / on_err(exception) then run back on the GUI thread."""
        if not self.act:
            return
        label = label or getattr(fn, "__name__", "cmd")
        fut = self.pool.submit(fn, *args)
        fut.add_done_callback(lambda f: self.bridge.done.emit((f, on_ok, on_err, label)))

    def _on_done(self, payload):
        fut, on_ok, on_err, label = payload
        try:
            result = fut.result()
        except Exception as e:   # ActuatorError, or the port vanished
            msg = str(e) if isinstance(e, ActuatorError) else f"{type(e).__name__}: {e}"
            if label != "status":   # the 2 Hz poll would flood the console; the state line shows LINK LOST
                self._log(f"!! {label}: {msg}")
            if on_err:
                on_err(e)
            return
        if on_ok:
            on_ok(result)

    # =================================================================
    # Connection
    # =================================================================

    def _refresh_ports(self, select: str | None = None):
        current = select or self.port_combo.currentData()
        self.port_combo.clear()
        for dev, desc in list_serial_ports():
            self.port_combo.addItem(f"{dev}  {desc}", dev)
        self.port_combo.addItem("Simulator (no hardware)", SIM_PORT)
        if current:
            i = self.port_combo.findData(current)
            if i >= 0:
                self.port_combo.setCurrentIndex(i)

    def _toggle_connect(self):
        if self.act:
            act, self.act = self.act, None
            self.status_timer.stop()
            if self.rec_btn.isChecked():
                self.rec_btn.setChecked(False)
            self.pool.submit(act.close).add_done_callback(lambda f: self.bridge.done.emit((f, None, None, "close")))
            self._set_connected_ui(False)
            self._log("# disconnected (motor disabled)")
            return

        port = self.port_combo.currentData()
        if port == SIM_PORT:
            from sim import SimTransport
            transport = SimTransport()
        else:
            transport = SerialTransport(port)
        self.act = Actuator(transport, on_sample=self.sample_q.put, on_log=self.log_q.put)
        self.connect_btn.setEnabled(False)
        self._log(f"# connecting to {port} ...")

        def ok(st):
            self.connect_btn.setEnabled(True)
            self._log(f"# connected: {st}")
            self._set_connected_ui(True)
            self._apply_status(st.fields)
            self.status_timer.start()
            self._read_params()

        def failed(_e):
            self.connect_btn.setEnabled(True)
            self.act = None
            self._set_connected_ui(False)

        self._run(self.act.connect, self.stream_spin.value(), on_ok=ok, on_err=failed, label="connect")

    def _set_connected_ui(self, on: bool):
        self.connect_btn.setText("Disconnect" if on else "Connect")
        self.port_combo.setEnabled(not on)
        for w in self.control_widgets:
            w.setEnabled(on)
        self.stop_btn.setEnabled(on)
        if not on:
            self.state_label.setText("Not connected")
        self._update_enables()

    def _apply_stream(self):
        if self.act:
            self._run(self.act.set_stream, self.stream_spin.value())

    # =================================================================
    # Actuator actions
    # =================================================================

    def _stop(self):
        if self.act:
            self.act.emergency_stop()   # bypasses the worker queue on purpose
            self._log(">> STOP")

    def _init(self):
        self._log("# init: bench supply must be ON. Shaft will twitch...")
        self.init_btn.setEnabled(False)

        def ok(_r):   # the reply itself is already in the console
            self.status["init"] = 1
            self._update_enables()
        self._run(self.act.init, on_ok=ok, label="init")
        QTimer.singleShot(500, self._update_enables)

    def _mode_toggled(self, idx: int, checked: bool):
        if not checked or self._suppress_mode_signal:
            return
        mode = MODES[idx]
        self._apply_mode_ui(mode)
        if self.act:
            self._run(self.act.set_mode, mode, on_ok=lambda r: self._set_spin_quiet(r.fields.get("target", 0.0)),
                      label="mode")

    def _apply_mode_ui(self, mode: str):
        mode_changed = mode != getattr(self, "_ui_mode", None)
        self._ui_mode = self.mode = mode
        lo, hi, step = SETPOINT_RANGE[mode]
        if mode == "torque":
            ulim = self.param_values.get("ulim", 2.5)
            lo, hi = -ulim, ulim
        elif mode == "vel":
            vlim = self.param_values.get("vlim", 12.0)
            lo, hi = -vlim, vlim
        self.sp_spin.blockSignals(True)
        self.sp_spin.setRange(lo, hi)
        self.sp_spin.setSingleStep(step)
        self.sp_spin.blockSignals(False)
        # slider covers +-limits in torque/vel, +-180 deg around 0 in position
        s_lo, s_hi = (lo, hi) if mode != "pos" else (-180.0, 180.0)
        self.sp_slider.blockSignals(True)
        self.sp_slider.setRange(int(s_lo * 100), int(s_hi * 100))
        self.sp_slider.blockSignals(False)
        if mode_changed:   # keep a step size you typed, unless the units changed
            self.step_spin.setValue(step)
        self.sp_units.setText(MODE_UNITS[mode])
        self.hold_btn.setVisible(mode == "pos")

    def _slider_moved(self, v: int):
        self.sp_spin.setValue(v / 100.0)

    def _set_spin_quiet(self, v: float):
        self.sp_spin.blockSignals(True)
        self.sp_spin.setValue(v)
        self.sp_spin.blockSignals(False)
        self.sp_slider.blockSignals(True)
        self.sp_slider.setValue(int(round(v * 100)))
        self.sp_slider.blockSignals(False)

    def _current_target(self) -> float:
        if self.last_sample is not None:
            return self.last_sample.target
        return self.sp_spin.value()

    def _send_target(self, value: float):
        if not self.act:
            return
        self._run(self.act.set_target, value,
                  on_ok=lambda r: self._set_spin_quiet(r.fields.get("target", value)), label="set")

    def _hold_here(self):
        if self.last_sample is not None:
            self._send_target(self.last_sample.pos)

    def _send_raw(self):
        line = self.cmd_line.text().strip()
        if not line or not self.act:
            return
        self.cmd_line.clear()
        self._log(f">> {line}")
        # The client already echoes replies to the console, except ping/status (hidden from the 2 Hz poll)
        quiet = line.split()[0].lower() in ("ping", "status")
        self._run(self.act.command, line, on_ok=(lambda r: self._log(f"<< {r}")) if quiet else None, label=line)

    # =================================================================
    # Parameters
    # =================================================================

    def _read_params(self):
        def ok(p: dict):
            self.param_values = {k: float(v) for k, v in p.items() if isinstance(v, (int, float))}
            for name, spin in self.param_spins.items():
                if name in self.param_values:
                    spin.blockSignals(True)
                    spin.setValue(self.param_values[name])
                    spin.setStyleSheet("")
                    spin.blockSignals(False)
            self._apply_mode_ui(self.mode)
        self._run(self.act.params, on_ok=ok, label="params")

    def _mark_param_dirty(self):
        for name, spin in self.param_spins.items():
            dirty = name in self.param_values and not math.isclose(spin.value(), self.param_values[name],
                                                                   rel_tol=1e-6, abs_tol=10 ** -spin.decimals())
            spin.setStyleSheet("background:#fff3cd;" if dirty else "")

    def _apply_params(self):
        changes = {n: s.value() for n, s in self.param_spins.items()
                   if n in self.param_values and not math.isclose(s.value(), self.param_values[n],
                                                                  rel_tol=1e-6, abs_tol=10 ** -s.decimals())}
        if not changes:
            self._log("# no parameter changes")
            return

        def apply_all():
            return {n: self.act.set_param(n, v) for n, v in changes.items()}

        def ok(applied):
            self._log("# applied " + ", ".join(f"{k}={v:g}" for k, v in applied.items()))
            self._read_params()
        self._run(apply_all, on_ok=ok, label="apply params")

    # =================================================================
    # Recording
    # =================================================================

    def _toggle_record(self, on: bool):
        if not self.act:
            return
        if not on:
            path = self.act.stop_recording()
            self.rec_btn.setText("● Record")
            if path:
                self.rec_info.setText(f"Saved {path.name}")
                self._log(f"# saved {path}")
            return

        label = "".join(c if c.isalnum() or c in "-_" else "_" for c in self.rec_label.text().strip()) or "run"
        path = LOG_DIR / f"{time.strftime('%Y%m%d_%H%M%S')}_{label}.csv"

        def start():
            meta = {"params": " ".join(f"{k}={v:g}" for k, v in self.act.params().items()),
                    "mode": self.mode, "label": self.rec_label.text().strip()}
            self.act.start_recording(path, meta)
            return path

        def ok(p):
            self.rec_btn.setText("■ Stop")
            self.rec_info.setText(f"Recording → {p.name}")
            self._log(f"# recording to {p}")
        self._run(start, on_ok=ok, label="record")

    # =================================================================
    # Status + periodic UI
    # =================================================================

    def _poll_status(self):
        if not self.act or self._status_busy:
            return
        self._status_busy = True

        def ok(st):
            self._apply_status(st)

        fut = self.pool.submit(self.act.status)

        def cb(f):
            self._status_busy = False
            self.bridge.done.emit((f, ok, None, "status"))
        fut.add_done_callback(cb)

    def _apply_status(self, st: dict):
        self.status = st
        mode = st.get("mode", "torque")
        if mode in MODES and mode != self.mode:   # firmware is the source of truth
            self._suppress_mode_signal = True
            self.mode_group.button(MODES.index(mode)).setChecked(True)
            self._suppress_mode_signal = False
            self._apply_mode_ui(mode)
        self._update_enables()

    def _update_enables(self):
        st = self.status if self.act else {}
        init = bool(st.get("init", 0))
        en = bool(st.get("en", 0))
        connected = self.act is not None
        self.init_btn.setEnabled(connected and not init)
        self.enable_btn.setEnabled(connected and init and not en)
        for w in self.setpoint_widgets + list(self.mode_group.buttons()) + [self.zero_btn]:
            w.setEnabled(connected and init)
        if not connected:
            return
        link = "link OK" if self.act.link_ok else "LINK LOST"
        bits = [f"<b>{'ENABLED' if en else 'disabled'}</b>",
                "initialized" if init else "<span style='color:#c0392b'>not initialized: supply on, then Init</span>",
                f"mode: {st.get('mode', '?')}", link,
                f"loop {int(st.get('loop_hz', 0))} Hz"]
        if st.get("wd_trip"):
            bits.append("<span style='color:#c0392b'>watchdog tripped</span>")
        if st.get("dropped"):
            bits.append(f"dropped {int(st['dropped'])}")
        self.state_label.setText(" · ".join(bits))

    def _log(self, text: str):
        self.console.appendPlainText(text)

    def _tick(self):
        # console
        for _ in range(200):
            try:
                self._log(self.log_q.get_nowait())
            except queue.Empty:
                break
        # samples
        got = False
        while True:
            try:
                s = self.sample_q.get_nowait()
            except queue.Empty:
                break
            self.buf.append((s.t, s.mode, s.target, s.pos, s.vel, s.uq))
            self.last_sample = s
            got = True
        if self.act and self.act.recording:
            self.rec_info.setText(f"Recording → {self.act.rec_path.name}  ({self.act.rec_rows} rows)")
        if not got or self.pause_chk.isChecked():
            return

        s = self.last_sample
        self.readouts["pos"].setText(f"{s.pos:+.2f}°")
        self.readouts["vel"].setText(f"{s.vel:+.1f}°/s")
        self.readouts["uq"].setText(f"{s.uq:+.2f} V")
        self.readouts["target"].setText(f"{s.target:+.2f} {MODE_UNITS[MODES[s.mode]]}")

        data = np.asarray(self.buf)
        t = data[:, 0]
        keep = t >= t[-1] - self.window_spin.value()
        data = data[keep]
        t = data[:, 0] - data[-1, 0]   # now = 0, history negative
        mode = data[:, 1]
        for col, key, mode_idx in [(3, "pos", 2), (4, "vel", 1), (5, "uq", 0)]:
            actual, target = self.curves[key]
            actual.setData(t, data[:, col])
            tgt = np.where(mode == mode_idx, data[:, 2], np.nan)   # target only where it means this quantity
            target.setData(t, tgt)

    def closeEvent(self, ev):
        self.status_timer.stop()
        self.ui_timer.stop()
        if self.act:
            try:
                self.act.close()
            except Exception:
                pass
        self.pool.shutdown(wait=False, cancel_futures=True)
        super().closeEvent(ev)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. COM5")
    ap.add_argument("--sim", action="store_true", help="simulated actuator, no hardware")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    w = MainWindow(port=args.port, sim=args.sim)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
