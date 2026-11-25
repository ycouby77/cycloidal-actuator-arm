# Actuator GUI

Desktop tool for driving and testing one actuator over USB serial. It talks the protocol in [`docs/protocol.md`](../../docs/protocol.md) to `firmware/actuator`.

## Setup (once)

```powershell
cd C:\dev\cycloidal-actuator-arm\host
python -m venv .venv
.venv\Scripts\activate
pip install -r actuator_gui\requirements.txt
```

## Run

```powershell
cd C:\dev\cycloidal-actuator-arm\host\actuator_gui
..\.venv\Scripts\python gui.py              # pick the port in the window
..\.venv\Scripts\python gui.py --port COM5  # connect straight away
..\.venv\Scripts\python gui.py --sim        # no hardware: simulated actuator
```

Close the PlatformIO serial monitor first. Only one program can hold the COM port.

## Using it

1. **Connect.** Pick the STLink port. The GUI arms a 1 s watchdog and starts telemetry.
2. **Bench supply on** (6.0 V, 1.5 A limit).
3. **Init FOC.** The shaft twitches during alignment, so keep your hands off. If it fails with the gearbox on, raise *Align voltage* a bit (e.g. 2.3 V), Apply, and try again.
4. **Pick a mode, then Enable.** Enable always starts safe: it holds position in Position mode, and starts at 0 in the others.
5. **Setpoint.** Type a value and press Enter or Send. The slider sends when you let go. **± step** moves the target by the step size, for step responses.
6. **STOP** (or **Esc**, from anywhere in the window) disables the driver immediately.

All setpoints are output side: deg, deg/s, or volts in torque mode.

- **Parameters.** Edit a value (it turns yellow), then *Apply changes*. The firmware clamps values and the GUI re-reads what was applied.
- **Record.** Type a label (e.g. `stall_2V_r150`) and press Record. Every telemetry sample goes to `host/logs/<date>_<time>_<label>.csv`, with the params written in the file header.
- **Console.** Type any raw protocol command, e.g. `help`, `param vp 0.15`, `status`.

## Files

| File | What it is |
|---|---|
| `actuator.py` | `Transport` (serial today, CAN later) and `Actuator` (commands, telemetry, heartbeat, recording). No GUI code, so scripts can use it directly |
| `gui.py` | PySide6 + pyqtgraph window |
| `sim.py` | Fake actuator speaking the same protocol, for `--sim` |

## Scripting a test without the GUI

```python
from actuator import Actuator, SerialTransport
a = Actuator(SerialTransport("COM5"))
a.connect(stream_hz=200)
a.init(); a.set_mode("pos"); a.enable()
a.set_target(30)
print(a.status())
a.close()   # disables the motor
```
