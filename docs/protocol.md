# Actuator serial protocol

Spoken by `firmware/actuator` and `host/actuator_gui`. Plain ASCII lines ending in `\n`, 460800 baud. You can type every command by hand in the PlatformIO monitor too.

## Rules

- One command per line: `name [arg] [arg]`. Lowercase, space separated.
- Every command gets exactly **one** reply line:
  - `ok <name> [key=value ...]`
  - `err <name> <reason>`
- Other lines from the board:
  - `# ...` is info for humans
  - `MOT: ...` is SimpleFOC's debug output during init
  - `d,...` is telemetry
- **Units are output side**, after the gearbox: position in deg, velocity in deg/s, torque mode in volts on the q axis. Positive means the same output direction in every mode.

## Commands

| Command | Reply | Notes |
|---|---|---|
| `ping` | `ok ping` | Heartbeat |
| `status` | `ok status init= en= mode= target= pos= vel= uq= wd= wd_trip= stream= loop_hz= dropped=` | |
| `init` | `ok init sensor_dir= zero_el=` | Runs initFOC. **Bench supply on first.** The shaft twitches. Leaves the driver disabled. Power-up position becomes 0 deg |
| `enable` | `ok enable` | Needs init. Resets to a safe target: hold position (pos), or 0 (vel, torque) |
| `disable` / `stop` | `ok disable` | Coils off |
| `mode torque\|vel\|pos` | `ok mode mode= target=` | Resets PIDs and picks a safe target (pos holds where it is) |
| `set <v>` | `ok set target=` | V, deg/s or deg. The echoed target is after clamping to `ulim` or `vlim` |
| `zero` | `ok zero` | Current output position becomes 0 |
| `stream <hz>` | `ok stream hz=` | 0 to 500. 0 = off |
| `wd <ms>` | `ok wd ms=` | Watchdog: disables the driver if no command arrives for this long while enabled. 0 = off, else at least 100 |
| `param <name> [value]` | `ok param <name>=<value>` | Read, or set and read back (after clamping) |
| `params` | `ok params vp= vi= ...` | All params |
| `rate` | `ok rate loop_hz=` | FOC loop rate |
| `help` | `# ...` lines, then `ok help` | |

## Params

| Name | Meaning | Default | Limits |
|---|---|---|---|
| `vp` `vi` `vd` | Velocity PID (motor side: V per motor rad/s) | 0.2, 5, 0 | at least 0 |
| `vf` | Velocity low-pass filter Tf (s) | 0.01 | 0 to 0.2 |
| `ramp` | Voltage ramp limit (V/s), 0 = off | 1000 | at least 0 |
| `ap` | Angle P (motor rad/s per motor rad) | 10 | at least 0 |
| `ulim` | Voltage limit (V) | 2.5 | 0.1 to 2.8 |
| `vlim` | Output velocity limit (deg/s) | 12 | 0.5 to 60 |
| `valign` | initFOC alignment voltage (V) | 2.0 | 0.5 to 2.8 |
| `ratio` | Gear ratio | 25 | at least 1 |
| `dir` | Output direction relative to the motor, +1 or -1 | -1 | |

Params live in RAM and reset at power-up.

## Telemetry

```
d,<t_us>,<mode>,<enabled>,<target>,<pos_deg>,<vel_dps>,<uq_V>
```

- `t_us` is the board's `micros()`. It wraps every ~71 min, and the host unwraps it.
- `mode`: 0 torque, 1 vel, 2 pos.
- The firmware skips a sample rather than block the FOC loop when the UART buffer is full. `status` reports the count as `dropped`.

## Safety behaviour

- Nothing moves at boot. `init` is needed, then `enable`.
- The GUI arms `wd 1000` and pings every 250 ms. If the GUI crashes, the laptop sleeps or the cable comes out, the motor disables within 1 s.
- `ulim` is hard capped at 2.8 V (~1.9 A in a 1.48 Ω coil, under the 2 A rating).

## Later: CAN

Each command maps onto a CAN frame (command ID + float payload). Telemetry maps onto a periodic status frame. On the host, only the `Transport` class changes.
