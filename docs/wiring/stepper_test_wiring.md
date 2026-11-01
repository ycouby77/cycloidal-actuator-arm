# Stepper Test Wiring (Open-Loop, L298N)

First open-loop stepper test, Oct 31, 2026. One 17HS19-2004D stepper, one L298N module, NUCLEO-G474RE, bench supply.
Firmware: `firmware/stepper_test`

## Module setup
- All three L298N jumper caps removed: **5V-EN**, **ENA**, **ENB**
- L298N logic powered from the Nucleo 5V (onboard regulator off, since Vs is below ~7 V)
- No motor current through the breadboard

## Connections

| # | From | To | Connection | Wire colour |
|---|---|---|---|---|
| 1 | Nucleo D3 | L298N IN1 | M-F jumper | Blue |
| 2 | Nucleo D5 | L298N IN2 | M-F jumper | Green |
| 3 | Nucleo D6 | L298N IN3 | M-F jumper | White |
| 4 | Nucleo D9 | L298N IN4 | M-F jumper | Orange |
| 5 | Nucleo D10 | L298N ENA (enable pin, in line with INs) | M-F jumper | Purple |
| 6 | Nucleo D11 | L298N ENB (enable pin, in line with INs) | M-F jumper | Yellow |
| 7 | Nucleo 5V | L298N +5V terminal | Jumper, stripped end | Red / marked |
| 8 | Nucleo GND | Supply − clip | Jumper, twisted with row 9 | Black |
| 9 | Supply − clip | L298N GND terminal | Short stub | Black |
| 10 | Supply + (3.4 V, 1 A limit) | L298N Vs / "12V" terminal | Alligator to short stub | Red |
| 11 | Motor GRN | L298N OUT1 | Motor lead | Green |
| 12 | Motor BLK | L298N OUT2 | Motor lead | Black |
| 13 | Motor RED | L298N OUT3 | Motor lead | Red |
| 14 | Motor BLU | L298N OUT4 | Motor lead | Blue |

Coil A = GRN/BLK (OUT1/OUT2), coil B = RED/BLU (OUT3/OUT4).

## Power-up order
1. Motor supply output off
2. Nucleo on USB, flash if needed, confirm `STOP` on serial (115200)
3. Motor supply output on
4. `g` to run

Power-down is the reverse. Always flash and reset with the motor supply off (pins float during reset).

## Serial commands
`w` wave, `t` two-phase, `+`/`-` speed, `r` reverse, `g` go, `s` stop, `p` status, `n` single step and hold

## Results
- Expected per-step coil voltages confirmed with `n` (about ±0.9-1.0 V per coil at ~0.5 A)
- Clean rotation up to ~333 steps/s (smoothest, lowest current). Heavy vibration and missed steps near 500 steps/s
- Ground offset (Nucleo GND to L298N GND) in two-phase: 0.06 V

## Issues found
- **L298N module #1:** IN4/OUT4 channel dead (B- step gave 0 V, 0 A with IN4 at 3.1 V on the header). Set aside, untested whether trace or chip.
- 5V-EN cap initially left on (regulator back-fed from the Nucleo 5V). Removed, no damage seen.

## For the permanent build / KiCad schematic
- Proper ground bus or terminal block instead of joining at the supply clip
- 10 kΩ pull-downs on ENA/ENB so bridges stay off during reset
- Check timer channels of the PWM pins before SimpleFOC
