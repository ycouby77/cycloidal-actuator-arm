# Cycloidal Stepper Actuator + Desk Arm

A custom robot actuator built from a NEMA stepper with closed-loop FOC, a 3D-printed 25:1 cycloidal gearbox, and a CAN link to a PC. Two of these actuators drive the shoulder and elbow of a ~5 DOF desk arm that picks up and organizes small objects.

**Status:** Phase 1 (actuator). Motors received, gearbox parts arriving Nov 2026.
**Target completion:** March 31, 2027

---

## Goals

- **Actuator:** close the loop on a stepper with my own FOC, drive it through a cycloidal gearbox, and command it over CAN
- **Arm:** ~5 DOF + gripper, 50 cm reach, 200 g payload, custom actuators at shoulder and elbow, smart servos elsewhere
- **Task:** scripted pick-and-place on my desk
- **Stretch:** learn the pick from a desk camera with an imitation learning policy

---

## Hardware

| Component | Choice | Notes |
|---|---|---|
| Motor | NEMA stepper (`model TBD`) | Run closed-loop with FOC |
| Driver | L298N | One per stepper |
| MCU | STM32 Nucleo (`model TBD`) | One per actuator |
| Encoder | `TBD` | |
| Gearbox | Cycloidal, 25:1 | 3D printed, steel pins and bearings |
| Comms | CAN | PC to actuators |

### Gearbox Specs

| Parameter | Value |
|---|---|
| Ratio | 25:1 |
| Discs | 2, 180° apart, 7 mm thick |
| Ring pins | 26 x (3 x 20 mm), ~50 mm pin circle |
| Eccentricity | 0.72 mm |
| Cam bearing | 6801 (21 mm disc bore) |
| Output pins | 6 on a 32 mm circle, with rollers |
| Output bearings | 2 x 6808ZZ with spacer |
| Housing OD | 66 mm |
| Fasteners | M3 throughout |

---

## Repo Layout

```
firmware/   C++ for the actuator MCUs (FOC, position/torque control, CAN)
host/       Python on the PC (CAN interface, kinematics, trajectories, test scripts)
cad/        Gearbox and arm CAD, STEP/STL exports
docs/       Design notes, decision log, test data, plots, photos
```

---

## Results

Filled in as testing happens.

| Metric | Actuator 1 | Actuator 2 |
|---|---|---|
| Peak output torque | | |
| Backlash | | |
| Step response (rise time / overshoot) | | |

---

## Roadmap

- [x] Requirements and parts selection
- [x] Gearbox design (v1)
- [ ] Bare motor running closed-loop
- [ ] Gearbox printed and assembled, backlash measured
- [ ] Output-side position and torque control
- [ ] PC commands actuator over CAN
- [ ] Test bench data + second actuator
- [ ] Arm CAD and kinematics
- [ ] Arm assembled, all joints under software control
- [ ] IK and smooth Cartesian moves
- [ ] Scripted pick-and-place demo
- [ ] (Stretch) Learned pick from camera

---

## Author

Yacoub, Mechatronic Systems Engineering + AI Systems Engineering, Western University
