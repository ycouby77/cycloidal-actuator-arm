// foc_closed_loop
// SimpleFOC on the bare 17HS19-2004D stepper: open-loop check, initFOC alignment,
// then torque, velocity and angle modes, all switched live over serial.
//
// Board:   NUCLEO-G474RE, PlatformIO + Arduino (STM32duino core 2.12.0, see platformio.ini)
// Driver:  L298N, IN1-IN4 on D3/D5/D6/D9, ENA/ENB on D2/D4
// Encoder: AS5047P board 2 (3.3 V mode), CSn D10, MOSI D11, MISO D12, CLK D13
//
// Supply option B: bench supply ~6.0 V, 1.5 A limit. The L298N drops ~2 V, so we tell
// SimpleFOC the supply is 4 V. Then commanded voltage ~= voltage the coil actually sees.
//
// Power order: USB first, flash, open the monitor. The sketch WAITS before touching the
// motor. Turn the bench supply on, then press Enter. Off: 'X', then supply off, then USB.

#include <Arduino.h>
#include <SimpleFOC.h>

// ---------- Build option ----------
// 1 = step 1: open-loop velocity, no encoder. Proves SimpleFOC's PWM drives the L298N.
// 0 = steps 2-5: encoder linked, initFOC, closed loop.
#define TEST_OPEN_LOOP 0

// ---------- Pins ----------
const int PIN_IN1 = D3, PIN_IN2 = D5;   // coil A (GRN/BLK)
const int PIN_IN3 = D6, PIN_IN4 = D9;   // coil B (RED/BLU)
const int PIN_ENA = D2, PIN_ENB = D4;
const int PIN_ENC_CS = D10;

// ---------- Electrical settings (option B) ----------
const float SUPPLY_EFFECTIVE = 4.0f;   // 6 V bench - ~2 V L298N drop
const float V_LIMIT          = 2.5f;   // coil cap: 2.5 V / 1.48 ohm ~ 1.7 A, under the 2 A rating
const float V_ALIGN          = 2.0f;   // voltage initFOC uses to pull the rotor around
const float V_OPEN_LOOP      = 1.5f;   // open loop runs at a fixed voltage: keep it cooler

// ---------- Objects ----------
// 1.8 deg stepper = 200 steps/rev = 50 electrical cycles per rev = 50 pole pairs
StepperMotor      motor  = StepperMotor(50);
StepperDriver4PWM driver = StepperDriver4PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENA, PIN_ENB);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, PIN_ENC_CS);
Commander         command = Commander(Serial);

// ---------- Loop rate counter ----------
uint32_t loopCount = 0;
uint32_t loopWindowStart = 0;

// =====================================================================
// Serial commands
// =====================================================================
// T<x>   target. Torque mode: volts. Velocity: rad/s. Angle: rad. Open loop: rad/s
// C0/C1/C2  switch to torque / velocity / angle mode (closed-loop build only)
// X      disable the driver (stop, coils off)      E   enable again
// P      monitor on/off (target, Uq, velocity, angle)
// R      report loop rate
// M...   SimpleFOC's built-in motor commands, used for tuning:
//          MVP0.2  velocity P    MVI5  velocity I    MVF0.01  velocity filter Tf
//          MAP10   angle P       MLV5  velocity limit (rad/s)    MLU2.5  voltage limit
//          M       (alone) prints all settings

void onTarget(char* cmd) { command.scalar(&motor.target, cmd); }

void onMotor(char* cmd) { command.motor(&motor, cmd); }

void onMode(char* cmd) {
#if TEST_OPEN_LOOP
  Serial.println("Mode switching needs the closed-loop build (TEST_OPEN_LOOP 0)");
#else
  int m = atoi(cmd);
  // Set a safe target BEFORE switching, so the new loop doesn't inherit the old
  // target in the wrong units (1 V of torque would become 1 rad of angle).
  // Clear the PID memory too, so stale integral from the last mode doesn't kick.
  motor.PID_velocity.reset();
  motor.P_angle.reset();
  switch (m) {
    case 0:
      motor.target = 0;
      motor.controller = MotionControlType::torque;
      Serial.println("Mode: torque (voltage). T = volts, start with T0.5");
      break;
    case 1:
      motor.target = 0;
      motor.controller = MotionControlType::velocity;
      Serial.println("Mode: velocity. T = rad/s, start with T2");
      break;
    case 2:
      motor.target = motor.shaft_angle;   // hold where it is now, no jump
      motor.controller = MotionControlType::angle;
      Serial.print("Mode: angle. Holding "); Serial.print(motor.shaft_angle, 3);
      Serial.println(" rad. T = absolute angle in rad");
      break;
    default:
      Serial.println("C0 torque, C1 velocity, C2 angle");
  }
#endif
}

void onDisable(char*) { motor.disable(); Serial.println("Driver disabled"); }
void onEnable(char*)  { motor.enable();  Serial.println("Driver enabled"); }

void onMonitor(char*) {
  // monitor_downsample = 0 turns monitoring off. Each print stalls the loop a few ms,
  // so keep it slow: one line every MON_EVERY loops.
  const uint16_t MON_EVERY = 1000;
  motor.monitor_downsample = motor.monitor_downsample ? 0 : MON_EVERY;
  Serial.println(motor.monitor_downsample ? "Monitor on" : "Monitor off");
}

void onRate(char*) {
  uint32_t now = millis();
  Serial.print("Loop rate: ");
  Serial.print(loopCount * 1000UL / (now - loopWindowStart));
  Serial.println(" Hz");
  loopCount = 0;
  loopWindowStart = now;
}

// =====================================================================
// Setup
// =====================================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  SimpleFOCDebug::enable(&Serial);   // init messages and errors from SimpleFOC

  // ---------- Sensor ----------
#if !TEST_OPEN_LOOP
  sensor.clock_speed = 1000000;      // 1 MHz, same as foc_sensor_test (jumper wiring)
  sensor.init();
  motor.linkSensor(&sensor);
#endif

  // ---------- Driver ----------
  driver.voltage_power_supply = SUPPLY_EFFECTIVE;
  driver.voltage_limit        = SUPPLY_EFFECTIVE;
  driver.pwm_frequency        = 20000;   // inaudible, still slow enough for the L298N's switching
  if (!driver.init()) {
    Serial.println("Driver init FAILED. Check the PWM pins in the debug output above.");
    while (1);
  }
  motor.linkDriver(&driver);

  // ---------- Limits ----------
  motor.voltage_limit        = V_LIMIT;
  motor.voltage_sensor_align = V_ALIGN;
  motor.velocity_limit       = 5;        // rad/s, caps the angle loop's speed (~48 rpm)

  // ---------- Controller ----------
#if TEST_OPEN_LOOP
  motor.controller    = MotionControlType::velocity_openloop;
  motor.voltage_limit = V_OPEN_LOOP;    // open loop applies this voltage all the time
#else
  motor.controller = MotionControlType::torque;   // start in the simplest closed-loop mode
  motor.torque_controller = TorqueControlType::voltage;   // no current sensing on the L298N

  // Starting gains. Tune these live with MVP / MVI / MVF / MAP.
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 5.0f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 1000;  // V/s, limits how fast the voltage command can jump
  motor.LPF_velocity.Tf = 0.01f;          // s, velocity filter. Larger = smoother but laggier
  motor.P_angle.P = 10.0f;
#endif

  // ---------- Monitoring (off until 'P') ----------
  motor.useMonitoring(Serial);
  motor.monitor_variables  = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
  motor.monitor_downsample = 0;

  motor.init();
  motor.target = 0;

  // ---------- Wait for the motor supply ----------
  // initFOC moves the rotor, so the bench supply has to be on before it runs.
  Serial.println();
  Serial.println("Bench supply: 6.0 V, 1.5 A limit. Turn it ON now, then press Enter.");
  while (!Serial.available()) {}
  while (Serial.available()) Serial.read();   // drop the Enter

#if !TEST_OPEN_LOOP
  // Alignment: applies V_ALIGN at known electrical angles, watches where the encoder
  // says the rotor went, and works out sensor direction and the zero electrical angle.
  // The shaft WILL twitch and turn a little. Hands off.
  if (!motor.initFOC()) {
    Serial.println("initFOC FAILED. Driver disabled. Send the debug output above.");
    motor.disable();
    while (1);
  }
  // Note these. Once they're stable we can hardcode them and skip alignment at boot.
  Serial.print("sensor_direction = ");    Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
  Serial.print("zero_electric_angle = "); Serial.println(motor.zero_electric_angle, 4);
#endif

  command.add('T', onTarget,  "target");
  command.add('C', onMode,    "mode 0/1/2");
  command.add('X', onDisable, "disable");
  command.add('E', onEnable,  "enable");
  command.add('P', onMonitor, "monitor on/off");
  command.add('R', onRate,    "loop rate");
  command.add('M', onMotor,   "motor settings");

#if TEST_OPEN_LOOP
  Serial.println("Open-loop velocity ready. Try T2 (rad/s). X to stop.");
#else
  Serial.println("Closed loop ready, torque mode, target 0 V. Try T0.5. X to stop.");
#endif
  loopWindowStart = millis();
}

// =====================================================================
// Loop
// =====================================================================

void loop() {
  motor.loopFOC();   // read the angle, turn the voltage command into PWM at the right electrical angle
  motor.move();      // outer loops: angle -> velocity -> voltage, using motor.target
  command.run();     // serial commands
  motor.monitor();   // prints only when 'P' turned it on
  loopCount++;
}