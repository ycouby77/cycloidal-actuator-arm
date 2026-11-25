// actuator
// Firmware for one cycloidal actuator: SimpleFOC closed loop on the 17HS19-2004D,
// through the 25:1 cycloid, controlled with a line-based serial protocol.
// Full command reference: docs/protocol.md. The GUI in host/actuator_gui speaks it.
//
// Units on the wire are OUTPUT side (after the gearbox):
//   position  deg     velocity  deg/s     torque mode  volts on the q axis
// Positive always means the same output direction in every mode.
// The firmware converts to motor-side radians for SimpleFOC.
//
// Board:   NUCLEO-G474RE, PlatformIO + Arduino (STM32duino core 2.12.0, see platformio.ini)
// Driver:  L298N, IN1-IN4 on D3/D5/D6/D9, ENA/ENB on D2/D4
// Encoder: AS5047P (3.3 V mode), CSn D10, MOSI D11, MISO D12, CLK D13
//
// Supply option B: bench supply ~6.0 V, 1.5 A limit. The L298N drops ~2 V, so SimpleFOC
// is told the supply is 4 V. Commanded voltage ~= voltage the coil actually sees.
//
// Boot is safe: nothing moves until the host sends "init" (turn the supply on first).
// After init the driver is DISABLED. The host has to send "enable" explicitly.

#include <Arduino.h>
#include <SimpleFOC.h>

// =====================================================================
// Hardware constants
// =====================================================================

const int PIN_IN1 = D3, PIN_IN2 = D5;   // coil A (GRN/BLK)
const int PIN_IN3 = D6, PIN_IN4 = D9;   // coil B (RED/BLU)
const int PIN_ENA = D2, PIN_ENB = D4;
const int PIN_ENC_CS = D10;

const uint32_t BAUD = 460800;           // must match monitor_speed and the GUI

const float SUPPLY_EFFECTIVE = 4.0f;    // 6 V bench - ~2 V L298N drop
const float V_LIMIT_DEFAULT  = 2.5f;    // 2.5 V / 1.48 ohm ~ 1.7 A per coil
const float V_LIMIT_MAX      = 2.8f;    // hard cap: 2.8 / 1.48 ~ 1.9 A, under the 2 A rating
const float V_ALIGN_DEFAULT  = 2.0f;    // initFOC alignment voltage. Raise if init fails with the gearbox on

// Gearbox. Fixed ring pins, output taken from the discs through the pins:
// output turns 1/(lobes) per input turn, in the OPPOSITE direction. Confirm with the tape test.
const float GEAR_RATIO_DEFAULT = 25.0f;
const int8_t GEAR_DIR_DEFAULT  = -1;

// =====================================================================
// SimpleFOC objects
// =====================================================================

// 1.8 deg stepper = 200 steps/rev = 50 electrical cycles per rev = 50 pole pairs
StepperMotor      motor  = StepperMotor(50);
StepperDriver4PWM driver = StepperDriver4PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENA, PIN_ENB);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, PIN_ENC_CS);

// =====================================================================
// Actuator state (everything the protocol can see or change)
// =====================================================================

enum class Mode : uint8_t { Torque = 0, Velocity = 1, Position = 2 };

struct State {
  bool     initialized = false;
  Mode     mode        = Mode::Torque;
  float    target      = 0.0f;              // output units for the current mode
  float    zeroDeg     = 0.0f;              // output angle that reads as 0 after "zero"
  float    ratio       = GEAR_RATIO_DEFAULT;
  int8_t   dir         = GEAR_DIR_DEFAULT;
  float    outVelLimit = 12.0f;             // deg/s at the output (= 5.2 rad/s at the motor)

  uint32_t wdTimeoutMs = 0;                 // 0 = watchdog off. GUI turns it on
  uint32_t lastRxMs    = 0;                 // last complete command line
  bool     wdTripped   = false;

  uint16_t streamHz    = 0;                 // 0 = no telemetry
  uint32_t lastStreamUs = 0;
  uint32_t droppedLines = 0;                // telemetry lines skipped because the TX buffer was full
} st;

// Loop rate, measured over 1 s windows
uint32_t loopCount = 0, loopWindowStart = 0, loopHz = 0;

// =====================================================================
// Unit conversion: output side <-> motor side
// =====================================================================
// out_rad = dir * motor_rad / ratio, then shift by the zero offset.

float outPosDeg() { return degrees(st.dir * motor.shaft_angle / st.ratio) - st.zeroDeg; }
float outVelDps() { return degrees(st.dir * motor.shaft_velocity / st.ratio); }
float outUq()     { return st.dir * motor.voltage.q; }   // sign matches torque-mode targets

const char* modeName(Mode m) {
  switch (m) {
    case Mode::Torque:   return "torque";
    case Mode::Velocity: return "vel";
    case Mode::Position: return "pos";
  }
  return "?";
}

// Push st.target (output units) into motor.target (motor units), clamped to the limits.
void applyTarget() {
  switch (st.mode) {
    case Mode::Torque:
      st.target    = constrain(st.target, -motor.voltage_limit, motor.voltage_limit);
      motor.target = st.dir * st.target;
      break;
    case Mode::Velocity:
      st.target    = constrain(st.target, -st.outVelLimit, st.outVelLimit);
      motor.target = st.dir * st.ratio * radians(st.target);
      break;
    case Mode::Position:
      motor.target = st.dir * st.ratio * radians(st.target + st.zeroDeg);
      break;
  }
}

// Velocity limit lives in two places in SimpleFOC: the angle loop's output clamp
// and motor.velocity_limit. Keep them in sync.
void applyVelLimit(float dps) {
  st.outVelLimit       = constrain(dps, 0.5f, 60.0f);
  motor.velocity_limit = st.ratio * radians(st.outVelLimit);
  motor.P_angle.limit  = motor.velocity_limit;
  if (st.mode == Mode::Velocity) applyTarget();   // re-clamp the running target
}

// Same for the voltage limit: motor.voltage_limit and the velocity PID's output clamp.
void applyVoltLimit(float v) {
  motor.voltage_limit      = constrain(v, 0.1f, V_LIMIT_MAX);
  motor.PID_velocity.limit = motor.voltage_limit;
  if (st.mode == Mode::Torque) applyTarget();
}

// Switch modes with a safe target, so the new loop never inherits a number in the
// wrong units (1 V of torque would become 1 deg of position). Clears PID memory too.
void setMode(Mode m) {
  motor.PID_velocity.reset();
  motor.P_angle.reset();
  st.mode = m;
  switch (m) {
    case Mode::Torque:   motor.controller = MotionControlType::torque;   st.target = 0;           break;
    case Mode::Velocity: motor.controller = MotionControlType::velocity; st.target = 0;           break;
    case Mode::Position: motor.controller = MotionControlType::angle;    st.target = outPosDeg(); break;  // hold here
  }
  applyTarget();
}

// =====================================================================
// Replies. Every command gets exactly ONE "ok ..." or "err ..." line.
// "#" lines are info for humans; "d," lines are telemetry.
// =====================================================================

void replyOk(const char* cmd)  { Serial.print("ok "); Serial.println(cmd); }
void replyErr(const char* cmd, const char* why) {
  Serial.print("err "); Serial.print(cmd); Serial.print(' '); Serial.println(why);
}
void printKV(const char* key, float v, uint8_t digits = 4) {
  Serial.print(' '); Serial.print(key); Serial.print('='); Serial.print(v, digits);
}
void printKV(const char* key, const char* v) {
  Serial.print(' '); Serial.print(key); Serial.print('='); Serial.print(v);
}
void printKV(const char* key, long v) {
  Serial.print(' '); Serial.print(key); Serial.print('='); Serial.print(v);
}

// =====================================================================
// Tunable parameters, by name. "param <name> [value]"
// =====================================================================

struct Param {
  const char* name;
  float (*get)();
  void  (*set)(float);
  const char* help;
};

const Param PARAMS[] = {
  {"vp",     [] { return motor.PID_velocity.P; },           [](float v) { motor.PID_velocity.P = max(v, 0.0f); },  "velocity P (V per motor rad/s)"},
  {"vi",     [] { return motor.PID_velocity.I; },           [](float v) { motor.PID_velocity.I = max(v, 0.0f); },  "velocity I"},
  {"vd",     [] { return motor.PID_velocity.D; },           [](float v) { motor.PID_velocity.D = max(v, 0.0f); },  "velocity D"},
  {"vf",     [] { return motor.LPF_velocity.Tf; },          [](float v) { motor.LPF_velocity.Tf = constrain(v, 0.0f, 0.2f); }, "velocity filter Tf (s)"},
  {"ramp",   [] { return motor.PID_velocity.output_ramp; }, [](float v) { motor.PID_velocity.output_ramp = max(v, 0.0f); }, "voltage ramp limit (V/s), 0 = off"},
  {"ap",     [] { return motor.P_angle.P; },                [](float v) { motor.P_angle.P = max(v, 0.0f); },       "angle P (motor rad/s per rad)"},
  {"ulim",   [] { return motor.voltage_limit; },            [](float v) { applyVoltLimit(v); },                    "voltage limit (V), max 2.8"},
  {"vlim",   [] { return st.outVelLimit; },                 [](float v) { applyVelLimit(v); },                     "output velocity limit (deg/s)"},
  {"valign", [] { return motor.voltage_sensor_align; },     [](float v) { motor.voltage_sensor_align = constrain(v, 0.5f, V_LIMIT_MAX); }, "initFOC alignment voltage (V)"},
  {"ratio",  [] { return st.ratio; },                       [](float v) { if (v >= 1) { st.ratio = v; applyVelLimit(st.outVelLimit); } }, "gear ratio"},
  {"dir",    [] { return (float)st.dir; },                  [](float v) { st.dir = (v < 0) ? -1 : 1; applyTarget(); }, "output direction vs motor, +1 or -1"},
};
const size_t N_PARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);

const Param* findParam(const char* name) {
  for (size_t i = 0; i < N_PARAMS; i++)
    if (strcmp(PARAMS[i].name, name) == 0) return &PARAMS[i];
  return nullptr;
}

// =====================================================================
// Command handlers
// =====================================================================

// Parse a float strictly: "12.5" ok, "12x" or "" rejected.
bool parseFloat(const char* s, float& out) {
  if (!s || !*s) return false;
  char* end;
  out = strtof(s, &end);
  return *end == '\0';
}

void cmdInit() {
  if (st.initialized) { Serial.println("ok init already=1"); return; }

  // Clear any half-finished calibration from a failed attempt, so alignment reruns fully.
  motor.sensor_direction    = Direction::UNKNOWN;
  motor.zero_electric_angle = NOT_SET;
  motor.enable();

  Serial.println("# initFOC: the shaft will twitch. Hands off.");
  if (!motor.initFOC()) {
    motor.disable();
    replyErr("init", "initfoc_failed (supply on? try a higher valign)");
    return;
  }
  st.initialized = true;
  setMode(Mode::Torque);
  st.zeroDeg = outPosDeg() + st.zeroDeg;   // power-up position reads as 0
  motor.disable();                        // host must "enable" on purpose

  Serial.print("ok init");
  printKV("sensor_dir", motor.sensor_direction == Direction::CW ? "cw" : "ccw");
  printKV("zero_el", motor.zero_electric_angle);
  Serial.println();
}

void cmdEnable() {
  if (!st.initialized) { replyErr("enable", "not_initialized"); return; }
  setMode(st.mode);   // fresh safe target: hold position, or 0 V / 0 deg/s
  motor.enable();
  st.wdTripped = false;
  replyOk("enable");
}

void cmdDisable() {
  motor.disable();
  replyOk("disable");
}

void cmdMode(const char* arg) {
  if (!st.initialized) { replyErr("mode", "not_initialized"); return; }
  if (!arg)                          { replyErr("mode", "usage: mode torque|vel|pos"); return; }
  if      (!strcmp(arg, "torque"))   setMode(Mode::Torque);
  else if (!strcmp(arg, "vel"))      setMode(Mode::Velocity);
  else if (!strcmp(arg, "pos"))      setMode(Mode::Position);
  else                               { replyErr("mode", "usage: mode torque|vel|pos"); return; }
  Serial.print("ok mode");
  printKV("mode", modeName(st.mode));
  printKV("target", st.target, 3);
  Serial.println();
}

void cmdSet(const char* arg) {
  float v;
  if (!st.initialized)       { replyErr("set", "not_initialized"); return; }
  if (!parseFloat(arg, v))   { replyErr("set", "usage: set <value>"); return; }
  st.target = v;
  applyTarget();   // may clamp
  Serial.print("ok set");
  printKV("target", st.target, 3);   // echo what was actually applied
  Serial.println();
}

void cmdZero() {
  if (!st.initialized) { replyErr("zero", "not_initialized"); return; }
  st.zeroDeg += outPosDeg();   // current position now reads 0
  if (st.mode == Mode::Position) { st.target = 0; applyTarget(); }
  replyOk("zero");
}

void cmdStream(const char* arg) {
  float hz;
  if (!parseFloat(arg, hz) || hz < 0 || hz > 500) { replyErr("stream", "usage: stream <0..500 Hz>"); return; }
  st.streamHz = (uint16_t)hz;
  Serial.print("ok stream"); printKV("hz", (long)st.streamHz); Serial.println();
}

void cmdWatchdog(const char* arg) {
  float ms;
  if (!parseFloat(arg, ms) || ms < 0 || (ms > 0 && ms < 100)) { replyErr("wd", "usage: wd <0 off, or >=100 ms>"); return; }
  st.wdTimeoutMs = (uint32_t)ms;
  st.wdTripped = false;
  Serial.print("ok wd"); printKV("ms", (long)st.wdTimeoutMs); Serial.println();
}

void cmdParam(const char* name, const char* value) {
  if (!name) { replyErr("param", "usage: param <name> [value]"); return; }
  const Param* p = findParam(name);
  if (!p) { replyErr("param", "unknown_name"); return; }
  if (value) {
    float v;
    if (!parseFloat(value, v)) { replyErr("param", "bad_value"); return; }
    p->set(v);
  }
  Serial.print("ok param");
  printKV(p->name, p->get(), 5);   // value after clamping
  Serial.println();
}

void cmdParams() {
  Serial.print("ok params");
  for (size_t i = 0; i < N_PARAMS; i++) printKV(PARAMS[i].name, PARAMS[i].get(), 5);
  Serial.println();
}

void cmdStatus() {
  Serial.print("ok status");
  printKV("init",  (long)st.initialized);
  printKV("en",    (long)motor.enabled);
  printKV("mode",  modeName(st.mode));
  printKV("target", st.target, 3);
  printKV("pos",   st.initialized ? outPosDeg() : 0.0f, 3);
  printKV("vel",   st.initialized ? outVelDps() : 0.0f, 3);
  printKV("uq",    outUq(), 3);
  printKV("wd",    (long)st.wdTimeoutMs);
  printKV("wd_trip", (long)st.wdTripped);
  printKV("stream", (long)st.streamHz);
  printKV("loop_hz", (long)loopHz);
  printKV("dropped", (long)st.droppedLines);
  Serial.println();
}

void cmdHelp() {
  Serial.println("# ping | status | params | init | enable | disable | zero");
  Serial.println("# mode torque|vel|pos | set <v>  (V, deg/s, deg at the output)");
  Serial.println("# stream <hz> | wd <ms> | param <name> [value]");
  for (size_t i = 0; i < N_PARAMS; i++) {
    Serial.print("#   "); Serial.print(PARAMS[i].name); Serial.print(": "); Serial.println(PARAMS[i].help);
  }
  replyOk("help");
}

// One complete line in. Tokens are space-separated, commands are lowercase.
void handleLine(char* line) {
  for (char* p = line; *p; p++) *p = tolower(*p);
  char* cmd  = strtok(line, " \t");
  char* arg1 = strtok(nullptr, " \t");
  char* arg2 = strtok(nullptr, " \t");
  if (!cmd) return;   // blank line

  if      (!strcmp(cmd, "ping"))    Serial.println("ok ping");
  else if (!strcmp(cmd, "status"))  cmdStatus();
  else if (!strcmp(cmd, "params"))  cmdParams();
  else if (!strcmp(cmd, "init"))    cmdInit();
  else if (!strcmp(cmd, "enable"))  cmdEnable();
  else if (!strcmp(cmd, "disable") || !strcmp(cmd, "stop")) cmdDisable();
  else if (!strcmp(cmd, "mode"))    cmdMode(arg1);
  else if (!strcmp(cmd, "set"))     cmdSet(arg1);
  else if (!strcmp(cmd, "zero"))    cmdZero();
  else if (!strcmp(cmd, "stream"))  cmdStream(arg1);
  else if (!strcmp(cmd, "wd"))      cmdWatchdog(arg1);
  else if (!strcmp(cmd, "param"))   cmdParam(arg1, arg2);
  else if (!strcmp(cmd, "help"))    cmdHelp();
  else if (!strcmp(cmd, "rate"))    { Serial.print("ok rate"); printKV("loop_hz", (long)loopHz); Serial.println(); }
  else                              replyErr(cmd, "unknown_command");
}

// =====================================================================
// Serial input: non-blocking line assembly
// =====================================================================

char     lineBuf[96];
uint8_t  lineLen = 0;
bool     lineTooLong = false;

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineTooLong)     Serial.println("err line too_long");
      else if (lineLen)    handleLine(lineBuf);
      // Stamp AFTER handling, so a slow command (init takes seconds) can't trip the watchdog
      st.lastRxMs = millis();
      lineLen = 0;
      lineTooLong = false;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineTooLong = true;   // keep reading until the newline, then reject the whole line
    }
  }
}

// =====================================================================
// Watchdog: if the host goes quiet while the motor is enabled, cut it.
// Covers a crashed GUI, a closed laptop, or a pulled USB cable.
// =====================================================================

void serviceWatchdog() {
  if (st.wdTimeoutMs == 0 || !motor.enabled) return;
  if (millis() - st.lastRxMs > st.wdTimeoutMs) {
    motor.disable();
    st.wdTripped = true;
    Serial.println("# wd_trip: no command from host, driver disabled");
  }
}

// =====================================================================
// Telemetry: d,<t_us>,<mode>,<en>,<target>,<pos_deg>,<vel_dps>,<uq_V>
// =====================================================================

void serviceStream() {
  if (st.streamHz == 0 || !st.initialized) return;
  uint32_t now = micros();
  if (now - st.lastStreamUs < 1000000UL / st.streamHz) return;
  st.lastStreamUs = now;

  // Never block the FOC loop on a full UART: skip the sample instead.
  if (Serial.availableForWrite() < 72) { st.droppedLines++; return; }

  Serial.print("d,");
  Serial.print(now);                 Serial.print(',');
  Serial.print((int)st.mode);        Serial.print(',');
  Serial.print((int)motor.enabled);  Serial.print(',');
  Serial.print(st.target, 3);        Serial.print(',');
  Serial.print(outPosDeg(), 3);      Serial.print(',');
  Serial.print(outVelDps(), 2);      Serial.print(',');
  Serial.println(outUq(), 3);
}

// =====================================================================
// Setup / loop
// =====================================================================

void setup() {
  Serial.begin(BAUD);
  delay(300);
  SimpleFOCDebug::enable(&Serial);   // SimpleFOC prints "MOT: ..." lines during init

  // Sensor
  sensor.clock_speed = 1000000;      // 1 MHz, plenty for jumper wiring
  sensor.init();
  motor.linkSensor(&sensor);

  // Driver
  driver.voltage_power_supply = SUPPLY_EFFECTIVE;
  driver.voltage_limit        = SUPPLY_EFFECTIVE;
  driver.pwm_frequency        = 20000;
  if (!driver.init()) {
    Serial.println("# FATAL: driver init failed, check the PWM pins in the debug output");
    while (1) { pollSerial(); }   // stay responsive so the GUI can see the board
  }
  motor.linkDriver(&driver);

  // Controller: voltage-mode torque (no current sensing on the L298N)
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller        = MotionControlType::torque;
  motor.voltage_sensor_align = V_ALIGN_DEFAULT;

  // Starting gains, same as the bare-motor tests. Tune live with "param".
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 5.0f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 1000;   // V/s
  motor.LPF_velocity.Tf = 0.01f;           // s
  motor.P_angle.P = 10.0f;

  motor.init();            // also enables the driver...
  motor.disable();         // ...so turn it straight back off. Nothing moves before "init".

  // Limits AFTER motor.init(), which overwrites the PID output clamps
  applyVoltLimit(V_LIMIT_DEFAULT);
  applyVelLimit(st.outVelLimit);

  Serial.println("# actuator ready. Supply on, then send: init");
  loopWindowStart = millis();
}

void loop() {
  if (st.initialized) {
    motor.loopFOC();   // angle read + voltage to the coils at the right electrical angle
    motor.move();      // outer loops: angle -> velocity -> voltage
  }
  pollSerial();
  serviceWatchdog();
  serviceStream();

  // Loop rate, 1 s windows
  loopCount++;
  uint32_t now = millis();
  if (now - loopWindowStart >= 1000) {
    loopHz = loopCount * 1000UL / (now - loopWindowStart);
    loopCount = 0;
    loopWindowStart = now;
  }
}
