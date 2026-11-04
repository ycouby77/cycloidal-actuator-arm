#include <Arduino.h>

// ---------- 1. Pin config ----------
// Check these D-labels against the silkscreen on the Nucleo when wiring
const int PIN_IN1 = D3;    // coil A (GRN/BLK) direction
const int PIN_IN2 = D5;
const int PIN_IN3 = D6;    // coil B (RED/BLU) direction
const int PIN_IN4 = D9;
const int PIN_ENA = D2;   // coil A on/off (PWM later)
const int PIN_ENB = D42;   // coil B on/off (PWM later)

// ---------- 2. Step tables ----------
// Each row = one step. Columns = {IN1, IN2, IN3, IN4}.
// Wave: one coil on per step -> A+, B+, A-, B-
const uint8_t WAVE[4][4] = {
  {HIGH, LOW,  LOW,  LOW },   // A+        (0 deg electrical)
  {LOW,  LOW,  HIGH, LOW },   // B+        (90 deg)
  {LOW,  HIGH, LOW,  LOW },   // A-        (180 deg)
  {LOW,  LOW,  LOW,  HIGH}    // B-        (270 deg)
};

// Two-phase-on: both coils on per step -> more torque, double current
const uint8_t TWO_PHASE[4][4] = {
  {HIGH, LOW,  HIGH, LOW },   // A+ B+     (45 deg)
  {LOW,  HIGH, HIGH, LOW },   // A- B+     (135 deg)
  {LOW,  HIGH, LOW,  HIGH},   // A- B-     (225 deg)
  {HIGH, LOW,  LOW,  HIGH}    // A+ B-     (315 deg)
};

// ---------- Speed limits ----------
const unsigned long INTERVAL_START = 20000UL;  // 50 steps/s (15 RPM) for first test
const unsigned long INTERVAL_MIN   = 2000UL;   // fastest: 500 steps/s (150 RPM)
const unsigned long INTERVAL_MAX   = 50000UL;  // slowest: 20 steps/s (6 RPM)
const unsigned long INTERVAL_STEP  = 1000UL;   // change per +/- keypress

// ---------- State ----------
int activeTable = 0;                          // 0 = wave, 1 = two-phase
int currentIndex = 0;                         // current step, 0-3
int direction = 1;                            // +1 forward, -1 reverse
unsigned long stepInterval = INTERVAL_START;  // microseconds between steps
unsigned long lastStepTime = 0;               // micros() at the last step
bool isRunning = false;                       // start stopped: nothing moves until 'g'

// ---------- 3. Write one step to the driver ----------
void applyStep(int idx) {
  // Point at whichever table is active, then write that row to IN1-IN4
  const uint8_t (*table)[4] = (activeTable == 0) ? WAVE : TWO_PHASE;

  digitalWrite(PIN_IN1, table[idx][0]);
  digitalWrite(PIN_IN2, table[idx][1]);
  digitalWrite(PIN_IN3, table[idx][2]);
  digitalWrite(PIN_IN4, table[idx][3]);
}

// ---------- Kill all coils ----------
// Used on startup and on the stop command
void coilsOff() {
  digitalWrite(PIN_ENA, LOW);   // disable bridges first so outputs float
  digitalWrite(PIN_ENB, LOW);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

// ---------- Print current state ----------
void printStatus() {
  Serial.print(isRunning ? "RUN" : "STOP");
  Serial.print(" | mode: ");
  Serial.print(activeTable == 0 ? "wave" : "two-phase");
  Serial.print(" | dir: ");
  Serial.print(direction > 0 ? "fwd" : "rev");
  Serial.print(" | ");
  Serial.print(1000000UL / stepInterval);     // steps per second
  Serial.print(" steps/s (");
  Serial.print(stepInterval);
  Serial.println(" us)");
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  // Everything off before the motor supply comes on
  coilsOff();

  Serial.println("Stepper test ready.");
  Serial.println("w=wave  t=two-phase  +=faster  -=slower  r=reverse  g=go  s=stop  p=status");
  printStatus();
}

void loop() {
  // ---------- 5. Serial commands ----------
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'w':
        activeTable = 0;
        if (isRunning) applyStep(currentIndex);   // switch pattern immediately
        printStatus();
        break;

      case 't':
        activeTable = 1;
        if (isRunning) applyStep(currentIndex);
        printStatus();
        break;

      case '+':   // faster = shorter interval
        if (stepInterval > INTERVAL_MIN + INTERVAL_STEP) stepInterval -= INTERVAL_STEP;
        else stepInterval = INTERVAL_MIN;
        printStatus();
        break;

      case '-':   // slower = longer interval
        stepInterval += INTERVAL_STEP;
        if (stepInterval > INTERVAL_MAX) stepInterval = INTERVAL_MAX;
        printStatus();
        break;

      case 'r':
        direction = -direction;
        printStatus();
        break;

      case 'g':
        if (!isRunning) {
          digitalWrite(PIN_ENA, HIGH);   // enable both bridges
          digitalWrite(PIN_ENB, HIGH);
          applyStep(currentIndex);       // energize current step so the rotor locks in place
          lastStepTime = micros();       // first step happens one interval from now
          isRunning = true;
        }
        printStatus();
        break;

      case 's':
        isRunning = false;
        coilsOff();
        printStatus();
        break;

      case 'p':
        printStatus();
        break;

      default:
        break;   // ignore newlines and unknown keys

      case 'n':   // single step and HOLD, for debugging with a multimeter
        isRunning = false;               // stop the timer from stepping
        digitalWrite(PIN_ENA, HIGH);     // bridges on
        digitalWrite(PIN_ENB, HIGH);
        currentIndex = (currentIndex + direction + 4) % 4;
        applyStep(currentIndex);
        Serial.print("Holding step ");
        Serial.println(currentIndex);
        break;
    }
  }

  // ---------- 4. Step timer (non-blocking) ----------
  // Unsigned subtraction stays correct even when micros() overflows (~71 min)
  if (isRunning && (micros() - lastStepTime >= stepInterval)) {
    lastStepTime = micros();

    // +4 keeps the value positive before the modulo:
    //   forward: (3 + 1 + 4) % 4 = 0
    //   reverse: (0 - 1 + 4) % 4 = 3
    currentIndex = (currentIndex + direction + 4) % 4;

    applyStep(currentIndex);
  }
}