// Motor + encoder test: step the stepper open-loop and compare the steps
// we COMMANDED with the rotation the AS5047P actually MEASURED.
//
// Board:   NUCLEO-G474RE, PlatformIO + Arduino (STM32duino)
// Driver:  L298N (working module). Supply ~3.4 V, 2 A limit, as before
// Encoder: CSn D10, MOSI D11, MISO D12, CLK D13, 3V3, GND
// Power order: Nucleo first, then motor supply. Off in reverse.
//
// What to look for:
//   err stays small and bounded  -> every commanded step happened
//   err jumps by ~1 step's worth -> a missed (or extra) step
//   err grows steadily           -> encoder direction flipped (press 'i') or stalling

#include <Arduino.h>
#include <AS5047P.h>

// ---------- Pins ----------
const int PIN_IN1 = D3;   // coil A (GRN/BLK) direction
const int PIN_IN2 = D5;
const int PIN_IN3 = D6;   // coil B (RED/BLU) direction
const int PIN_IN4 = D9;
const int PIN_ENA = D2;   // coil A on/off (moved off D10/D11 to free SPI1)
const int PIN_ENB = D4;   // coil B on/off
const int PIN_ENC_CS = D10;

// ---------- Motor + encoder constants ----------
const int   STEPS_PER_REV   = 200;  // 1.8 deg full steps
const float COUNTS_PER_STEP = (float)AS5047P::COUNTS_PER_REV / STEPS_PER_REV;

// ---------- Step tables ----------
// Each row = one step. Columns = {IN1, IN2, IN3, IN4}.
const uint8_t WAVE[4][4] = {
  {HIGH, LOW,  LOW,  LOW },   // A+     (0 deg electrical)
  {LOW,  LOW,  HIGH, LOW },   // B+     (90 deg)
  {LOW,  HIGH, LOW,  LOW },   // A-     (180 deg)
  {LOW,  LOW,  LOW,  HIGH}    // B-     (270 deg)
};

const uint8_t TWO_PHASE[4][4] = {
  {HIGH, LOW,  HIGH, LOW },   // A+ B+  (45 deg)
  {LOW,  HIGH, HIGH, LOW },   // A- B+  (135 deg)
  {LOW,  HIGH, LOW,  HIGH},   // A- B-  (225 deg)
  {HIGH, LOW,  LOW,  HIGH}    // A+ B-  (315 deg)
};

// ---------- Timing ----------
const unsigned long INTERVAL_START = 20000UL;  // 50 steps/s
const unsigned long INTERVAL_MIN   = 2000UL;   // 500 steps/s
const unsigned long INTERVAL_MAX   = 50000UL;  // 20 steps/s
const unsigned long INTERVAL_STEP  = 1000UL;
const unsigned long ENC_PERIOD_US  = 1000UL;   // sample encoder at 1 kHz
const unsigned long PRINT_PERIOD_MS = 100UL;   // telemetry at 10 Hz while running
const unsigned long SETTLE_MS      = 300UL;    // let the rotor settle after energizing

// ---------- Motor state ----------
int  activeTable  = 0;       // 0 = wave, 1 = two-phase
int  currentIndex = 0;       // current row in the table, 0-3
int  direction    = 1;       // +1 forward, -1 reverse
bool isRunning    = false;
bool coilsOn      = false;
unsigned long stepInterval = INTERVAL_START;
unsigned long lastStepTime = 0;

// ---------- Encoder state ----------
AS5047P enc(PIN_ENC_CS);
AS5047P::Sample lastSample;
uint16_t lastRaw  = 0;
bool     haveLast = false;   // false until the first good sample
int      encDir   = 1;       // +1 or -1, flip with 'i' if counts run backwards

// ---------- Tracking (all relative to the last zero) ----------
long  stepCount = 0;         // full steps commanded
long  encCounts = 0;         // encoder counts measured, unwrapped across 0/16383
float maxAbsErr = 0;         // worst |error| seen, in counts

// ---------- Error counters, one per AS5047P::Status ----------
uint32_t statusCount[(uint8_t)AS5047P::Status::Count] = {0};

unsigned long lastEncTime   = 0;
unsigned long lastPrintTime = 0;

// =====================================================================
// Motor
// =====================================================================

void applyStep(int idx) {
  const uint8_t (*table)[4] = (activeTable == 0) ? WAVE : TWO_PHASE;
  digitalWrite(PIN_IN1, table[idx][0]);
  digitalWrite(PIN_IN2, table[idx][1]);
  digitalWrite(PIN_IN3, table[idx][2]);
  digitalWrite(PIN_IN4, table[idx][3]);
}

void coilsOff() {
  digitalWrite(PIN_ENA, LOW);   // disable bridges first so outputs float
  digitalWrite(PIN_ENB, LOW);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  coilsOn = false;
}

// Advance one full step and count it
void stepOnce() {
  currentIndex = (currentIndex + direction + 4) % 4;
  applyStep(currentIndex);
  stepCount += direction;
}

// =====================================================================
// Encoder
// =====================================================================

// Read the encoder and add the change since the last good sample to encCounts.
// Wrap handling: a jump of more than half a turn between samples is really
// the angle crossing 16383 -> 0 (or back). At 1 kHz and 500 steps/s the shaft
// moves ~0.9 deg per sample, so real motion never gets close to half a turn.
void updateEncoder() {
  AS5047P::Sample s;
  AS5047P::Status st = enc.read(s);
  statusCount[(uint8_t)st]++;
  if (st != AS5047P::Status::Ok) return;  // bad sample: skip it, keep the last good one

  lastSample = s;
  // No usable magnet field: the angle is noise, so don't add it up.
  // Reset haveLast so tracking restarts cleanly once the field is back.
  if (s.magl() || s.magh()) {
    haveLast = false;
    return;
  }
  if (!haveLast) {
    lastRaw  = s.raw;
    haveLast = true;
    return;
  }

  int32_t delta = (int32_t)s.raw - (int32_t)lastRaw;
  const int32_t HALF = AS5047P::COUNTS_PER_REV / 2;
  if (delta >  HALF) delta -= AS5047P::COUNTS_PER_REV;
  if (delta < -HALF) delta += AS5047P::COUNTS_PER_REV;

  encCounts += encDir * delta;
  lastRaw = s.raw;
}

// Measured minus expected, in encoder counts
float positionError() {
  return encCounts - stepCount * COUNTS_PER_STEP;
}

void zeroTracking() {
  stepCount = 0;
  encCounts = 0;
  maxAbsErr = 0;
  Serial.println("Zeroed: steps and encoder counts reset");
}

// Energize the current step if the coils are off, wait for the rotor to
// snap into place, then zero. Without this, the snap (up to 2 full steps)
// would show up as a fake error.
void energizeAndZero() {
  if (coilsOn) return;
  digitalWrite(PIN_ENA, HIGH);
  digitalWrite(PIN_ENB, HIGH);
  applyStep(currentIndex);
  coilsOn = true;

  // Blocking is fine here: it only happens once, before any stepping.
  // Keep sampling during the wait so the snap is tracked.
  unsigned long t0 = millis();
  while (millis() - t0 < SETTLE_MS) {
    updateEncoder();
    delay(1);
  }
  zeroTracking();
}

// =====================================================================
// Printing
// =====================================================================

void printStatus() {
  Serial.print(isRunning ? "RUN" : "STOP");
  Serial.print(" | mode: ");  Serial.print(activeTable == 0 ? "wave" : "two-phase");
  Serial.print(" | dir: ");   Serial.print(direction > 0 ? "fwd" : "rev");
  Serial.print(" | ");        Serial.print(1000000UL / stepInterval);
  Serial.print(" steps/s | encDir: "); Serial.println(encDir);
}

void printTelemetry() {
  float err = positionError();
  if (fabs(err) > maxAbsErr) maxAbsErr = fabs(err);

  Serial.print("steps=");    Serial.print(stepCount);
  Serial.print(" enc=");     Serial.print(encCounts);
  Serial.print(" expect=");  Serial.print(stepCount * COUNTS_PER_STEP, 1);
  Serial.print(" err=");     Serial.print(err, 1);
  Serial.print(" (");        Serial.print(err / COUNTS_PER_STEP, 2);
  Serial.print(" steps) maxErr="); Serial.print(maxAbsErr, 1);
  Serial.print(" | raw=");   Serial.print(lastSample.raw);
  Serial.print(" AGC=");     Serial.print(lastSample.agc());
  Serial.print(" MAGL=");    Serial.print(lastSample.magl());
  Serial.print(" MAGH=");    Serial.print(lastSample.magh());

  // Failures since boot. Order matches AS5047P::Status
  Serial.print(" | noResp=");  Serial.print(statusCount[(uint8_t)AS5047P::Status::NoResponse]);
  Serial.print(" parity=");    Serial.print(statusCount[(uint8_t)AS5047P::Status::ParityError]);
  Serial.print(" chipErr=");   Serial.print(statusCount[(uint8_t)AS5047P::Status::ChipError]);
  Serial.print(" cof=");       Serial.println(statusCount[(uint8_t)AS5047P::Status::CordicOverflow]);
}

// =====================================================================
// Setup + loop
// =====================================================================

void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  coilsOff();  // everything off before the motor supply comes on

  enc.begin();

  Serial.println("Motor + encoder test ready.");
  Serial.println("g=go s=stop n=single step  w=wave t=two-phase  +/-=speed r=reverse");
  Serial.println("z=zero  i=invert encoder dir  p=print");
  printStatus();
}

void loop() {
  // ---------- Serial commands ----------
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'g':
        if (!isRunning) {
          energizeAndZero();          // no-op if coils are already on
          lastStepTime = micros();    // first step one interval from now
          isRunning = true;
        }
        printStatus();
        break;

      case 's':
        isRunning = false;
        coilsOff();
        printStatus();
        printTelemetry();
        break;

      case 'n':   // single step and HOLD, then report where the shaft ended up
        isRunning = false;
        energizeAndZero();
        stepOnce();
        delay(SETTLE_MS);             // fine to block: we're stopped
        updateEncoder();
        printTelemetry();
        break;

      case 'w':
      case 't':
        // Switching pattern moves the rotor half a step (two-phase sits
        // between wave positions), so re-zero after it settles.
        activeTable = (cmd == 'w') ? 0 : 1;
        if (coilsOn) {
          applyStep(currentIndex);
          Serial.println("Mode changed: rotor moved half a step, press z once it settles");
        }
        printStatus();
        break;

      case '+':
        if (stepInterval > INTERVAL_MIN + INTERVAL_STEP) stepInterval -= INTERVAL_STEP;
        else stepInterval = INTERVAL_MIN;
        printStatus();
        break;

      case '-':
        stepInterval += INTERVAL_STEP;
        if (stepInterval > INTERVAL_MAX) stepInterval = INTERVAL_MAX;
        printStatus();
        break;

      case 'r':
        direction = -direction;
        printStatus();
        break;

      case 'z':
        zeroTracking();
        break;

      case 'i':
        encDir = -encDir;
        zeroTracking();               // old counts were summed with the wrong sign
        printStatus();
        break;

      case 'p':
        printStatus();
        printTelemetry();
        break;

      default:
        break;   // ignore newlines and unknown keys
    }
  }

  // ---------- Step timer ----------
  if (isRunning && (micros() - lastStepTime >= stepInterval)) {
    lastStepTime = micros();
    stepOnce();
  }

  // ---------- Encoder sampling, 1 kHz, even when stopped ----------
  if (micros() - lastEncTime >= ENC_PERIOD_US) {
    lastEncTime = micros();
    updateEncoder();
  }

  // ---------- Telemetry, 10 Hz while running ----------
  if (isRunning && (millis() - lastPrintTime >= PRINT_PERIOD_MS)) {
    lastPrintTime = millis();
    printTelemetry();
  }
}