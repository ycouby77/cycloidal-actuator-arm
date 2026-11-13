// foc_sensor_test
// Step 1 of closed loop: read the AS5047P through SimpleFOC's sensor class
// (no motor control yet) and prove the angle is clean before FOC depends on it.
//
// Board:   NUCLEO-G474RE, PlatformIO + Arduino (STM32duino)
// Encoder: AS5047P board 2 (3.3 V mode), CSn D10, MOSI D11, MISO D12, CLK D13
// Driver:  L298N is held DISABLED here (EN pins low). Motor supply can stay off.
//
// Printout at 20 Hz, 115200 baud:
//   angle    : total angle in rad. About +/-6.28 per full turn, keeps counting past 1 turn
//   mech     : angle within one turn, 0 to 6.28, wraps
//   vel      : rad/s, averaged over the print window. ~0 at rest
//   spread   : peak-to-apeak angle over the print window, in encoder counts.
//              Hold the shaft still: this is your noise floor. Expect ~0-2
//   glitches : running total of impossible jumps between two reads (bad SPI frames).
//              Should stay at 0. Wiggle the jumpers and watch it
//   loop_hz  : how many sensor reads per second (tells you SPI time per read)

#include <Arduino.h>
#include <SimpleFOC.h>

// ---------- Pins ----------
const int PIN_ENC_CS = D10;
const int PIN_IN1 = D3, PIN_IN2 = D5;   // coil A (GRN/BLK)
const int PIN_IN3 = D6, PIN_IN4 = D9;   // coil B (RED/BLU)
const int PIN_ENA = D2, PIN_ENB = D4;

// ---------- Sensor ----------
// AS5047_SPI is SimpleFOC's built-in config for this chip:
// 14-bit angle, SPI mode 1, angle register 0x3FFF
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, PIN_ENC_CS);

// ---------- Test settings ----------
const float    COUNTS_PER_RAD  = 16384.0f / _2PI;   // 14 bits per turn
const uint32_t PRINT_PERIOD_MS = 50;                // 20 Hz printout

// Hand-turning moves far less than 1 count between two reads at these loop rates,
// so a jump bigger than ~52 counts (0.02 rad) in one read can only be a bad frame.
const float GLITCH_RAD = 0.02f;

// ---------- Stats ----------
float    lastAngle = 0;
float    winMin = 0, winMax = 0;   // min/max angle inside the current print window
uint32_t glitches  = 0;            // running total, never reset
uint32_t loops     = 0;            // reads in the current print window
uint32_t lastPrint = 0;

void setup() {
  // Keep the L298N off. This sketch never drives the motor.
  pinMode(PIN_ENA, OUTPUT); digitalWrite(PIN_ENA, LOW);
  pinMode(PIN_ENB, OUTPUT); digitalWrite(PIN_ENB, LOW);
  pinMode(PIN_IN1, OUTPUT); digitalWrite(PIN_IN1, LOW);
  pinMode(PIN_IN2, OUTPUT); digitalWrite(PIN_IN2, LOW);
  pinMode(PIN_IN3, OUTPUT); digitalWrite(PIN_IN3, LOW);
  pinMode(PIN_IN4, OUTPUT); digitalWrite(PIN_IN4, LOW);

  Serial.begin(115200);
  delay(500);
  SimpleFOCDebug::enable(&Serial);   // SimpleFOC prints init info and errors here

  sensor.clock_speed = 1000000;      // 1 MHz: forgiving on jumper wires (chip max is 10 MHz)
  sensor.init();                     // uses the default SPI (SPI1 on D11/D12/D13)

  // Seed the stats with a first reading
  sensor.update();
  lastAngle = sensor.getAngle();
  winMin = winMax = lastAngle;
  lastPrint = millis();

  Serial.println("foc_sensor_test ready");
}

void loop() {
  // One SPI read. SimpleFOC also tracks full rotations here, which is why
  // getAngle() keeps counting past 2*pi instead of wrapping.
  sensor.update();
  float a = sensor.getAngle();

  // Glitch check: an impossible jump between two consecutive reads.
  // Note: a single bad frame usually shows up as 2 glitches (jump out, jump back).
  if (fabsf(a - lastAngle) > GLITCH_RAD) glitches++;
  lastAngle = a;

  // Track min/max for the noise spread
  if (a < winMin) winMin = a;
  if (a > winMax) winMax = a;
  loops++;

  // Print at 20 Hz. Printing every loop would slow the loop and flood the monitor.
  uint32_t now = millis();
  if (now - lastPrint >= PRINT_PERIOD_MS) {
    float spreadCounts = (winMax - winMin) * COUNTS_PER_RAD;
    uint32_t loopHz    = loops * 1000UL / (now - lastPrint);

    Serial.print("angle ");      Serial.print(a, 4);
    Serial.print("  mech ");     Serial.print(sensor.getMechanicalAngle(), 4);
    Serial.print("  vel ");      Serial.print(sensor.getVelocity(), 3);
    Serial.print("  spread ");   Serial.print(spreadCounts, 1);
    Serial.print("  glitches "); Serial.print(glitches);
    Serial.print("  loop_hz ");  Serial.println(loopHz);

    // Start a new window
    winMin = winMax = a;
    loops = 0;
    lastPrint = now;
    sensor.update();
    lastAngle = sensor.getAngle();
  }
}