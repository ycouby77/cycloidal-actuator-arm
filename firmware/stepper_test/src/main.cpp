#include <Arduino.h>

// How often to toggle the LED and print, in milliseconds
const unsigned long BLINK_INTERVAL_MS = 1000;

unsigned long lastToggle = 0;   // time of the last toggle
bool ledOn = false;             // current LED state
unsigned long count = 0;        // number of blinks so far

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // green user LED (LD2) on the Nucleo
  Serial.begin(115200);          // must match monitor_speed in platformio.ini
  delay(500);                    // give the serial monitor a moment to connect
  Serial.println("Nucleo G474RE alive");
}

void loop() {
  unsigned long now = millis();  // ms since boot

  // Non-blocking timing: only act once the interval has passed.
  // Using millis() instead of delay() keeps loop() free for other work later,
  // which matters once you're stepping a motor in here too.
  if (now - lastToggle >= BLINK_INTERVAL_MS) {
    lastToggle = now;
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);

    if (ledOn) {
      count++;
      Serial.print("Blink #");
      Serial.print(count);
      Serial.print("  uptime: ");
      Serial.print(now / 1000);
      Serial.println(" s");
    }
  }
}