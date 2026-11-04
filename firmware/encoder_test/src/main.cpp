// AS5047P bring-up: read the compensated angle + magnet diagnostics over SPI1.
// Board: NUCLEO-G474RE, PlatformIO + Arduino (STM32duino)
// Wiring: CSn D10, MOSI D11, MISO D12, CLK D13, 3V3, GND. Stepper supply OFF.

#include <Arduino.h>
#include <SPI.h>

const uint8_t PIN_CS = D10;

// 1 MHz, MSB first, mode 1 (CPOL=0, CPHA=1)
SPISettings encSpi(1000000, MSBFIRST, SPI_MODE1);

// Command frames: bit15 = even parity, bit14 = 1 (read), bits13:0 = address
const uint16_t CMD_READ_ANGLECOM = 0xFFFF;  // 0x3FFF
const uint16_t CMD_READ_DIAAGC   = 0xFFFC;  // 0x3FFC
const uint16_t CMD_READ_ERRFL    = 0x4001;  // 0x0001, reading clears the error flag
const uint16_t CMD_NOP           = 0xC000;  // 0x0000

// Response frame: bit15 = parity, bit14 = error flag, bits13:0 = data
const uint16_t EF_BIT    = 1 << 14;
const uint16_t DATA_MASK = 0x3FFF;

// True if x has an even number of 1s across all 16 bits
bool hasEvenParity(uint16_t x) {
  uint8_t count = 0;
  for (int i = 0; i < 16; i++) {
    count += x & 1;
    x >>= 1;
  }
  return count % 2 == 0;
}

// One 16-bit frame. CSn going high marks the end of the frame.
uint16_t transfer16(uint16_t out) {
  SPI.beginTransaction(encSpi);
  digitalWrite(PIN_CS, LOW);
  uint16_t in = SPI.transfer16(out);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(1);  // minimum CSn high time between frames
  return in;
}

// ERRFL: bit2 PARERR, bit1 INVCOMM, bit0 FRERR. Reading it clears the flag.
uint16_t readAndClearErrors() {
  transfer16(CMD_READ_ERRFL);
  return transfer16(CMD_NOP) & 0x0007;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin();  // SPI1 on D11/D12/D13
  delay(10);    // chip power-up
  readAndClearErrors();
}

bool readRegister(uint16_t cmd, uint16_t &value) {
  transfer16(cmd);
  uint16_t resp = transfer16(CMD_NOP);
  Serial.print("resp=0x"); Serial.println(resp, HEX);  // debug, remove once reads work

  if (!hasEvenParity(resp)) return false;
  if (resp & EF_BIT)        return false;
  value = resp & DATA_MASK;
  return true;
}

void loop() {
  uint16_t raw, diag;
  bool okA = readRegister(CMD_READ_ANGLECOM, raw);
  bool okD = readRegister(CMD_READ_DIAAGC, diag);

  if (okA && okD) {
    Serial.print("raw=");   Serial.print(raw);
    Serial.print(" deg=");  Serial.print(raw * 360.0f / 16384.0f, 2);
    // DIAAGC: bit11 MAGL (field too weak), bit10 MAGH (too strong),
    //         bit9 COF (CORDIC overflow), bits7:0 AGC (higher = weaker field)
    Serial.print(" MAGL="); Serial.print((diag >> 11) & 1);
    Serial.print(" MAGH="); Serial.print((diag >> 10) & 1);
    Serial.print(" COF=");  Serial.print((diag >> 9) & 1);
    Serial.print(" AGC=");  Serial.println(diag & 0xFF);
  } else {
    Serial.print("read error, ERRFL=0b");
    Serial.println(readAndClearErrors(), BIN);
  }
  delay(50);
}