// AS5047P bring-up: read the compensated angle + magnet diagnostics over SPI1.
// Board: NUCLEO-G474RE, PlatformIO + Arduino (STM32duino)
// Wiring: CSn D10, MOSI D11, MISO D12, CLK D13, 3V3, GND. Stepper supply OFF.
//
// Every sample is classified as good or as one specific failure, so a loose
// wire shows up as an error instead of a fake angle.

#include <Arduino.h>
#include <SPI.h>

// ---- Debug: set to 1 to print every raw response frame ----
#define DEBUG_FRAMES 0

// ---- Pins ----
const uint8_t PIN_CS = D10;

// ---- SPI: 1 MHz, MSB first, mode 1 (CPOL=0, CPHA=1) ----
SPISettings encSpi(1000000, MSBFIRST, SPI_MODE1);

// ---- Command frames: bit15 = even parity, bit14 = 1 (read), bits13:0 = address ----
const uint16_t CMD_READ_ANGLECOM = 0xFFFF; // addr 0x3FFF
const uint16_t CMD_READ_DIAAGC   = 0xFFFC; // addr 0x3FFC
const uint16_t CMD_READ_ERRFL    = 0x4001; // addr 0x0001, reading it clears the error flag
const uint16_t CMD_NOP           = 0xC000; // read addr 0x0000, just clocks out the last answer

// ---- Response frame layout ----
const uint16_t EF_BIT    = 1 << 14;  // error flag
const uint16_t DATA_MASK = 0x3FFF;   // 14 data bits

// ---- DIAAGC bits ----
const uint16_t DIAG_MAGL = 1 << 11;  // field too weak
const uint16_t DIAG_MAGH = 1 << 10;  // field too strong
const uint16_t DIAG_COF  = 1 << 9;   // CORDIC overflow, angle invalid
const uint16_t DIAG_LF   = 1 << 8;   // offset compensation finished. Always 1 on a booted chip

// ---- Why a read failed ----
enum ReadStatus {
  READ_OK,
  READ_NO_RESPONSE,  // nobody driving MISO: 0xFFFF, or 0x0000 where that's impossible
  READ_PARITY,       // bits flipped on the way back
  READ_EF            // chip answered, but flagged a bad command (check ERRFL)
};

// ---- Error counters, printed on every line so wiggle tests show up as climbing numbers ----
uint32_t nGood = 0, nNoResp = 0, nParity = 0, nEF = 0, nCOF = 0;

// True if x has an even number of 1s across all 16 bits.
bool hasEvenParity(uint16_t x) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 16; i++) {
    count += x & 1;
    x >>= 1;
  }
  return count % 2 == 0;
}

// One SPI frame: CSn low -> 16 clocks -> CSn high.
// CSn going high is how the chip knows the frame ended.
uint16_t transfer16(uint16_t out) {
  SPI.beginTransaction(encSpi);
  digitalWrite(PIN_CS, LOW);
  uint16_t in = SPI.transfer16(out);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(1);  // comfortably covers the minimum CSn high time
  return in;
}

// Pipeline-aware read. The answer to a command arrives in the NEXT frame,
// so send the command, then a NOP to clock the answer out.
// Checks are ordered so the most specific diagnosis wins.
ReadStatus readRegister(uint16_t cmd, uint16_t &value) {
  transfer16(cmd);                      // this response belongs to the previous command: ignore it
  uint16_t resp = transfer16(CMD_NOP);  // this one is the answer to cmd

#if DEBUG_FRAMES
  Serial.print("resp=0x"); Serial.println(resp, HEX);
#endif

  // 0xFFFF passes parity AND has EF set, so without this check it would be
  // misreported as a chip error. It really means MISO is floating high.
  if (resp == 0xFFFF)        return READ_NO_RESPONSE;
  if (!hasEvenParity(resp))  return READ_PARITY;
  if (resp & EF_BIT)         return READ_EF;

  value = resp & DATA_MASK;
  return READ_OK;
}

// Read ERRFL (which also clears it). Returns false if the ERRFL response is
// itself bad, because then its data bits mean nothing (last night's 0b111 and
// today's 0b0 were both this case).
// ERRFL bits: bit2 PARERR (parity), bit1 INVCOMM (bad address), bit0 FRERR (framing)
bool readAndClearErrors(uint16_t &errfl) {
  uint16_t v;
  if (readRegister(CMD_READ_ERRFL, v) != READ_OK) return false;
  errfl = v & 0x0007;
  return true;
}

// Count one failure and print why it failed.
void reportFailure(const char *where, ReadStatus s) {
  Serial.print("FAIL ");
  Serial.print(where);
  Serial.print(": ");

  switch (s) {
    case READ_NO_RESPONSE:
      nNoResp++;
      Serial.print("no response (check CSn, MISO, power)");
      break;
    case READ_PARITY:
      nParity++;
      Serial.print("parity error (bit flip: check MISO, CLK)");
      break;
    case READ_EF: {
      nEF++;
      Serial.print("chip flagged error, ERRFL=");
      uint16_t errfl;
      if (readAndClearErrors(errfl)) {
        Serial.print("0b"); Serial.print(errfl, BIN);
      } else {
        // The chip rejected the ERRFL read too. Every command is arriving
        // garbled, so suspect MOSI or CLK.
        Serial.print("unreadable (check MOSI, CLK)");
      }
      break;
    }
    default:
      break;
  }
  Serial.println();
}

void printCounters() {
  Serial.print(" | good=");    Serial.print(nGood);
  Serial.print(" noResp=");    Serial.print(nNoResp);
  Serial.print(" parity=");    Serial.print(nParity);
  Serial.print(" EF=");        Serial.print(nEF);
  Serial.print(" COF=");       Serial.println(nCOF);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);  // deselected while idle
  SPI.begin();                 // SPI1 on D11/D12/D13 by default
  delay(10);                   // let the chip finish powering up

  uint16_t errfl;
  readAndClearErrors(errfl);   // clear anything left over from power-up. Result ignored on purpose
}

void loop() {
  uint16_t raw = 0, diag = 0;

  ReadStatus sA = readRegister(CMD_READ_ANGLECOM, raw);
  if (sA != READ_OK) { reportFailure("ANGLECOM", sA); delay(50); return; }

  ReadStatus sD = readRegister(CMD_READ_DIAAGC, diag);
  if (sD != READ_OK) { reportFailure("DIAAGC", sD); delay(50); return; }

  // 0x0000 passes parity and has EF clear, so readRegister can't reject it.
  // For ANGLECOM, 0 is a real angle. But a booted chip always reports LF=1
  // in DIAAGC, so LF=0 means the line is dead (MISO floating low), and the
  // angle from the same sample can't be trusted either.
  if (!(diag & DIAG_LF)) {
    reportFailure("DIAAGC LF=0", READ_NO_RESPONSE);
    delay(50);
    return;
  }

  // COF means the CORDIC overflowed, so the angle in this sample is invalid.
  // That's not a wiring fault, but the angle still can't be used.
  if (diag & DIAG_COF) {
    nCOF++;
    Serial.print("BAD SAMPLE: COF=1, angle invalid");
    printCounters();
    delay(50);
    return;
  }

  nGood++;
  Serial.print("raw=");   Serial.print(raw);
  Serial.print(" deg=");  Serial.print(raw * 360.0f / 16384.0f, 2);
  Serial.print(" MAGL="); Serial.print((diag & DIAG_MAGL) ? 1 : 0);
  Serial.print(" MAGH="); Serial.print((diag & DIAG_MAGH) ? 1 : 0);
  Serial.print(" AGC=");  Serial.print(diag & 0xFF);  // higher = weaker field
  printCounters();

  delay(50);  // ~20 Hz printout
}