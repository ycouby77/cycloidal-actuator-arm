#include "AS5047P.h"

// ---- Register addresses ----
static constexpr uint16_t REG_NOP      = 0x0000;
static constexpr uint16_t REG_ERRFL    = 0x0001;
static constexpr uint16_t REG_DIAAGC   = 0x3FFC;
static constexpr uint16_t REG_ANGLECOM = 0x3FFF;

// ---- Frame layout ----
// Command:  bit15 even parity | bit14 R/W (1 = read) | bits13:0 address
// Response: bit15 even parity | bit14 EF (error)     | bits13:0 data
static constexpr uint16_t PARITY_BIT = 1u << 15;
static constexpr uint16_t RW_BIT     = 1u << 14;
static constexpr uint16_t EF_BIT     = 1u << 14;
static constexpr uint16_t DATA_MASK  = 0x3FFF;

// ---- DIAAGC bits used for validation ----
static constexpr uint16_t DIAG_COF = 1u << 9;  // CORDIC overflow, angle invalid
static constexpr uint16_t DIAG_LF  = 1u << 8;  // offset comp finished. Always 1 on a booted chip

AS5047P::AS5047P(uint8_t csPin, uint32_t spiHz, SPIClass &spi)
    : cs_(csPin),
      settings_(spiHz, MSBFIRST, SPI_MODE1),  // mode 1: CPOL=0, CPHA=1
      spi_(spi) {
  cmdAngle_ = readCommand(REG_ANGLECOM);  // 0xFFFF
  cmdDiag_  = readCommand(REG_DIAAGC);    // 0xFFFC
  cmdErrfl_ = readCommand(REG_ERRFL);     // 0x4001
  cmdNop_   = readCommand(REG_NOP);       // 0xC000
}

void AS5047P::begin() {
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);  // deselected while idle
  spi_.begin();
  delay(10);                // let the chip finish powering up
  fetchErrfl();             // clear anything left over from power-up
}

// One SPI frame: CSn low -> 16 clocks -> CSn high.
// CSn going high is how the chip knows the frame ended.
uint16_t AS5047P::transfer16(uint16_t out) {
  spi_.beginTransaction(settings_);
  digitalWrite(cs_, LOW);
  uint16_t in = spi_.transfer16(out);
  digitalWrite(cs_, HIGH);
  spi_.endTransaction();
  delayMicroseconds(1);  // covers the minimum CSn high time
  return in;
}

// Validate one response frame. Order matters: 0xFFFF passes parity AND has
// EF set, so it has to be caught first or it looks like a chip error.
AS5047P::Status AS5047P::checkFrame(uint16_t resp) const {
  if (resp == 0xFFFF)       return Status::NoResponse;  // MISO floating high
  if (!hasEvenParity(resp)) return Status::ParityError;
  if (resp & EF_BIT)        return Status::ChipError;
  return Status::Ok;
}

AS5047P::Status AS5047P::read(Sample &s) {
  // 3-frame pipeline. Each response answers the PREVIOUS command, so asking
  // for DIAAGC while ANGLECOM's answer comes back saves a frame and keeps
  // the angle and diagnostics as close together in time as possible.
  transfer16(cmdAngle_);                     // response: leftover, ignore
  uint16_t rAngle = transfer16(cmdDiag_);    // response: ANGLECOM
  uint16_t rDiag  = transfer16(cmdNop_);     // response: DIAAGC

  Status st = checkFrame(rAngle);
  if (st == Status::Ok) st = checkFrame(rDiag);
  if (st != Status::Ok) {
    if (st == Status::ChipError) fetchErrfl();
    return st;
  }

  s.raw  = rAngle & DATA_MASK;
  s.diag = rDiag & DATA_MASK;

  // 0x0000 passes parity with EF clear, and 0 is a real angle, so the angle
  // frame alone can't tell a dead line from angle 0. DIAAGC can: a booted
  // chip always reports LF=1, so LF=0 means MISO is floating low.
  if (!(s.diag & DIAG_LF)) return Status::NoResponse;
  if (s.diag & DIAG_COF)   return Status::CordicOverflow;
  return Status::Ok;
}

AS5047P::Status AS5047P::readRegister(uint16_t addr, uint16_t &value) {
  transfer16(readCommand(addr));        // response: leftover, ignore
  uint16_t resp = transfer16(cmdNop_);  // response: the register

  Status st = checkFrame(resp);
  if (st == Status::ChipError) fetchErrfl();
  if (st == Status::Ok) value = resp & DATA_MASK;
  return st;
}

// Read ERRFL (which also clears it). Uses raw transfers, not readRegister(),
// so a bad ERRFL response can't trigger another ERRFL read.
void AS5047P::fetchErrfl() {
  transfer16(cmdErrfl_);
  uint16_t resp = transfer16(cmdNop_);

  // If the chip rejected this read too, the data bits are meaningless
  errflValid_ = (checkFrame(resp) == Status::Ok);
  errfl_      = errflValid_ ? (resp & 0x0007) : 0;
}

bool AS5047P::lastErrfl(uint16_t &errfl) const {
  errfl = errfl_;
  return errflValid_;
}

// Build a read command: set R/W, then set bit 15 if needed to make the
// total number of 1s even.
uint16_t AS5047P::readCommand(uint16_t addr) {
  uint16_t cmd = (addr & DATA_MASK) | RW_BIT;
  if (!hasEvenParity(cmd)) cmd |= PARITY_BIT;
  return cmd;
}

// True if x has an even number of 1s across all 16 bits.
bool AS5047P::hasEvenParity(uint16_t x) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 16; i++) {
    count += x & 1;
    x >>= 1;
  }
  return count % 2 == 0;
}

float AS5047P::toDegrees(uint16_t raw) {
  return raw * 360.0f / COUNTS_PER_REV;
}

const char *AS5047P::statusText(Status s) {
  switch (s) {
    case Status::Ok:             return "ok";
    case Status::NoResponse:     return "no response (check CSn, MISO, power)";
    case Status::ParityError:    return "parity error (check MISO, CLK)";
    case Status::ChipError:      return "chip error (check MOSI, CLK)";
    case Status::CordicOverflow: return "CORDIC overflow (angle invalid)";
    default:                     return "unknown";
  }
}
