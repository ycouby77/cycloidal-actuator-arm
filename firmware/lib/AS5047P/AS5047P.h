// AS5047P magnetic encoder driver (SPI, 14-bit absolute angle).
// Shared by encoder_test, motor_encoder_test, and later the FOC firmware.
//
// Every read is validated and returns a Status, so a loose wire shows up
// as an error instead of a fake angle.

#pragma once

#include <Arduino.h>
#include <SPI.h>

class AS5047P {
public:
  // Why a read failed. Ordered roughly from "wiring" to "magnet".
  enum class Status : uint8_t {
    Ok = 0,
    NoResponse,      // nobody driving MISO: 0xFFFF, or DIAAGC with LF=0. Check CSn, MISO, power
    ParityError,     // bits flipped on the way back. Check MISO, CLK
    ChipError,       // chip flagged a bad command (EF). See lastErrfl(). Check MOSI, CLK
    CordicOverflow,  // wiring fine, but this angle is invalid
    Count            // number of statuses, for sizing counter arrays
  };

  // One validated sample: angle plus magnet diagnostics from the same read.
  struct Sample {
    uint16_t raw  = 0;  // angle, 0..16383
    uint16_t diag = 0;  // DIAAGC register

    bool    magl() const { return diag & (1u << 11); }  // field too weak
    bool    magh() const { return diag & (1u << 10); }  // field too strong
    uint8_t agc()  const { return diag & 0xFF; }        // gain, higher = weaker field
  };

  static constexpr uint16_t COUNTS_PER_REV = 16384;

  // csPin: chip select. spiHz: 10 MHz max per datasheet, 1 MHz is plenty with jumpers.
  explicit AS5047P(uint8_t csPin, uint32_t spiHz = 1000000, SPIClass &spi = SPI);

  // Sets up CS and SPI, waits for power-up, clears leftover error flags.
  void begin();

  // Read angle + DIAAGC in one 3-frame burst and validate both.
  // s is only meaningful when this returns Status::Ok.
  Status read(Sample &s);

  // Read any register by address (0x0000..0x3FFF). For debugging and setup.
  Status readRegister(uint16_t addr, uint16_t &value);

  // ERRFL from the last ChipError. Returns false if the ERRFL read itself
  // came back bad, in which case the bits mean nothing.
  // bit2 PARERR (parity), bit1 INVCOMM (bad address), bit0 FRERR (framing)
  bool lastErrfl(uint16_t &errfl) const;

  static float       toDegrees(uint16_t raw);
  static const char *statusText(Status s);

private:
  uint16_t transfer16(uint16_t out);
  Status   checkFrame(uint16_t resp) const;
  void     fetchErrfl();

  static uint16_t readCommand(uint16_t addr);
  static bool     hasEvenParity(uint16_t x);

  uint8_t     cs_;
  SPISettings settings_;
  SPIClass   &spi_;

  // Command frames, built once from addresses (parity computed, not hand-typed)
  uint16_t cmdAngle_, cmdDiag_, cmdErrfl_, cmdNop_;

  uint16_t errfl_      = 0;
  bool     errflValid_ = false;
};
