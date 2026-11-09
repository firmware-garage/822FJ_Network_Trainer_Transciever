// panel_logic.h - hardware-free logic for the S3 cluster trainer.
//
// Everything here is plain C++ with no Arduino calls, so the same code runs in
// the firmware and in the host tests (test/host). Frames, scaling, the knob's
// step rules, the LCD row text and the three-state UI live here.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace panel {

// ------------------------------------------------------------ CRC ---------
// SAE J1850 CRC-8: poly 0x1D, init 0xFF, final XOR 0xFF. Same result as the
// library's crc(); used only to show and log the CRC byte the library appends.
inline uint8_t j1850Crc(const uint8_t* d, uint8_t n) {
  uint8_t r = 0xFF;
  for (uint8_t i = 0; i < n; i++) {
    r ^= d[i];
    for (uint8_t b = 0; b < 8; b++) r = (r & 0x80) ? (uint8_t)((r << 1) ^ 0x1D) : (uint8_t)(r << 1);
  }
  return (uint8_t)~r;
}

inline int32_t clampI(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ------------------------------------------------------- channels ---------
enum Chan : uint8_t { CH_FUEL = 0, CH_SPEED, CH_TEMP, CH_PRNDL, CH_COUNT };

struct ChanDef {
  const char* name;
  uint8_t     header[4];   // priority, target, source, message byte
  uint8_t     valueBytes;  // 1, or 2 big-endian
};

// LCD row order and selection order, top to bottom. Headers unchanged from the
// classic build; the knob only ever changes the value byte(s).
static const ChanDef CHANS[CH_COUNT] = {
  {"FUEL",  {0xA8, 0x83, 0x10, 0x12}, 1},
  {"SPEED", {0x88, 0x29, 0x11, 0x02}, 2},
  {"TEMP",  {0x68, 0x49, 0x10, 0x10}, 1},
  {"PRNDL", {0x68, 0xEA, 0x10, 0x0A}, 1},
};

// ----------------------------------------------------------- fuel ---------
// Raw byte, bench scale unknown beyond these marks, so the knob steps raw.
constexpr int32_t FUEL_RAW_MIN = 0x00, FUEL_RAW_MAX = 0xE2;
constexpr int32_t FUEL_STEP = 2, FUEL_STEP_FAST = 8;        // raw counts per detent
constexpr int32_t FUEL_BOOT_RAW = 0x7A;                      // 1/2
struct FuelMark { const char* label; uint8_t raw; };
static const FuelMark FUEL_MARKS[] = {
  {"E", 0x00}, {"1/4", 0x4A}, {"1/2", 0x7A}, {"3/4", 0xB0}, {"F", 0xE2},
};
inline const char* fuelNearestLabel(int32_t raw) {
  const char* best = FUEL_MARKS[0].label;
  int32_t bestDist = 1 << 30;
  for (const FuelMark& m : FUEL_MARKS) {
    int32_t d = raw - m.raw; if (d < 0) d = -d;
    if (d < bestDist) { bestDist = d; best = m.label; }
  }
  return best;
}

// ---------------------------------------------------------- speed ---------
// Knob works in whole mph. mph -> raw is piecewise-linear through the bench
// table, so every bench point comes out exact. 0 mph = 0x0000 is the UNM-16-5
// zero, not a bench reading; the 0..25 segment leans on it.
constexpr int32_t SPEED_MIN_MPH = 0, SPEED_MAX_MPH = 120;
constexpr int32_t SPEED_STEP = 1, SPEED_STEP_FAST = 5;
constexpr int32_t SPEED_BOOT_MPH = 0;
struct SpeedPoint { int32_t mph; int32_t raw; };
static const SpeedPoint SPEED_TABLE[] = {
  {0, 0x0000}, {25, 0x13F0}, {45, 0x23C7}, {65, 0x33AE},
  {85, 0x4394}, {100, 0x4F81}, {120, 0x5F68},
};
constexpr size_t SPEED_POINTS = sizeof(SPEED_TABLE) / sizeof(SPEED_TABLE[0]);

inline int32_t speedMphToRaw(int32_t mph) {
  mph = clampI(mph, SPEED_MIN_MPH, SPEED_MAX_MPH);
  for (size_t k = 1; k < SPEED_POINTS; k++) {
    const SpeedPoint& a = SPEED_TABLE[k - 1];
    const SpeedPoint& b = SPEED_TABLE[k];
    if (mph <= b.mph) {
      int32_t span = b.mph - a.mph;
      return a.raw + ((b.raw - a.raw) * (mph - a.mph) + span / 2) / span;
    }
  }
  return SPEED_TABLE[SPEED_POINTS - 1].raw;
}

// Nearest whole mph for any raw value (used for ramp values on the console).
inline int32_t speedRawToMph(int32_t raw) {
  if (raw <= 0) return 0;
  for (size_t k = 1; k < SPEED_POINTS; k++) {
    const SpeedPoint& a = SPEED_TABLE[k - 1];
    const SpeedPoint& b = SPEED_TABLE[k];
    if (raw <= b.raw) {
      int32_t span = b.raw - a.raw;
      return a.mph + ((b.mph - a.mph) * (raw - a.raw) + span / 2) / span;
    }
  }
  return SPEED_MAX_MPH;
}

// ----------------------------------------------------------- temp ---------
// J2178 coolant 68 49 10 10 xx: degC = raw - 40. Bench check: 140 F = 0x64.
constexpr int32_t TEMP_MIN_F = 100, TEMP_MAX_F = 260;
constexpr int32_t TEMP_STEP_F = 2, TEMP_STEP_FAST_F = 10;
constexpr int32_t TEMP_BOOT_F = 100;

// Integer round-half-away-from-zero of (f - 32) * 5 / 9, plus 40.
inline int32_t tempFtoRaw(int32_t f) {
  int32_t num = (f - 32) * 5;
  int32_t c = (num >= 0) ? (num + 4) / 9 : -((-num + 4) / 9);
  return clampI(40 + c, 0, 255);
}
inline int32_t tempRawToF(int32_t raw) {
  int32_t num = (raw - 40) * 9;
  int32_t q = (num >= 0) ? (num + 2) / 5 : -((-num + 2) / 5);
  return q + 32;
}

// ---------------------------------------------------------- PRNDL ---------
// Gate order, top to bottom. Clockwise moves down the gate, counter-clockwise
// moves up; it stops at P and at D1 instead of wrapping.
struct Gear { const char* label; uint8_t raw; };
static const Gear GEARS[] = {
  {"P", 0x01}, {"R", 0x02}, {"N", 0x03}, {"D", 0x09},
  {"D3", 0x08}, {"D2", 0x07}, {"D1", 0x06},
};
constexpr int32_t GEAR_COUNT = (int32_t)(sizeof(GEARS) / sizeof(GEARS[0]));
constexpr int32_t PRNDL_BOOT_IDX = 0;   // P

// ------------------------------------------------- setpoints / steps ------
// A channel's setpoint is kept in the unit the knob turns:
//   FUEL raw byte, SPEED mph, TEMP degF, PRNDL gear index.
inline int32_t bootSetpoint(uint8_t ch) {
  switch (ch) {
    case CH_FUEL:  return FUEL_BOOT_RAW;
    case CH_SPEED: return SPEED_BOOT_MPH;
    case CH_TEMP:  return TEMP_BOOT_F;
    default:       return PRNDL_BOOT_IDX;
  }
}

inline int32_t setpointToRaw(uint8_t ch, int32_t sp) {
  switch (ch) {
    case CH_FUEL:  return clampI(sp, FUEL_RAW_MIN, FUEL_RAW_MAX);
    case CH_SPEED: return speedMphToRaw(sp);
    case CH_TEMP:  return tempFtoRaw(sp);
    default:       return GEARS[clampI(sp, 0, GEAR_COUNT - 1)].raw;
  }
}

// New setpoint after `detents` clicks (signed). `fast` = the knob is spinning.
inline int32_t stepSetpoint(uint8_t ch, int32_t sp, int32_t detents, bool fast) {
  switch (ch) {
    case CH_FUEL:  return clampI(sp + detents * (fast ? FUEL_STEP_FAST : FUEL_STEP), FUEL_RAW_MIN, FUEL_RAW_MAX);
    case CH_SPEED: return clampI(sp + detents * (fast ? SPEED_STEP_FAST : SPEED_STEP), SPEED_MIN_MPH, SPEED_MAX_MPH);
    case CH_TEMP:  return clampI(sp + detents * (fast ? TEMP_STEP_FAST_F : TEMP_STEP_F), TEMP_MIN_F, TEMP_MAX_F);
    default:       return clampI(sp + detents, 0, GEAR_COUNT - 1);   // one gear per click, never accelerated
  }
}

// --------------------------------------------------------- frames ---------
// Payload WITHOUT CRC: 4 header bytes + value. Returns its length (5 or 6).
inline uint8_t buildPayload(uint8_t ch, int32_t raw, uint8_t* out) {
  memcpy(out, CHANS[ch].header, 4);
  if (CHANS[ch].valueBytes == 2) {
    out[4] = (uint8_t)((raw >> 8) & 0xFF);
    out[5] = (uint8_t)(raw & 0xFF);
    return 6;
  }
  out[4] = (uint8_t)(raw & 0xFF);
  return 5;
}

// ------------------------------------------------------- LCD rows ---------
// Col 0 is the marker. Then the three header bytes as six digits, a space,
// the data bytes (message byte + value), a space, and the CRC:
//   " A88310 127A B5"      six-byte frame, 15 columns
//   " 882911 020000 3B"    seven-byte speed frame, 17 columns
// `frame` is the complete on-wire frame including its CRC (len >= 5).
// Writes exactly 20 characters plus a terminating 0 into out[21].
inline uint8_t formatRow(char marker, const uint8_t* frame, uint8_t len, char* out) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  uint8_t col = 0;
  memset(out, ' ', 20);
  out[20] = 0;
  out[col++] = marker;
  auto putByte = [&](uint8_t b) {
    if (col + 2 > 20) return;
    out[col++] = HEX_DIGITS[b >> 4];
    out[col++] = HEX_DIGITS[b & 0x0F];
  };
  for (uint8_t k = 0; k < 3 && k < len; k++) putByte(frame[k]);
  col++;                                          // space
  for (uint8_t k = 3; k + 1 < len; k++) putByte(frame[k]);
  col++;                                          // space
  if (len >= 1) putByte(frame[len - 1]);          // CRC
  return col;
}

// ------------------------------------------------------------ UI ----------
// PASSIVE: frames only, no marker, knob turns ignored.
// press -> SELECT: blinking '>' on one row, turning moves it (wraps).
// press -> EDIT:   solid filled arrow on that row, turning changes its value.
// press -> back to SELECT. No input for idleTimeoutMs in SELECT or EDIT -> PASSIVE.
enum UiState : uint8_t { UI_PASSIVE = 0, UI_SELECT, UI_EDIT };

constexpr char MARK_NONE   = ' ';
constexpr char MARK_SELECT = '>';
constexpr char MARK_EDIT   = 0x01;   // HD44780 CGRAM slot 1: filled right triangle

struct Ui {
  UiState  state = UI_PASSIVE;
  uint8_t  sel = CH_FUEL;          // remembered between visits
  uint32_t lastInputMs = 0;
  uint32_t blinkT0 = 0;
  uint32_t lastDetentMs = 0;
};

struct UiConfig {
  uint32_t idleTimeoutMs;
  uint32_t blinkMs;
  uint32_t fastDetentMs;           // detents closer together than this count as fast
};

// Press. Returns the new state.
inline UiState uiPress(Ui& ui, uint32_t now) {
  ui.lastInputMs = now;
  switch (ui.state) {
    case UI_PASSIVE: ui.state = UI_SELECT; ui.blinkT0 = now; break;
    case UI_SELECT:  ui.state = UI_EDIT; break;
    case UI_EDIT:    ui.state = UI_SELECT; ui.blinkT0 = now; break;
  }
  return ui.state;
}

// What a turn asks the firmware to do.
struct TurnResult {
  bool    adjust = false;    // true: change channel `ch` by `detents`
  uint8_t ch = 0;
  int32_t detents = 0;
  bool    fast = false;
};

inline TurnResult uiTurn(Ui& ui, int32_t detents, uint32_t now, const UiConfig& cfg) {
  TurnResult r;
  if (detents == 0 || ui.state == UI_PASSIVE) return r;   // passive: turns ignored
  ui.lastInputMs = now;
  if (ui.state == UI_SELECT) {
    int32_t s = ((int32_t)ui.sel + detents) % (int32_t)CH_COUNT;
    if (s < 0) s += CH_COUNT;
    ui.sel = (uint8_t)s;
    ui.blinkT0 = now;            // marker shows immediately at its new row
    return r;
  }
  // UI_EDIT
  int32_t mag = detents < 0 ? -detents : detents;
  r.fast = (mag > 1) || ((now - ui.lastDetentMs) < cfg.fastDetentMs);
  ui.lastDetentMs = now;
  r.adjust = true;
  r.ch = ui.sel;
  r.detents = detents;
  return r;
}

// Idle timeout. Returns true when it just dropped to PASSIVE.
inline bool uiTick(Ui& ui, uint32_t now, const UiConfig& cfg) {
  if (ui.state != UI_PASSIVE && (now - ui.lastInputMs) >= cfg.idleTimeoutMs) {
    ui.state = UI_PASSIVE;
    return true;
  }
  return false;
}

inline char uiMarker(const Ui& ui, uint8_t row, uint32_t now, const UiConfig& cfg) {
  if (ui.state == UI_PASSIVE || row != ui.sel) return MARK_NONE;
  if (ui.state == UI_EDIT) return MARK_EDIT;
  return (((now - ui.blinkT0) / cfg.blinkMs) & 1) == 0 ? MARK_SELECT : MARK_NONE;
}

}  // namespace panel
