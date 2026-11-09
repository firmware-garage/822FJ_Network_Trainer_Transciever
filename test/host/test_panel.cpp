// Host tests for src/panel_logic.h. Build and run:  ./test/host/run.sh
// Checks frames, CRC (against the vendored library's own crc()), scaling,
// knob steps, LCD row text and the three-state UI. No hardware involved.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include "../../src/panel_logic.h"

using namespace panel;

// Verbatim copy of crc() from lib/J1850VPW_ESP32/src/j1850vpw.cpp lines 62-91
// (copied by run.sh into lib_crc.inc so it is the real library text).
#include "lib_crc.inc"

static int fails = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void frameFor(uint8_t ch, int32_t raw, uint8_t* f, uint8_t& len) {
  uint8_t pl = buildPayload(ch, raw, f);
  f[pl] = j1850Crc(f, pl);
  len = pl + 1;
}

int main() {
  const UiConfig cfg = {10000, 400, 40};

  // ---- CRC vectors from the bench / earlier work
  { uint8_t n[] = {0x68, 0xEA, 0x10, 0x0A, 0x03};       CHECK(j1850Crc(n, 5) == 0x94, "neutral crc"); }
  { uint8_t n[] = {0x68, 0xEA, 0x10, 0x0A, 0x03, 0x40}; CHECK(j1850Crc(n, 6) == 0xB7, "neutral+40 crc"); }
  { uint8_t h[] = {0xE8, 0xFF, 0x10, 0x03};             CHECK(j1850Crc(h, 4) == 0xB3, "heartbeat crc"); }
  { uint8_t b[] = {0x8A, 0xEA, 0x40, 0xA0, 0x91, 0x00}; CHECK(j1850Crc(b, 6) == 0x5B, "beam on crc"); }
  { uint8_t b[] = {0x8A, 0xEA, 0x40, 0x20, 0x91, 0x00}; CHECK(j1850Crc(b, 6) == 0x9B, "beam off crc"); }

  // ---- ours == library's, random frames 1..11 bytes
  {
    std::mt19937 rng(1850);
    int mism = 0;
    for (int t = 0; t < 200000; t++) {
      uint8_t buf[12]; int n = 1 + (int)(rng() % 11);
      for (int k = 0; k < n; k++) buf[k] = (uint8_t)rng();
      if (j1850Crc(buf, (uint8_t)n) != crc(buf, (int8_t)n)) mism++;
    }
    CHECK(mism == 0, "%d of 200000 random frames differ from library crc()", mism);
  }

  // ---- frame lengths and headers untouched by value changes
  for (uint8_t ch = 0; ch < CH_COUNT; ch++) {
    uint8_t f[8], len;
    frameFor(ch, setpointToRaw(ch, bootSetpoint(ch)), f, len);
    CHECK(len == (ch == CH_SPEED ? 7 : 6), "%s frame length %d", CHANS[ch].name, len);
    for (int32_t d = -300; d <= 300; d += 7) {
      int32_t sp = stepSetpoint(ch, bootSetpoint(ch), d, false);
      uint8_t g[8], gl;
      frameFor(ch, setpointToRaw(ch, sp), g, gl);
      CHECK(memcmp(g, CHANS[ch].header, 4) == 0, "%s header changed", CHANS[ch].name);
    }
  }

  // ---- speed: exact at every bench point, monotonic, bounded
  CHECK(speedMphToRaw(25) == 0x13F0 && speedMphToRaw(45) == 0x23C7 && speedMphToRaw(65) == 0x33AE &&
        speedMphToRaw(85) == 0x4394 && speedMphToRaw(100) == 0x4F81 && speedMphToRaw(120) == 0x5F68,
        "speed table points");
  CHECK(speedMphToRaw(0) == 0 && speedMphToRaw(-5) == 0 && speedMphToRaw(500) == 0x5F68, "speed clamp");
  { bool mono = true; for (int m = 1; m <= 120; m++) if (speedMphToRaw(m) <= speedMphToRaw(m - 1)) mono = false;
    CHECK(mono, "speed monotonic"); }
  { bool rt = true; for (int m = 0; m <= 120; m++) if (speedRawToMph(speedMphToRaw(m)) != m) rt = false;
    CHECK(rt, "speed round trip"); }
  { uint8_t f[8], len; frameFor(CH_SPEED, speedMphToRaw(25), f, len);
    CHECK(f[4] == 0x13 && f[5] == 0xF0, "speed big-endian"); }

  // ---- temp
  CHECK(tempFtoRaw(140) == 0x64, "140F = 0x64 (bench)");
  CHECK(tempFtoRaw(212) == 140, "212F = 100C = raw 140");
  CHECK(tempFtoRaw(100) == 78 && tempFtoRaw(260) == 167, "temp range ends");
  CHECK(stepSetpoint(CH_TEMP, 258, 5, false) == 260 && stepSetpoint(CH_TEMP, 102, -5, false) == 100, "temp clamp");
  CHECK(stepSetpoint(CH_TEMP, 180, 1, false) == 182 && stepSetpoint(CH_TEMP, 180, 1, true) == 190, "temp steps");

  // ---- fuel
  CHECK(stepSetpoint(CH_FUEL, 0x7A, 1, false) == 0x7C && stepSetpoint(CH_FUEL, 0x7A, -1, true) == 0x72, "fuel steps");
  CHECK(stepSetpoint(CH_FUEL, 0xE0, 10, true) == 0xE2 && stepSetpoint(CH_FUEL, 0x02, -10, false) == 0x00, "fuel clamp");
  CHECK(!strcmp(fuelNearestLabel(0x7A), "1/2") && !strcmp(fuelNearestLabel(0x02), "E"), "fuel labels");

  // ---- PRNDL: P R N D D3 D2 D1, no wrap, one per click even when fast
  {
    int32_t g = 0;
    const uint8_t expect[] = {0x01, 0x02, 0x03, 0x09, 0x08, 0x07, 0x06};
    for (int k = 0; k < 7; k++) { CHECK(setpointToRaw(CH_PRNDL, g) == expect[k], "gear %d raw", k); g = stepSetpoint(CH_PRNDL, g, 1, true); }
    CHECK(g == 6, "stops at D1");
    CHECK(stepSetpoint(CH_PRNDL, 0, -1, false) == 0, "stops at P");
    uint8_t f[8], len; frameFor(CH_PRNDL, 0x03, f, len);
    CHECK(len == 6 && f[5] == 0x94, "neutral frame on the wire ends 94");
  }

  // ---- LCD rows: exact text and widths
  {
    char row[21];
    uint8_t f[8], len;
    frameFor(CH_PRNDL, 0x03, f, len);
    uint8_t w = formatRow(' ', f, len, row);
    CHECK(w == 15 && !strcmp(row, " 68EA10 0A03 94     "), "neutral row '%s' width %d", row, w);

    frameFor(CH_SPEED, speedMphToRaw(25), f, len);
    char expect[21]; snprintf(expect, sizeof expect, ">882911 0213F0 %02X   ", f[6]);
    w = formatRow('>', f, len, row);
    CHECK(w == 17 && !strcmp(row, expect), "speed row '%s' width %d", row, w);

    frameFor(CH_FUEL, 0x00, f, len);   // leading zeros kept
    formatRow(' ', f, len, row);
    CHECK(!strncmp(row, " A88310 1200 ", 13), "fuel E row '%s'", row);

    for (uint8_t ch = 0; ch < CH_COUNT; ch++) {
      frameFor(ch, setpointToRaw(ch, bootSetpoint(ch)), f, len);
      w = formatRow(MARK_EDIT, f, len, row);
      CHECK(strlen(row) == 20 && row[0] == MARK_EDIT && row[7] == ' ' && row[w - 3] == ' ', "%s row layout", CHANS[ch].name);
    }
  }

  // ---- UI state machine
  {
    Ui ui; uint32_t t = 1000; ui.lastInputMs = t;
    TurnResult r = uiTurn(ui, 3, t, cfg);
    CHECK(ui.state == UI_PASSIVE && !r.adjust && ui.sel == CH_FUEL, "passive ignores turns");
    for (uint8_t row = 0; row < 4; row++) CHECK(uiMarker(ui, row, t, cfg) == MARK_NONE, "passive: no markers");

    uiPress(ui, t);                                     // 1st press
    CHECK(ui.state == UI_SELECT, "press 1 -> SELECT");
    CHECK(uiMarker(ui, CH_FUEL, t, cfg) == MARK_SELECT && uiMarker(ui, CH_FUEL, t + 400, cfg) == MARK_NONE &&
          uiMarker(ui, CH_FUEL, t + 800, cfg) == MARK_SELECT, "select marker blinks");
    CHECK(uiMarker(ui, CH_SPEED, t, cfg) == MARK_NONE, "only the selected row is marked");

    t += 500; r = uiTurn(ui, 1, t, cfg);
    CHECK(!r.adjust && ui.sel == CH_SPEED && uiMarker(ui, CH_SPEED, t, cfg) == MARK_SELECT, "turn moves selection, shown at once");
    t += 100; uiTurn(ui, -2, t, cfg);
    CHECK(ui.sel == CH_PRNDL, "selection wraps upward");
    t += 100; uiTurn(ui, 1, t, cfg);
    CHECK(ui.sel == CH_FUEL, "selection wraps downward");
    t += 100; uiTurn(ui, 2, t, cfg);
    CHECK(ui.sel == CH_TEMP, "select temp");

    t += 100; uiPress(ui, t);                           // 2nd press
    CHECK(ui.state == UI_EDIT, "press 2 -> EDIT");
    CHECK(uiMarker(ui, CH_TEMP, t, cfg) == MARK_EDIT && uiMarker(ui, CH_TEMP, t + 400, cfg) == MARK_EDIT, "edit marker solid");
    t += 500; r = uiTurn(ui, 1, t, cfg);
    CHECK(r.adjust && r.ch == CH_TEMP && r.detents == 1 && !r.fast, "edit turn adjusts temp, slow");
    t += 20; r = uiTurn(ui, 1, t, cfg);
    CHECK(r.fast, "quick second click is fast");
    t += 20; r = uiTurn(ui, 3, t + 500, cfg);
    CHECK(r.fast, "several clicks in one pass count as fast");
    t += 1000;

    uiPress(ui, t);                                     // 3rd press
    CHECK(ui.state == UI_SELECT && ui.sel == CH_TEMP, "press 3 -> back to SELECT on same row");

    CHECK(!uiTick(ui, t + 9999, cfg) && ui.state == UI_SELECT, "not yet idle at 9.999 s");
    CHECK(uiTick(ui, t + 10000, cfg) && ui.state == UI_PASSIVE, "idle 10 s -> PASSIVE");
    for (uint8_t row = 0; row < 4; row++) CHECK(uiMarker(ui, row, t + 10000, cfg) == MARK_NONE, "markers hidden after timeout");

    t += 20000; uiPress(ui, t); uiPress(ui, t + 10);
    CHECK(ui.state == UI_EDIT && ui.sel == CH_TEMP, "selection remembered");
    t += 10; uiTurn(ui, 1, t + 5000, cfg);
    CHECK(!uiTick(ui, t + 14000, cfg), "turning in EDIT resets the idle timer");
    CHECK(uiTick(ui, t + 15000, cfg) && ui.state == UI_PASSIVE, "EDIT times out too");

    Ui w; w.lastInputMs = 0xFFFFF000u; w.state = UI_SELECT;   // millis() rollover
    CHECK(!uiTick(w, 0x00000100u, cfg), "timeout survives millis rollover");
  }

  printf("%d checks, %d failed\n", checks, fails);
  return fails ? 1 : 0;
}
