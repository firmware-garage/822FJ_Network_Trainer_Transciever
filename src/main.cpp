// Cluster trainer, ESP32-S3 port: drives a GM Class 2 (J1850 VPW) instrument
// cluster's fuel, speed, coolant temp and PRNDL from one rotary encoder, and
// shows each gauge's on-wire frame on a 20x4 I2C LCD at 0x27.
//
// Two cores, same split as your web-injector build:
//   core 1  "vpw" task: scheduler, ramps and the library's bit-banged send().
//           Nothing else runs there, and no GPIO interrupt lands there.
//   core 0  "ui" task: encoder, button, LCD, serial console, received frames,
//           and the one shared GPIO interrupt service (encoder + VPW RX edges).
// The two tasks only meet in `shared`, under one short spinlock.
//
// Bus driver: VoodooMods' ESP32 fork of matafonoff's J1850 VPW library,
// vendored in lib/J1850VPW_ESP32 (see PATCHES.md there). send() appends the CRC.
// Classic ESP32 build stays untouched in ../cluster_trainer.

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <j1850vpw.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <soc/gpio_reg.h>
#include <soc/soc.h>

#include "panel_logic.h"

using namespace panel;

// The library's CRC routine is a global in j1850vpw.cpp but not in its header.
extern uint8_t crc(uint8_t* msg_buf, int8_t nbytes);

#if !CONFIG_IDF_TARGET_ESP32S3
#error "This is the ESP32-S3 build. The classic ESP32 build is in ../cluster_trainer."
#endif
#if CONFIG_FREERTOS_UNICORE
#error "Needs both cores: bus timing on one, UI on the other"
#endif

// ---------------------------------------------------------------- pins -----
// Chosen from GPIO 1-13, which both an ESP32-S3 Super Mini and a
// DevKitC-1 N16R8 bring out. Avoided: 0/3/45/46 (strapping), 19/20 (native
// USB), 26-32 (in-package flash/PSRAM), 33-37 (octal PSRAM on R8 parts),
// 43/44 (UART0), 48 (RGB LED on many S3 boards).
constexpr uint8_t PIN_VPW_RX  = 4;
constexpr uint8_t PIN_VPW_TX  = 5;
constexpr uint8_t PIN_ENC_A   = 6;
constexpr uint8_t PIN_ENC_B   = 7;
constexpr uint8_t PIN_SDA     = 8;    // also the S3 Arduino default SDA
constexpr uint8_t PIN_SCL     = 9;    // also the S3 Arduino default SCL
constexpr uint8_t PIN_ENC_BTN = 10;

constexpr bool pinUsable(uint8_t p) {
  return p <= 48 && p != 0 && p != 3 && p != 45 && p != 46 && p != 19 && p != 20 &&
         !(p >= 22 && p <= 25) && !(p >= 26 && p <= 37) && p != 43 && p != 44 && p != 48;
}
static_assert(pinUsable(PIN_VPW_RX) && pinUsable(PIN_VPW_TX) && pinUsable(PIN_ENC_A) &&
              pinUsable(PIN_ENC_B) && pinUsable(PIN_SDA) && pinUsable(PIN_SCL) && pinUsable(PIN_ENC_BTN),
              "pin is reserved, missing or a strapping pin on the ESP32-S3");

// --------------------------------------------------------------- cores -----
constexpr BaseType_t VPW_CORE = 1;
constexpr BaseType_t UI_CORE  = 0;
constexpr UBaseType_t VPW_TASK_PRIO = 5;   // above loopTask (1) and idle (0) on core 1
constexpr UBaseType_t UI_TASK_PRIO  = 1;
static_assert(VPW_CORE != UI_CORE, "bus and UI must be on different cores");

// -------------------------------------------------------------- tuning -----
constexpr uint8_t  LCD_ADDR           = 0x27;  // your PCF8574 backpack
constexpr uint32_t LCD_REFRESH_MS     = 100;   // rows rebuilt this often; only changed cells are written
constexpr uint8_t  LCD_CHARS_PER_PASS = 8;     // I2C chars per UI pass (1 ms); never touches the bus core

constexpr int8_t   ENC_TRANSITIONS_PER_DETENT = 4;  // KY-040 style: 4. Some encoders: 2.
constexpr bool     ENC_REVERSE        = false;      // flip if clockwise goes the wrong way
constexpr uint32_t BTN_DEBOUNCE_MS    = 30;

// ASSUMPTION: you did not give a timeout. 10 s with no press or turn in
// selection or edit drops back to the passive display.
constexpr UiConfig UI_CFG = {
  /* idleTimeoutMs */ 10000,
  /* blinkMs       */ 400,     // selection '>' on 400 ms, off 400 ms
  /* fastDetentMs  */ 40,      // clicks closer than this use the coarse step
};

constexpr uint32_t RAMP_MS[CH_COUNT]  = {2000, 2000, 0, 0};   // fuel, speed ramp; temp, PRNDL follow the knob
constexpr uint32_t PERIOD_MS[CH_COUNT]= {500, 200, 500, 500};
constexpr uint32_t PHASE_MS[CH_COUNT] = {0, 60, 120, 180};    // stagger at boot
constexpr uint32_t PERIOD_HEARTBEAT_MS= 1000;
constexpr uint32_t PERIOD_RAMPING_MS  = 100;   // faster refresh while a ramp runs
constexpr uint32_t MIN_GAP_SAME_MS    = 50;    // fastest one frame may repeat
constexpr uint32_t MIN_GAP_ANY_MS     = 10;    // breathing room between our frames
constexpr uint32_t RETRY_MS           = 15;    // after bus busy / arbitration lost

// Idle wait inside send(). The library multiplies this int16 by 1000.
constexpr int16_t  VPW_IDLE_WAIT_MS   = 20;
static_assert(VPW_IDLE_WAIT_MS <= 32, "library overflows int16 above 32 ms");

// The vendored send() does buff[nbytes] = crc(buff, nbytes) on every call.
constexpr bool LIBRARY_APPENDS_CRC = true;

// Library default: ACTIVE = LOW on both RX and TX. Your sniffer confirms the
// RX side (pin LOW = bus active). The library has one level for both pins.
constexpr uint8_t VPW_TX_PASSIVE_LEVEL = 1;

static const uint8_t HEARTBEAT_PAYLOAD[] = {0xE8, 0xFF, 0x10, 0x03};          // wire: ...B3
static const uint8_t BEAM_ON_PAYLOAD[]   = {0x8A, 0xEA, 0x40, 0xA0, 0x91, 0x00}; // wire: ...5B
static const uint8_t BEAM_OFF_PAYLOAD[]  = {0x8A, 0xEA, 0x40, 0x20, 0x91, 0x00}; // wire: ...9B

static inline bool timeReached(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

// ======================================================= shared state ======
// Every field is touched only inside shMux. Critical sections copy a few
// bytes and never call send(), Serial, Wire or the LCD.
struct Shared {
  // UI -> bus
  int32_t  reqRaw[CH_COUNT];
  uint32_t reqSeq[CH_COUNT];      // bumped on every new target
  uint8_t  oneShot[BS];
  uint8_t  oneShotLen;            // 0 = nothing queued
  bool     hbEnabled;
  bool     hbKick;                // send the heartbeat right away
  // bus -> UI
  uint8_t  frame[CH_COUNT][8];    // last frame that went out OK, CRC included
  uint8_t  frameLen[CH_COUNT];
  int32_t  txRaw[CH_COUNT];
  int32_t  target[CH_COUNT];
  bool     ramping[CH_COUNT];
  uint8_t  sentOnce[BS + 1];      // console frame that went out, for the UI to print
  uint8_t  sentOnceLen;
  uint32_t txOk, txBusy, txArb, txOther;
};
static Shared shared;
static portMUX_TYPE shMux = portMUX_INITIALIZER_UNLOCKED;

J1850VPW vpw;
static volatile bool vpwReady = false;     // set by the UI task once vpw.init() ran on core 0

// ==================================================== core 1: bus task =====
struct ChannelState {
  int32_t  target, rampFrom, current, lastTxRaw;
  uint32_t rampT0, nextDueMs, lastSentMs;
  uint32_t seqSeen;
  bool     ramping;
};
static ChannelState st[CH_COUNT];          // bus task only
static uint32_t hbNextDueMs = 0;
static uint32_t lastAnyTxMs = 0;

static void sendSoon(uint8_t i, uint32_t now) {
  uint32_t earliest = st[i].lastSentMs + MIN_GAP_SAME_MS;
  st[i].nextDueMs = timeReached(now, earliest) ? now : earliest;
}

static void updateRamp(uint8_t i, uint32_t now) {
  ChannelState& s = st[i];
  if (!s.ramping) return;
  uint32_t dur = RAMP_MS[i], el = now - s.rampT0;
  if (dur == 0 || el >= dur) { s.current = s.target; s.ramping = false; return; }
  s.current = s.rampFrom + (int32_t)(((int64_t)(s.target - s.rampFrom) * (int64_t)el) / (int64_t)dur);
}

// New target. A ramp starts from the value last put on the wire, so a retarget
// mid-ramp continues from where the cluster actually is: no jump.
static void setTarget(uint8_t i, int32_t raw, uint32_t now) {
  ChannelState& s = st[i];
  s.target = raw;
  if (RAMP_MS[i] == 0 || s.lastTxRaw == raw) {
    s.current = raw;
    s.ramping = false;
  } else {
    s.rampFrom = s.lastTxRaw;
    s.current  = s.lastTxRaw;
    s.rampT0   = now;
    s.ramping  = true;
  }
  sendSoon(i, now);
}

// One send. Payload WITHOUT CRC; the library appends it.
static uint8_t vpwSend(const uint8_t* payload, uint8_t len) {
  uint8_t buf[BS];
  memcpy(buf, payload, len);                 // send() takes a non-const pointer
  if (!LIBRARY_APPENDS_CRC) {
    uint8_t c = j1850Crc(payload, len);
    buf[len++] = c;
  }
  uint8_t r = vpw.send(buf, len, VPW_IDLE_WAIT_MS);
  lastAnyTxMs = millis();
  portENTER_CRITICAL(&shMux);
  switch (r) {
    case J1850_OK:                   shared.txOk++;    break;
    case J1850_ERR_BUS_IS_BUSY:      shared.txBusy++;  break;
    case J1850_ERR_ARBITRATION_LOST: shared.txArb++;   break;
    default:                         shared.txOther++; break;
  }
  portEXIT_CRITICAL(&shMux);
  return r;
}

// Take new targets and a queued console frame from the UI.
static void pullRequests(uint32_t now, uint8_t* once, uint8_t& onceLen, bool& hbOn) {
  int32_t raw[CH_COUNT];
  uint32_t seq[CH_COUNT];
  bool kick;
  portENTER_CRITICAL(&shMux);
  memcpy(raw, shared.reqRaw, sizeof(raw));
  memcpy(seq, shared.reqSeq, sizeof(seq));
  hbOn = shared.hbEnabled;
  kick = shared.hbKick;
  shared.hbKick = false;
  if (onceLen == 0 && shared.oneShotLen) {
    onceLen = shared.oneShotLen;
    memcpy(once, shared.oneShot, onceLen);
    shared.oneShotLen = 0;
  }
  portEXIT_CRITICAL(&shMux);
  if (kick) hbNextDueMs = now;
  for (uint8_t i = 0; i < CH_COUNT; i++) {
    if (seq[i] != st[i].seqSeen) {
      st[i].seqSeen = seq[i];
      if (raw[i] != st[i].target) setTarget(i, raw[i], now);
    }
  }
}

static void publishChannel(uint8_t i, const uint8_t* payload, uint8_t len) {
  uint8_t c = j1850Crc(payload, len);
  portENTER_CRITICAL(&shMux);
  memcpy(shared.frame[i], payload, len);
  shared.frame[i][len] = c;
  shared.frameLen[i] = len + 1;
  shared.txRaw[i] = st[i].lastTxRaw;
  shared.target[i] = st[i].target;
  shared.ramping[i] = st[i].ramping;
  portEXIT_CRITICAL(&shMux);
}

// At most one frame per call. Returns true if it tried the bus.
static bool serviceBus(uint32_t now, uint8_t* once, uint8_t& onceLen, bool hbOn) {
  if ((now - lastAnyTxMs) < MIN_GAP_ANY_MS) return false;

  if (onceLen) {                              // queued console frame first
    if (vpwSend(once, onceLen) == J1850_OK) {
      portENTER_CRITICAL(&shMux);
      memcpy(shared.sentOnce, once, onceLen);
      shared.sentOnce[onceLen] = j1850Crc(once, onceLen);
      shared.sentOnceLen = onceLen + 1;
      portEXIT_CRITICAL(&shMux);
      onceLen = 0;
    }
    return true;   // on busy/arb lost it stays queued and retries
  }

  int8_t pick = -1;                           // most overdue periodic channel
  int32_t worst = -1;
  for (uint8_t i = 0; i < CH_COUNT; i++) {
    int32_t late = (int32_t)(now - st[i].nextDueMs);
    if (late >= 0 && late > worst) { worst = late; pick = i; }
  }
  bool hbDue = hbOn && timeReached(now, hbNextDueMs);
  if (hbDue && (int32_t)(now - hbNextDueMs) > worst) pick = -2;

  if (pick == -2) {
    uint8_t r = vpwSend(HEARTBEAT_PAYLOAD, sizeof(HEARTBEAT_PAYLOAD));
    hbNextDueMs = now + ((r == J1850_OK) ? PERIOD_HEARTBEAT_MS : RETRY_MS);
    return true;
  }
  if (pick < 0) return false;

  uint8_t i = (uint8_t)pick;
  updateRamp(i, now);
  int32_t raw = st[i].current;
  uint8_t payload[8];
  uint8_t len = buildPayload(i, raw, payload);
  if (vpwSend(payload, len) == J1850_OK) {
    st[i].lastTxRaw = raw;
    st[i].lastSentMs = now;
    bool stillMoving = st[i].ramping || st[i].current != st[i].target;
    st[i].nextDueMs = now + (stillMoving ? PERIOD_RAMPING_MS : PERIOD_MS[i]);
    publishChannel(i, payload, len);
  } else {
    st[i].nextDueMs = now + RETRY_MS;
  }
  return true;
}

static void vpwTask(void*) {
  while (!vpwReady) vTaskDelay(1);            // UI task owns init (see uiTask)

  uint32_t now = millis();
  for (uint8_t i = 0; i < CH_COUNT; i++) {    // boot values, no ramp
    ChannelState& s = st[i];
    memset(&s, 0, sizeof(s));
    portENTER_CRITICAL(&shMux);
    s.target = s.current = s.lastTxRaw = shared.reqRaw[i];
    s.seqSeen = shared.reqSeq[i];
    portEXIT_CRITICAL(&shMux);
    s.nextDueMs = now + 100 + PHASE_MS[i];
    s.lastSentMs = now;
  }
  hbNextDueMs = now + 100 + 240;

  uint8_t once[BS];
  uint8_t onceLen = 0;
  for (;;) {
    now = millis();
    bool hbOn;
    pullRequests(now, once, onceLen, hbOn);
    for (uint8_t i = 0; i < CH_COUNT; i++) updateRamp(i, now);
    if (!serviceBus(now, once, onceLen, hbOn)) vTaskDelay(1);   // nothing due: give core 1 back for 1 ms
  }
}

// ========================================== core 0: interrupts (IRAM) ======
static volatile int32_t encTransitions = 0;
static volatile uint8_t encPrev = 0;
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
DRAM_ATTR static const int8_t ENC_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// One register read per pin. S3: GPIO0-31 in GPIO_IN_REG, 32-48 in GPIO_IN1_REG.
static inline uint8_t IRAM_ATTR gpioLevel(uint8_t pin) {
  return pin < 32 ? (uint8_t)((REG_READ(GPIO_IN_REG) >> pin) & 1)
                  : (uint8_t)((REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1);
}
static inline uint8_t IRAM_ATTR readEncAB() {
  return (uint8_t)((gpioLevel(PIN_ENC_A) << 1) | gpioLevel(PIN_ENC_B));
}

static volatile int8_t gpioIsrCore = -1;   // bench check: should read 0 in `stat`

void IRAM_ATTR encoderIsr() {
  gpioIsrCore = (int8_t)xPortGetCoreID();
  uint8_t ab = readEncAB();
  portENTER_CRITICAL_ISR(&encMux);
  uint8_t idx = (uint8_t)((encPrev << 2) | ab) & 0x0F;
  encTransitions = encTransitions + ENC_TABLE[idx];
  encPrev = ab;
  portEXIT_CRITICAL_ISR(&encMux);
}

// Read errors from the library's RX interrupt path (core 0) and from
// tryGetReceivedFrame (also core 0). Counters only, no Serial.
static volatile uint32_t rxErrCount = 0, rxCrcErrCount = 0;
void IRAM_ATTR onVpwError(J1850_Operations op, J1850_ERRORS err) {
  if (op != J1850_Read) return;      // write results are counted from send()'s return
  rxErrCount = rxErrCount + 1;
  if (err == J1850_ERR_CRC) rxCrcErrCount = rxCrcErrCount + 1;
}

// ===================================================== core 0: UI task =====
LiquidCrystal_I2C* lcd = nullptr;
static Ui ui;
static int32_t setpoint[CH_COUNT];          // knob units: fuel raw, mph, degF, gear index
static bool rxPrintEnabled = false;
static uint32_t rxFrames = 0;

// The library's RX ISR pushes into its frame store and tryGetReceivedFrame()
// pops with no locking. Both run on core 0 here, so masking core 0's
// interrupts during the pop is enough.
static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

static int32_t takeDetents() {
  static int32_t acc = 0;
  portENTER_CRITICAL(&encMux);
  acc += encTransitions;
  encTransitions = 0;
  portEXIT_CRITICAL(&encMux);
  int32_t d = acc / ENC_TRANSITIONS_PER_DETENT;
  acc -= d * ENC_TRANSITIONS_PER_DETENT;
  return ENC_REVERSE ? -d : d;
}

static bool buttonPressed(uint32_t now) {
  static bool stable = HIGH, lastRead = HIGH;
  static uint32_t changedAt = 0;
  bool r = gpioLevel(PIN_ENC_BTN);
  if (r != lastRead) { lastRead = r; changedAt = now; }
  if ((now - changedAt) >= BTN_DEBOUNCE_MS && r != stable) {
    stable = r;
    if (stable == LOW) return true;   // press edge (active low)
  }
  return false;
}

static void postTarget(uint8_t ch, int32_t raw) {
  portENTER_CRITICAL(&shMux);
  shared.reqRaw[ch] = raw;
  shared.reqSeq[ch]++;
  portEXIT_CRITICAL(&shMux);
}

static void handleInput(uint32_t now) {
  int32_t d = takeDetents();
  if (d != 0) {
    TurnResult t = uiTurn(ui, d, now, UI_CFG);
    if (t.adjust) {
      int32_t sp = stepSetpoint(t.ch, setpoint[t.ch], t.detents, t.fast);
      if (sp != setpoint[t.ch]) {
        setpoint[t.ch] = sp;
        postTarget(t.ch, setpointToRaw(t.ch, sp));
      }
    }
  }
  if (buttonPressed(now)) uiPress(ui, now);
  uiTick(ui, now, UI_CFG);
}

static void drainRx() {
  static uint8_t buf[BS];
  for (;;) {
    portENTER_CRITICAL(&rxMux);
    int8_t n = vpw.tryGetReceivedFrame(buf);
    portEXIT_CRITICAL(&rxMux);
    if (n <= 0) break;
    rxFrames++;
    if (rxPrintEnabled) {
      Serial.print("[RX] ");
      for (int8_t k = 0; k < n; k++) Serial.printf("%02X ", buf[k]);
      Serial.println();
    }
  }
}

static void printSentOnce() {
  uint8_t f[BS + 1], n;
  portENTER_CRITICAL(&shMux);
  n = shared.sentOnceLen;
  if (n) memcpy(f, shared.sentOnce, n);
  shared.sentOnceLen = 0;
  portEXIT_CRITICAL(&shMux);
  if (!n) return;
  Serial.print("[TX once]");
  for (uint8_t k = 0; k < n; k++) Serial.printf(" %02X", f[k]);
  Serial.println();
}

// ---------------------------------------------------------------- LCD ------
static char lcdWant[4][21];
static char lcdShown[4][21];
static uint8_t lcdScan = 0;          // 0..79, next cell to compare
static int16_t lcdCursor = -1;       // cell the HD44780 cursor sits on, -1 unknown

// Filled right-pointing triangle for the edit marker (CGRAM slot 1).
static uint8_t GLYPH_EDIT[8] = {0b10000, 0b11000, 0b11100, 0b11110, 0b11100, 0b11000, 0b10000, 0b00000};

// Row = marker + the frame that actually went out (or, before the first send,
// the frame about to go).
static void buildLcdRows(uint32_t now) {
  uint8_t frames[CH_COUNT][8], lens[CH_COUNT];
  portENTER_CRITICAL(&shMux);
  memcpy(frames, shared.frame, sizeof(frames));
  memcpy(lens, shared.frameLen, sizeof(lens));
  portEXIT_CRITICAL(&shMux);
  for (uint8_t i = 0; i < CH_COUNT; i++) {
    if (lens[i] == 0) {
      uint8_t pl = buildPayload(i, setpointToRaw(i, setpoint[i]), frames[i]);
      frames[i][pl] = j1850Crc(frames[i], pl);
      lens[i] = pl + 1;
    }
    formatRow(uiMarker(ui, i, now, UI_CFG), frames[i], lens[i], lcdWant[i]);
  }
}

// Writes up to LCD_CHARS_PER_PASS changed cells, then returns.
static void lcdFlushSome() {
  if (!lcd) return;
  uint8_t written = 0;
  for (uint8_t scanned = 0; scanned < 80 && written < LCD_CHARS_PER_PASS; scanned++) {
    uint8_t cell = lcdScan;
    lcdScan = (uint8_t)((lcdScan + 1) % 80);
    uint8_t r = cell / 20, col = cell % 20;
    char w = lcdWant[r][col];
    if (w == lcdShown[r][col]) continue;
    if (lcdCursor != cell) lcd->setCursor(col, r);
    lcd->write((uint8_t)w);
    lcdShown[r][col] = w;
    // HD44780 20x4 auto-increment wraps row 0->2->1->3, so only trust it inside a row
    lcdCursor = (col == 19) ? -1 : cell + 1;
    written++;
  }
}

static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// ------------------------------------------------------- serial console ----
static int8_t hexNibble(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  ch = (char)toupper(ch);
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  return -1;
}

static void queueOneShot(const uint8_t* p, uint8_t n) {
  portENTER_CRITICAL(&shMux);
  memcpy(shared.oneShot, p, n);
  shared.oneShotLen = n;
  portEXIT_CRITICAL(&shMux);
}

static const char* uiStateName() {
  return ui.state == UI_PASSIVE ? "PASSIVE" : (ui.state == UI_SELECT ? "SELECT" : "EDIT");
}

static void valueText(uint8_t i, int32_t raw, char* out, size_t n) {
  switch (i) {
    case CH_FUEL:  snprintf(out, n, "%s", fuelNearestLabel(raw)); break;
    case CH_SPEED: snprintf(out, n, "%ldmph", (long)speedRawToMph(raw)); break;
    case CH_TEMP:  snprintf(out, n, "%ldF", (long)tempRawToF(raw)); break;
    default: {
      const char* g = "?";
      for (int32_t k = 0; k < GEAR_COUNT; k++) if (GEARS[k].raw == raw) g = GEARS[k].label;
      snprintf(out, n, "%s", g);
    }
  }
}

static void printStatus() {
  Shared s;
  portENTER_CRITICAL(&shMux);
  memcpy(&s, &shared, sizeof(s));
  portEXIT_CRITICAL(&shMux);
  Serial.printf("ui=%s row=%s heartbeat=%s rxprint=%s | GPIO ISR core=%d (turn the knob once; expect 0)\n",
                uiStateName(), CHANS[ui.sel].name, s.hbEnabled ? "on" : "off", rxPrintEnabled ? "on" : "off",
                (int)gpioIsrCore);
  for (uint8_t i = 0; i < CH_COUNT; i++) {
    char v[12];
    valueText(i, s.txRaw[i], v, sizeof(v));
    Serial.printf("  %-5s set=%ld on-wire~%-6s target=0x%04lX tx=0x%04lX %s wire:", CHANS[i].name,
                  (long)setpoint[i], v, (unsigned long)s.target[i], (unsigned long)s.txRaw[i],
                  s.ramping[i] ? "ramping" : "       ");
    for (uint8_t k = 0; k < s.frameLen[i]; k++) Serial.printf(" %02X", s.frame[i][k]);
    Serial.println();
  }
  Serial.printf("  tx ok=%lu busy=%lu arbLost=%lu other=%lu | rx frames=%lu errors=%lu (crc %lu)\n",
                (unsigned long)s.txOk, (unsigned long)s.txBusy, (unsigned long)s.txArb, (unsigned long)s.txOther,
                (unsigned long)rxFrames, (unsigned long)rxErrCount, (unsigned long)rxCrcErrCount);
}

static void printHelp() {
  Serial.println(F("commands:"));
  Serial.println(F("  stat             channel values, frames, counters"));
  Serial.println(F("  hb on|off        PCM node-alive E8 FF 10 03 every second"));
  Serial.println(F("  beam on|off      one high-beam frame (8A EA 40 A0/20 91 00)"));
  Serial.println(F("  tx <hex>         one raw frame, NO CRC (library appends it), max 11 bytes"));
  Serial.println(F("  rx on|off        print received frames"));
}

static void setHeartbeat(bool on) {
  portENTER_CRITICAL(&shMux);
  shared.hbEnabled = on;
  if (on) shared.hbKick = true;
  portEXIT_CRITICAL(&shMux);
  Serial.println(on ? "heartbeat on" : "heartbeat off");
}

static void handleCommand(char* line) {
  for (char* p = line; *p; p++) *p = (char)tolower(*p);
  if (!strcmp(line, "help") || !strcmp(line, "?")) { printHelp(); return; }
  if (!strcmp(line, "stat"))     { printStatus(); return; }
  if (!strcmp(line, "hb on"))    { setHeartbeat(true);  return; }
  if (!strcmp(line, "hb off"))   { setHeartbeat(false); return; }
  if (!strcmp(line, "rx on"))    { rxPrintEnabled = true;  return; }
  if (!strcmp(line, "rx off"))   { rxPrintEnabled = false; return; }
  if (!strcmp(line, "beam on"))  { queueOneShot(BEAM_ON_PAYLOAD,  sizeof(BEAM_ON_PAYLOAD));  return; }
  if (!strcmp(line, "beam off")) { queueOneShot(BEAM_OFF_PAYLOAD, sizeof(BEAM_OFF_PAYLOAD)); return; }
  if (!strncmp(line, "tx ", 3)) {
    uint8_t bytes[BS];
    uint8_t n = 0;
    int8_t hi = -1;
    for (char* p = line + 3; *p; p++) {
      if (*p == ' ') continue;
      int8_t v = hexNibble(*p);
      if (v < 0) { Serial.println("bad hex"); return; }
      if (hi < 0) { hi = v; continue; }
      if (n >= BS - 1) { Serial.printf("too long: max %d bytes before the CRC\n", BS - 1); return; }
      bytes[n++] = (uint8_t)((hi << 4) | v);
      hi = -1;
    }
    if (hi >= 0 || n < 2) { Serial.println("need whole bytes, at least 2"); return; }
    queueOneShot(bytes, n);
    return;
  }
  Serial.println("unknown, type help");
}

static void serviceSerial() {
  static char line[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { line[len] = 0; if (len) handleCommand(line); len = 0; continue; }
    if (len < sizeof(line) - 1) line[len++] = ch;
  }
}

static void uiTask(void*) {
  // The first attachInterrupt() installs the one GPIO interrupt service on
  // the calling core. Doing it here puts the encoder AND the library's RX
  // edge interrupt on core 0, so the bit-banging core never takes a GPIO
  // interrupt mid-symbol. send() later detaches/re-attaches RX from core 1;
  // that only edits the handler table, the interrupt stays on core 0.
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_BTN, INPUT_PULLUP);
  encPrev = readEncAB();
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderIsr, CHANGE);

  vpw.onError(onVpwError);
  vpw.init(PIN_VPW_RX, PIN_VPW_TX);

  Wire.begin(PIN_SDA, PIN_SCL);               // I2C interrupt on core 0 as well
  Wire.setClock(100000);
  const uint8_t neutral[] = {0x68, 0xEA, 0x10, 0x0A, 0x03};
  bool crcOk = j1850Crc(neutral, 5) == 0x94 && j1850Crc(HEARTBEAT_PAYLOAD, 4) == 0xB3 &&
               j1850Crc(neutral, 5) == crc((uint8_t*)neutral, 5);   // ours == library's
  Serial.printf("CRC self-test %s\n", crcOk ? "PASS" : "FAIL");
  if (i2cPresent(LCD_ADDR)) {
    lcd = new LiquidCrystal_I2C(LCD_ADDR, 20, 4);
    lcd->init();
    lcd->backlight();
    lcd->createChar(1, GLYPH_EDIT);
    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print(crcOk ? "CRC self-test OK" : "CRC self-test FAIL");
    Serial.printf("LCD at 0x%02X\n", LCD_ADDR);
  } else {
    Serial.printf("No LCD answering at 0x%02X. I2C scan:", LCD_ADDR);
    for (uint8_t a = 0x08; a < 0x78; a++) if (i2cPresent(a)) Serial.printf(" 0x%02X", a);
    Serial.println(" -- running without the LCD");
  }

  vpwReady = true;                            // bus task may start sending
  printHelp();

  if (lcd) { vTaskDelay(pdMS_TO_TICKS(600)); lcd->clear(); }
  memset(lcdShown, ' ', sizeof(lcdShown));    // matches the cleared screen
  memset(lcdWant, ' ', sizeof(lcdWant));
  ui.lastInputMs = millis();

  uint32_t lastLcdBuild = 0;
  UiState lastUiState = UI_PASSIVE;
  uint8_t lastSel = 0;
  for (;;) {
    uint32_t now = millis();
    handleInput(now);
    drainRx();
    printSentOnce();

    bool uiMoved = (ui.state != lastUiState) || (ui.sel != lastSel);
    if (uiMoved || now - lastLcdBuild >= LCD_REFRESH_MS) {
      lastLcdBuild = now;
      lastUiState = ui.state;
      lastSel = ui.sel;
      buildLcdRows(now);
    }
    lcdFlushSome();
    serviceSerial();
    vTaskDelay(1);                            // also feeds core 0's idle-task watchdog
  }
}

// --------------------------------------------------------------- setup -----
void setup() {
  // Hold the transmitter passive before anything else touches the pin.
  gpio_set_level((gpio_num_t)PIN_VPW_TX, VPW_TX_PASSIVE_LEVEL);

  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  Serial.setTxTimeoutMs(0);   // never stall when no terminal reads the USB port
#endif
  delay(200);
  Serial.printf("\nCluster trainer S3, reset reason %d\n", (int)esp_reset_reason());
#ifdef PROPOGATION_DELAY
  Serial.printf("VPW arbitration check delay %d us\n", (int)PROPOGATION_DELAY);
#endif
  Serial.printf("bus task on core %d, UI on core %d\n", (int)VPW_CORE, (int)UI_CORE);

  for (uint8_t i = 0; i < CH_COUNT; i++) {    // boot values
    setpoint[i] = bootSetpoint(i);
    shared.reqRaw[i] = setpointToRaw(i, setpoint[i]);
    shared.reqSeq[i] = 0;
  }
  shared.hbEnabled = true;

  xTaskCreatePinnedToCore(uiTask,  "ui",  6144, nullptr, UI_TASK_PRIO,  nullptr, UI_CORE);
  xTaskCreatePinnedToCore(vpwTask, "vpw", 4096, nullptr, VPW_TASK_PRIO, nullptr, VPW_CORE);
}

// loopTask (core 1, priority 1) has nothing to do; the vpw task outranks it.
void loop() {
  vTaskDelete(nullptr);
}
