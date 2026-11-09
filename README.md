# Cluster trainer, ESP32-S3 port

This firmware drives a GM Class 2 (J1850 VPW) instrument cluster's fuel, speed, coolant temperature and PRNDL gauges from one rotary encoder. A 20x4 LCD on an I2C backpack at 0x27 shows each gauge's frame exactly as it went out on the bus. The classic ESP32 build is untouched in `../cluster_trainer`.

**Status:** it compiles for real against the real VPW library, for both S3 boards, and 412 host tests pass. Nothing has run on hardware yet. Bus timing, arbitration and the LCD are all unproven until the bench checks at the end of this file.

## How it is built

It uses both cores, the same split as your web-injector build: one core does the VPW timing, the other does everything else.

- **Core 1, "vpw" task (priority 5):** scheduler, ramps and the library's bit-banged `send()`. Nothing else runs on this core, and no GPIO interrupt lands here, so only the FreeRTOS tick can interrupt a symbol.
- **Core 0, "ui" task:** encoder, button, LCD, serial console and received frames. It also owns the one shared GPIO interrupt service, which handles the encoder edges and the library's RX edges.
- The two tasks only share one struct, under one spinlock, and only to copy a few bytes. The bus core never touches Serial, I2C or the LCD.

**Why all GPIO interrupts end up on core 0:** Arduino installs a single GPIO interrupt service on whichever core calls `attachInterrupt()` first. The UI task does that first on core 0. ESP-IDF's `gpio_intr_enable()` and `gpio_isr_handler_add()` always enable on the service's core (checked in ESP-IDF 5.5.1's `gpio.c`; this build uses 5.5.5). So the detach and re-attach that `send()` does on core 1 every frame doesn't move the RX interrupt. The `stat` command prints which core the ISR actually ran on.

## The VPW library: what's in here and why

What the source on this machine shows:

- **`class2_sniffer`** has its own listen-only decoder. It uses edge timestamps in an IRAM interrupt, compensates for receiver release lag (25 us by default, adjustable with `skew`), and has no transmit code at all. It uses no library.
- **`sunfire_head_unit`** is a single-loop sketch with no VPW code.
- **The classic `cluster_trainer`** calls the matafonoff-style API, but it was only compile-checked against a stub. Its README says the patched library was never on this machine.
- **Your dual-core web-injector firmware** (VPW timing on one core, web page on the other) **is not on this machine.** I built the same split from your description, not from that source. If you still have it, its transmit loop and its `PROPOGATION_DELAY` value are worth comparing against this one.

The library in `lib/J1850VPW_ESP32` is the latest VoodooMods ESP32 fork of matafonoff's library (commit e9fe8eb, December 2024). It has three small changes, listed in `lib/J1850VPW_ESP32/PATCHES.md`:

1. `PROPOGATION_DELAY` can be set from `platformio.ini`.
2. The TX pin no longer drives the bus active at boot.
3. The architecture tag was fixed so PlatformIO accepts the library.

The timing, CRC, decoder and send loops were not changed.

**What a source review shows, before any bench test:**

- **CRC:** `send()` always appends the J1850 CRC-8 (poly 0x1D, init and final XOR 0xFF). The sketch passes payloads without the CRC. The host tests compare our CRC with the library's own `crc()`, copied verbatim, over 200,000 random frames: no mismatches.
- **Transmit:** it busy-waits on `micros()` using `digitalWrite`. The targets are 64 us and 128 us symbols, a 200 us SOF and a 280 us EOF. RX interrupts are detached during a send. On core 1 the only things left that can stretch a symbol are the tick interrupt and the occasional cross-core IPC.
- **Arbitration:** at each passive symbol it waits `PROPOGATION_DELAY` us, then checks that RX shows passive. If not, it stops with `J1850_ERR_ARBITRATION_LOST`. Upstream uses 10 us. Your sniffer found the receiver releases late, so 10 us would make every frame look like it lost arbitration. This build uses **40** (25 us lag plus 15 us margin), which still leaves 24 us of each short passive symbol for a real check. It's set in `platformio.ini` and needs tuning on the bench.
- **Bus idle:** before sending it waits up to `VPW_IDLE_WAIT_MS` (20 ms) for 300 us of idle. Otherwise it returns `J1850_ERR_BUS_IS_BUSY`, and the scheduler retries 15 ms later.
- **Receive:** decoding happens in the edge interrupt, on core 0 here. Frames go into a store that the library doesn't lock. The UI pops frames on the same core with that core's interrupts masked, which closes the race.
- **Known limits:** the library's RX interrupt (`__handleRnChange`) and `digitalRead`/`digitalWrite` are in flash, not IRAM (checked in the built ELF). That's only a problem if something writes flash while it runs, and this firmware has no Wi-Fi, NVS or OTA. Active level is one setting for both pins (default: LOW = active, which matches your sniffer's RX). Frames are capped at 12 bytes including the CRC.

## Board and pins

Default env `s3_supermini` targets an ESP32-S3 Super Mini (ESP32-S3FH4R2). Env `s3_n16r8` targets a DevKitC-1 N16R8. Both use the same pins, all from GPIO 1-13, which both boards bring out. **Check them against your board's silkscreen.** I couldn't confirm a Super Mini pinout from the maker.

| Signal | GPIO | Note |
|---|---|---|
| VPW RX (from transceiver) | 4 | library sets INPUT_PULLUP |
| VPW TX (to transceiver) | 5 | held passive (HIGH) from the first line of `setup()` |
| Encoder A / B | 6 / 7 | internal pull-ups |
| Encoder push button | 10 | to GND, internal pull-up |
| LCD SDA / SCL | 8 / 9 | also the S3 Arduino defaults |

Pins it avoids: 0, 3, 45 and 46 (strapping), 19 and 20 (USB), 26-37 (flash and PSRAM), 43 and 44 (UART0), and 48 (RGB LED). A `static_assert` refuses those pins if you remap.

**The S3 is not 5 V tolerant.** Most PCF8574 backpacks pull SDA and SCL up to their own VCC. Either power the backpack from 3.3 V (contrast may need a tweak), or use a level shifter or remove the backpack's pull-ups. The same applies to your transceiver's RX output.

Serial is the USB-C port (native USB CDC) at 115200. On the N16R8 board, use the port marked USB.

## Screen and controls

Each row is a marker column followed by the frame actually sent, CRC included, in uppercase hex with leading zeros:

```
 col 0   1-6       7   8 ...     space  CRC
  ' '    A88310        127A         B5        fuel   (15 columns)
  ' '    882911        020000       3B        speed  (17 columns)
  ' '    684910        104E         B5        temp   (100 F)
  ' '    68EA10        0A01         AE        PRNDL  (P)
```

The first group is priority, destination and source. Everything after the first space is data (message byte and value). The last group is the CRC. On the case, the priority/destination/source labels go under columns 1-6. Until a channel's first frame goes out, its row shows the frame about to be sent.

One encoder runs everything:

| State | Marker | Turn | Press |
|---|---|---|---|
| Passive | none | ignored | go to Select |
| Select | `>` blinking (400 ms) on one row | move between rows (wraps) | go to Edit |
| Edit | solid filled arrow on that row | change that row's value | back to Select |

After 10 seconds without a turn or press in Select or Edit, it goes back to Passive and the markers disappear. **That's my assumption, since you didn't give a timeout.** Change `idleTimeoutMs` in `UI_CFG`. The selected row is remembered for next time.

**What one click does in Edit.** Clockwise goes up. A "fast" click means one less than 40 ms after the last, or several in one pass.

| Row | Slow click | Fast click | Range | On the wire |
|---|---|---|---|---|
| Fuel | 2 raw counts | 8 | 0x00 (E) to 0xE2 (F) | raw byte. Only the five marks are known, so it steps raw |
| Speed | 1 mph | 5 mph | 0 to 120 mph | piecewise-linear through your bench table, so 25/45/65/85/100/120 come out exact. The 0 mph = 0x0000 point is assumed, not measured |
| Temp | 2 F | 10 F | 100 to 260 F | raw = degC + 40 (140 F = 0x64) |
| PRNDL | one gear | one gear | P R N D D3 D2 D1, stops at both ends | 01 02 03 09 08 07 06 |

Fuel and speed glide to a new value over 2 seconds, starting from the value last sent, so the needle never jumps. Temperature and gear follow the knob immediately. Turning only ever changes the value bytes. Header and message bytes are fixed.

## Bus schedule (unchanged from the classic build)

- **Repeat rates:** fuel, temp and PRNDL every 500 ms, speed every 200 ms. A channel that's ramping sends every 100 ms.
- **Heartbeat:** `E8 FF 10 03` (CRC B3) every second.
- **Spacing and order:** at least 10 ms between any two of our frames and 50 ms between repeats of the same frame. The most overdue frame goes first, and one frame goes per pass.
- **On a busy bus or lost arbitration:** retry after 15 ms.

## Serial console

- `stat`: each channel's knob value, target, raw value sent, last frame and ramp state. Also TX ok/busy/lost-arbitration/other counts, RX frame and error counts, and which core the GPIO ISR ran on.
- `hb on` / `hb off`: turn the heartbeat on or off.
- `beam on` / `beam off`: send one high-beam frame.
- `tx <hex>`: send one raw frame. Don't include the CRC, the library adds it. At least 2 and at most 11 bytes.
- `rx on` / `rx off`: print received frames.

## What has been tested

Software, on this machine:

- `pio run -e s3_supermini` and `pio run -e s3_n16r8`: both SUCCESS against the real vendored library. Arduino-ESP32 core 3.3.12 / ESP-IDF 5.5.5, pioarduino platform 55.03.312. No warnings from `src/`. The remaining warnings come from the upstream library's volatile style and the LCD library's `B0000` constants.
- In the ELF: `encoderIsr` and `onVpwError` are in IRAM, and `J1850VPW::send` and `init` are the real library code.
- `./test/host/run.sh`: 412 checks, 0 failed. They cover CRC vectors (neutral 94, heartbeat B3, high beam 5B/9B), the 200,000-frame match against the library's `crc()`, frame lengths (6 and 7), headers that never change, speed table points with monotonic round trips, temp and fuel steps and limits, gear order and end stops, exact LCD row text and widths, and every UI state change, including the timeout and `millis()` rollover.

Not tested: anything on the board, the bus or the LCD.

## Bench checks, in order

1. **Boot without the bus connected.** The serial log shows `CRC self-test PASS`, `LCD at 0x27` and `bus task on core 1, UI on core 0`. The LCD shows four rows of hex. If the LCD isn't found, the log lists the addresses that did answer.
2. **Knob and states.** Press once: a blinking `>` appears. Turn: it moves between rows. Press again: it changes to the solid arrow and turning changes the hex. Press: back to blinking. Wait 10 s: markers disappear. Then type `stat` and look for `GPIO ISR core=0`.
3. **Idle TX level.** With the transceiver connected but the cluster unplugged, the TX pin should sit HIGH. Put the scope on the bus while the board resets: there should be no active pulse during boot.
4. **Symbol timing.** Scope the bus while `hb on`. Short symbols should be 64 us (+/- about 15 is fine for receivers), long symbols 128 us, SOF 200 us, EOF 280 us or more. Do it again while spinning the knob and while `rx on` floods the console: the widths should not change.
5. **Arbitration delay.** With only the trainer on the bus, `stat` should show `arbLost` staying at 0. If it climbs, the receiver lags more than 40 us. Measure the lag with the sniffer's `hist` and set `PROPOGATION_DELAY` to that plus 15. If it stays at 0 even with a second node talking over us, try lowering it.
6. **Round trip.** Run the sniffer (or `rx on` on a second board) and check that each frame matches its LCD row byte for byte, CRC included.
7. **Cluster.** Sweep each gauge through its full range, then compare 25/45/65/85/100/120 mph and 140 F against the bench table.

## Build

```
pio run                 # Super Mini
pio run -e s3_n16r8     # DevKitC-1 N16R8
pio run -t upload       # flash (hold BOOT while plugging in if the port won't appear)
./test/host/run.sh      # host tests, no board needed
```

# Acknowledgments
Firmware design, code review and documentation were developed with Maya, an AI agent by Sesame.
