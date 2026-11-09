# Local patches to the vendored library

Source: https://github.com/voodoomods/J1850-VPW-ESP32-Interface-GM-Class2
Commit: e9fe8eb (7 Dec 2024), the latest on the default branch when fetched on 9 Nov 2026.
It is VoodooMods' ESP32 adaptation of matafonoff's J1850-VPW-Arduino-Transceiver-Library.
License: Microsoft Public License (LICENSE in this folder, kept unchanged).

Only three changes, all marked `[cluster_trainer_s3 patch N]` in the source:

1. `PROPOGATION_DELAY`: moved from `src/j1850vpw.cpp` (upstream hard-codes 10 us) to `src/j1850vpw.h`, behind `#ifndef`, with a default of **18 us**. This is the same change, and the same value, as in Greg's validated firmware (github.com/firmware-garage/J1850VPW_Transceiver, which also leaves a commented-out 25 in the .cpp). `platformio.ini` can still override it, and a build error fires if it is 50 us or more. That guard is the only addition beyond Greg's version.
2. `src/pins.cpp`: the output pin is no longer forced LOW when it is created. With the library's default ACTIVE=LOW that briefly drove the bus active at boot. The sketch now pre-sets the latch to the passive level.
3. `library.properties`: `architectures=esp` changed to `esp32` so PlatformIO's library finder accepts it.

Nothing in the timing, the CRC, the receive decoder or the send loops was changed.

## Compared with Greg's validated copy

Greg's repo vendors the same upstream commit (every file except `j1850vpw.h` and `j1850vpw.cpp` is byte-identical apart from line endings, the schematic image included). His only change is patch 1 above. He did not change `pins.cpp` (patch 2), `library.properties` (patch 3), IRAM placement, the symbol timings, `RX_ARBITRATION_TOL`, or the CRC.
