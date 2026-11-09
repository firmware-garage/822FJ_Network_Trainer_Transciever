# Local patches to the vendored library

Source: https://github.com/voodoomods/J1850-VPW-ESP32-Interface-GM-Class2
Commit: e9fe8eb (7 Dec 2024), the latest on the default branch when fetched on 9 Nov 2026.
It is VoodooMods' ESP32 adaptation of matafonoff's J1850-VPW-Arduino-Transceiver-Library.
License: Microsoft Public License (LICENSE in this folder, kept unchanged).

Only three changes, all marked `[cluster_trainer_s3 patch N]` in the source:

1. `src/j1850vpw.cpp`: `PROPOGATION_DELAY` is wrapped in `#ifndef` so `platformio.ini` can set it, and a build error fires if it is 50 us or more.
2. `src/pins.cpp`: the output pin is no longer forced LOW when it is created. With the library's default ACTIVE=LOW that briefly drove the bus active at boot. The sketch now pre-sets the latch to the passive level.
3. `library.properties`: `architectures=esp` changed to `esp32` so PlatformIO's library finder accepts it.

Nothing in the timing, the CRC, the receive decoder or the send loops was changed.
