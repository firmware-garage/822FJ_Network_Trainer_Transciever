#!/usr/bin/env bash
# Host tests: no board needed. Pulls crc() verbatim out of the vendored library.
set -euo pipefail
cd "$(dirname "$0")"
LIB=../../lib/J1850VPW_ESP32/src/j1850vpw.cpp
awk '/^uint8_t crc\(uint8_t\* msg_buf, int8_t nbytes\)/{on=1} on{print} on&&/^}/{exit}' "$LIB" > lib_crc.inc
g++ -std=c++17 -O2 -Wall -Wextra -o /tmp/test_panel test_panel.cpp
/tmp/test_panel
