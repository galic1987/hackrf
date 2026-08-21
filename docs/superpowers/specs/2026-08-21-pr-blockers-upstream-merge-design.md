# Sub-project 1: PR blockers + upstream merge

**Date:** 2026-08-21
**Branch:** `upstream-pr-submit` (fork: galic1987/hackrf)
**Goal:** branch compiles and runs against current upstream `origin/main` (55b1b219); semantic collisions resolved; known bugs in the fork's headline features fixed.
**Non-goal:** PR-submittable linear history (user chose merge; a clean cherry-pick branch can be made later if the PR goes upstream).

## Step 1 — Merge

`git merge origin/main` into `upstream-pr-submit`.

Predicted conflicts (from review):

- `host/libhackrf/src/hackrf.c` — FPGA timeout: keep upstream's `FPGA_BITSTREAM_TIMEOUT 500` (PR #1801), drop the fork's 1000ms hunk.
- `firmware/common/rffc5071.c` — delay API + `spi_bus_transfer` signature changes around `rffc5071_lock_test()`: keep upstream's modernization, re-apply the fork's `#ifdef IS_NOT_RAD1O` guard and mixer-ID-4544 acceptance.

If conflicts appear beyond these two files, stop and report before resolving.

## Step 2 — Semantic fixes (post-merge compile-breakers)

- `firmware/hackrf_usb/usb_api_sync.c:65`: `RADIO_BANK_ACTIVE` → `RADIO_BANK_REQUESTED` (upstream rename, commit 2b7ba8d7).
- USB API version `0x0113` → `0x0114` (upstream took 0x0113 for `RADIO_CLOCK_CORRECTION`, commit 8bfbd716): `usb_descriptor.c`, `hackrf.c` `USB_API_REQUIRED` sites, `hackrf.h` version table, docs. Vendor request 62 stays (free upstream).

## Step 3 — Bug fixes

1. `host/hackrf-tools/src/hackrf_pro.c`:
   - `--quarter-shift`: write `0x00` (none) / `0x40` (down) / `0xC0` (up), per `firmware/common/fpga.c:110-116` and gateware `rx_pstep[6]=enable, [7]=up`. Current 0/1/2 never enables the shift.
   - `--nco-freq HZ`: convert Hz → pstep via `round(hz * 1024 / dac_clk)` with bounds check (pstep is 8-bit; actual NCO freq = `pstep * dac_clk / 1024`). Current code writes `hz & 0xFF`.
   - Add `hackrf_board_id_read` gate: refuse to run on non-Pro boards.
2. `firmware/hackrf_usb/usb_api_sync.c`: OFF path disarms `RADIO_TRIGGER` (write 0) so a later plain `hackrf_start_rx()` doesn't hang waiting for a trigger edge. Document recovery in `docs/source/hardware_triggering.rst`.
3. `host/libhackrf/src/hackrf.c` `hackrf_close`: remove the post-`free()` check (use-after-free at hackrf.c:2518-2524). `open_devices` decremented only for a handle that was counted; NULL returns `HACKRF_ERROR_INVALID_PARAM`. Double-close of a freed handle remains a caller bug (upstream semantics).
4. `firmware/common/cpld_xc2c.c` + `firmware/hackrf_usb/usb_api_cpld.c`: propagate JTAG read failure — checksum returns false on read failure, USB handler STALLs instead of presenting a garbage CRC as success.

## Step 4 — Verify

- Firmware build (arm-none-eabi) + host build (cmake), zero warnings on macOS arm64.
- `ctest` mock-libusb suite (4 tests) passes.
- clang-format clean on touched files.
- Hardware-in-the-loop (user approved; requires HackRF Pro r1.2 + One r9 attached):
  - `hackrf_debug -k` CPLD checksum on the One r9.
  - FPGA guard: reject in RX, accept in OFF (Pro).
  - sync-start arm/disarm + plain `start_rx` works after `sync_start(OFF)`.
  - `hackrf_pro` quarter-shift/NCO register writes read back correctly.

## Out of scope (later sub-projects)

2. Proper Pro FPGA host API (DC block/decimation/NCO surviving sample-rate changes)
3. ext_precision_rx 16-bit host decoder
4. Coherent beamforming (sync-start + `-H` trigger + RADIO_CLOCK_CORRECTION)
