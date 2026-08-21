# Sub-project 2: Proper HackRF Pro FPGA host API

**Date:** 2026-08-21
**Branch:** `upstream-pr-submit` (fork: galic1987/hackrf)
**Depends on:** sub-project 1 (merged upstream, API 0x0114, both devices on 2026.01.3+ firmware)
**Goal:** high-level libhackrf API for Pro FPGA DSP features that cooperates with `radio.c`'s configuration management instead of being clobbered by it; `hackrf_pro` becomes a thin CLI over it.

## Background

The firmware radio-register interface (transport: `hackrf_radio_read/write_register`, API 0x0111) already manages: DC block (`RADIO_DC_BLOCK` 22), RX decimation / TX interpolation (`RADIO_RESAMPLE_RX/TX`), quarter-shift (`RADIO_ROTATION`, mode << 30), clock correction (`RADIO_CLOCK_CORRECTION` 23). `hackrf_pro` currently bypasses this with raw FPGA register pokes that `radio.c` overwrites on every sample-rate change. TX NCO has gateware support (reg 4 bit 0 enable, reg 6 pstep; `f_nco = pstep * dac_clk / 1024`) but no radio register.

## Firmware changes (one new register)

- `RADIO_TX_NCO = 24` in `firmware/common/radio.h`; `RADIO_NUM_REGS` → 25.
- Value semantics: desired NCO offset in Hz (uint64); `0` = NCO disabled. Not integrated into the tuning solver (unlike `RADIO_ROTATION`): independent digital offset, documented as `f_actual = f_tuned + f_nco`.
- New `radio_update_tx_nco()` in `firmware/common/radio.c` (praline-only, like `radio_update_dc_block`): converts Hz → pstep using the firmware's own `afe_rate` (the real DAC clock — eliminates host-side `--dac-clk` guessing); `pstep = round(freq * 1024 / afe_rate)`, reject (no-apply) if out of 1..255; writes `fpga_set_tx_nco_pstep()` + `fpga_set_tx_nco_enable(freq != 0)`; stores applied value in the applied bank.
- USB API version `0x0114` → `0x0115` in `usb_descriptor.c`; `hackrf.h` version-table entry (`## 0x0115 - hackrf_set_tx_nco`). Old firmware bounds-rejects register 24; host wrapper gates on API version for a clean error.

## libhackrf wrappers (public API, Doxygen-documented)

All use `hackrf_radio_read/write_register` transport; writes go to `RADIO_BANK_ALL` so settings persist across RX/TX/OFF switches (verify bank-copy semantics in `radio.c` during implementation; fall back to `RADIO_BANK_REQUESTED` + doc note if ALL misbehaves); getters read `RADIO_BANK_APPLIED`.

- `hackrf_set_dc_block(device, bool)` / `hackrf_get_dc_block`
- `hackrf_set_rx_decimation(device, log2)` / `hackrf_get_rx_decimation` — log2 in 0..5 (÷1..÷32)
- `hackrf_set_tx_interpolation(device, log2)` / `hackrf_get_tx_interpolation` — log2 in 0..3
- `hackrf_set_quarter_shift(device, mode)` / `hackrf_get_quarter_shift` — mode 0=none, 1=down, 2=up (host enum; encoded `mode << 30` into `RADIO_ROTATION` — mapping to gateware encodings 0x00/0x40/0xC0 verified during implementation)
- `hackrf_set_clock_correction(device, double ppm)` / `hackrf_get_clock_correction` — fp_1_63 encode/decode inside
- `hackrf_set_tx_nco(device, freq_hz)` / `hackrf_get_tx_nco` — `USB_API_REQUIRED(0x0115)`

Errors: `HACKRF_ERROR_INVALID_PARAM` (out-of-range, NULL device), `HACKRF_ERROR_USB_API_VERSION` (NCO on old firmware), transport errors propagated.

## hackrf_pro rewrite

Thin CLI over the wrappers:
- `--dc-block on|off`, `--decimation N`, `--interpolation N`, `--quarter-shift up|down|none` — now via managed API (survive sample-rate changes)
- `--nco-freq HZ` — via `hackrf_set_tx_nco`; **drop `--dac-clk`** (firmware computes pstep from `afe_rate`)
- `--clock-corr PPM` — new, via `hackrf_set_clock_correction`
- `--read-reg` / `--write-reg` — kept as raw FPGA debug access, with a printed warning that raw writes bypass radio management and may be overwritten
- Pro board gate stays

## Testing

- **Unit (mock-libusb, ctest):** new tests mirroring the existing 4-test harness — each wrapper's control-transfer path and value encoding (fp_1_63 round-trip, rotation << 30, decimation bounds).
- **Hardware (Pro r1.2):**
  - Set each feature, read back applied bank — values match
  - Change sample rate after setting decimation/DC block/quarter-shift → setting survives (regression test for the original clobber bug)
  - NCO: set known offset, verify `reg6 == round(freq * 1024 / afe_rate)` and enable bit
  - TX NCO offset observed on the One r9's RX as poor-man's spectrum check
- Zero-warning builds: host (clang) + firmware (xpack arm-none-eabi, UNIVERSAL image, built in /tmp/hackrf-fw due to space-in-path libopencm3 issue)
- Both devices reflashed (UNIVERSAL on Pro; HACKRF_ONE or UNIVERSAL on One r9)

## Out of scope

- NCO integration into the frequency tuning solver
- GNU Radio / SoapySDR bindings for the new API
- ext_precision_rx decoding (sub-project 3)
- Anything upstream-bound (GSG does not accept LLM-generated PRs; fork-only by user decision)
