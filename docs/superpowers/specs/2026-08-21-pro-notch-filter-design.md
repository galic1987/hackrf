# Sub-project 5: Programmable RX notch filter in standard gateware

**Date:** 2026-08-21
**Branch:** `upstream-pr-submit` (fork: galic1987/hackrf)
**Depends on:** sub-projects 1+2 (merged upstream, radio-register infrastructure, USB API 0x0115)
**Goal:** kill a narrowband in-band interferer (observed: fixed carrier at 1575.0925 MHz, ~34–44 dB above noise) in the FPGA before it consumes USB-bandwidth dynamic range and pollutes GNSS correlation — always on, no host CPU, available to every application.

## Background

Live evidence (2026-08-20/21): a rock-stable narrowband tone at 1575.0925 MHz (−327.5 kHz from GPS L1) compresses the RX chain enough to mask the bias-tee LNA swing and block GPS acquisition. A host-side FFT notch restored acquisition (PRN 9, metric 3.4–3.6). A digital notch cannot undo analog compression, but measurements show the front end stays sufficiently linear at the verified gain profile (lna 40 / vga 46 / amp off / bias on), so removing the tone digitally is sufficient.

## Scope

- Standard gateware image only (`top/standard.py`, image index 0). Half-precision and extended-precision images are untouched.
- RX path only. No TX changes.
- One programmable notch. No adaptive/multi-notch generality (YAGNI).

## Gateware (firmware/fpga)

New module `dsp/notch.py` — heterodyne notch built from existing blocks:

```
                 ┌── NCO(−f) ── leaky integrator (tone estimate at DC) ── NCO(+f) ──┐
 x ──[delay]─────┤                                                                ├──(−)──► y
                 └─────────────────────────────────────────────────────────────────┘
```

- **NCO**: existing `dsp/nco.py` (phase_width=24). One instance, time-shared phase or two phases from one accumulator; final resource count decided at build time (iCE40UP5K has 8 DSP MAC16 blocks; the half-band chain uses several — if over budget, fall back to a time-multiplexed mix).
- **Tone estimate**: leaky integrator `avg += (x − avg) >> ratio`, shift-only (no multiplier), ratio ≈ 2⁻¹⁴ → notch width a few hundred Hz at 40 MHz AFE rate.
- **Delay matching**: clean path delayed by the estimator's group delay (NCO 2 cycles + integrator + mix-back 2 cycles) before subtraction.
- **Placement**: in the RX chain after `quarter_shift`, before `hbfir5`, running at the AFE/ADC rate — notch frequency is then independent of the host sample rate / decimation setting.
- **Registers** (8-bit SPI register file, `firmware/common/fpga_regs.def`):
  - REG 7 `RX_NOTCH_CTRL` — bit 0: enable
  - REG 8/9/10 `RX_NOTCH_PSTEP` — 24-bit phase step, LSB-first
  - Resolution at 40 MHz AFE: 40e6 / 2²⁴ ≈ 2.4 Hz.

## Firmware (firmware/common)

- New radio register `RADIO_RX_NOTCH = 25`; `RADIO_NUM_REGS` → 26.
- Value semantics: signed int64 tone offset from the current LO in Hz; `0` = notch disabled. Independent of the tuning solver (same contract as `RADIO_TX_NCO`).
- New `radio_update_rx_notch()` in `radio.c` (Praline-only): converts Hz → pstep using the applied AFE rate (`pstep = round(hz · 2²⁴ / f_afe)`); bounds: reject (no-apply) if `|hz| ≥ f_afe/2` or pstep out of 24-bit range; writes FPGA regs 7–10; stores applied value.
- Add `RADIO_RX_NOTCH` to `radio_reapply_fpga_state()` so the notch is rewritten after a bitstream reload.
- USB API version `0x0115` → `0x0116` in `usb_descriptor.c`, `hackrf.h` version table, docs.

## Host (host/libhackrf, host/hackrf-tools)

- `hackrf_set_rx_notch(device, int64_t hz)` / `hackrf_get_rx_notch(device, int64_t* hz)` — transport via `hackrf_radio_read/write_register`, write to `RADIO_BANK_ALL` (persists across RX/TX/OFF), getter reads `RADIO_BANK_APPLIED`. Gated on API 0x0116 → `HACKRF_ERROR_USB_API_VERSION` on old firmware; old firmware STALLs register 25.
- `hackrf_pro --rx-notch HZ` (0 disables), with applied-value read-back print like the other managed options.
- Raw `--write-reg` path remains as debug escape hatch.

## Error handling

- Out-of-range offset → firmware no-apply, old value retained (mirrors TX NCO contract).
- NULL device / out-of-range on host → `HACKRF_ERROR_INVALID_PARAM`.
- Old firmware ↔ new host → clean API-version error, no silent failure.

## Testing

1. **Amaranth simulation** (`dsp/notch.py` unit sim): synthetic tone at notch frequency suppressed >40 dB; off-frequency tones and noise pass unaltered; enable/disable glitch-free.
2. **Host unit tests**: extend mock-libusb ctest suite — Hz→pstep encoding, bounds rejection, applied-bank read-back, API gate.
3. **Hardware-in-the-loop (Pro r1.2)**: live L1 capture with the real 1575.0925 MHz tone; notch OFF vs ON; measure tone drop in FFT; run `hackrf_gnss` acquisition and compare against the 2026-08-20 baseline (PRN 9 metric 3.4–3.6 with host-side notch).

## Build & deployment

- Toolchain self-contained on this disk: oss-cad-suite (yosys/nextpnr-ice40/icestorm) + Python venv with `requirements.txt` (amaranth v0.5.8) under `mac-archive/hackrf/tools/`. Nothing installed system-wide.
- Build only the `0_standard` image during development.
- Flash as user bitstream via `hackrf_debug -P` / SPI-flash image table; the stock bundled `praline_fpga.bin` stays untouched so a reset always recovers stock behavior.

## Out of scope

- ext_precision / half_precision images (later, if the standard-image notch proves out)
- Adaptive notch frequency estimation
- Host-side notch in hackrf_gnss (already demonstrated; remains as fallback)
- GNU Radio / SoapySDR bindings
