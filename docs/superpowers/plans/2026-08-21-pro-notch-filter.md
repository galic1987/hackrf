# HackRF Pro Programmable RX Notch Filter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a host-programmable narrowband notch filter to the HackRF Pro standard FPGA gateware so the 1575.0925 MHz interferer (and any future fixed tone) can be removed in hardware before it reaches USB.

**Architecture:** Heterodyne notch in the standard gateware RX chain (after `quarter_shift`, before `hbfir5`, at AFE rate): mix the tone to DC with the existing NCO module, estimate it with a shift-only leaky integrator, mix back up, subtract from the delayed input. Host programs the tone offset in Hz via a new radio register `RADIO_RX_NOTCH = 25`; firmware converts Hz → 24-bit NCO phase step from the real AFE clock (same contract as `RADIO_TX_NCO`).

**Tech Stack:** Amaranth HDL v0.5.8 (pinned in `firmware/fpga/requirements.txt`), yosys/nextpnr-ice40/icestorm via oss-cad-suite, arm-none-eabi-gcc (xpack 15.2.1), C11 firmware, libhackrf C API, ctest mock-libusb suite.

**Spec:** `docs/superpowers/specs/2026-08-21-pro-notch-filter-design.md`

## Global Constraints

- Repo: `/Volumes/Radiator 8TB/mac-archive/hackrf`, branch `upstream-pr-submit`, fork remote `galic1987`.
- **The repo path contains a space** (`Radiator 8TB`): the firmware build breaks in-tree (libopencm3). Always build firmware in a `/tmp/hackrf-fw` copy (rsync, see Task 7).
- FPGA toolchain and Python venv live under `mac-archive/hackrf/tools/` — never installed system-wide; never committed (`.gitignore`).
- Firmware: no floating point (upstream commit d5361ea5 "Don't use doubles in firmware").
- All new code follows `.clang-format` style manually (clang-format v22 on this machine is incompatible with the project config — do not run `clang-format -i`).
- Gateware builds produce all 4 images (`build.py` builds the full table); the image table indices must stay 0=standard, 1=half_precision, 2=ext_precision_rx, 3=ext_precision_tx.
- Do not run `clang-format -i`. Do not commit `tools/`, build outputs, or data files.
- HackRF Pro serial: `0000000000000000977c64de2b557213`; HackRF One serial: `0000000000000000922c63dc21748847` (leave the One alone unless a step says otherwise).

---

### Task 1: FPGA toolchain on disk + stock gateware baseline

**Files:**
- Create: `tools/oss-cad-suite/` (downloaded, git-ignored), `tools/venv-fpga/` (git-ignored)
- Modify: `.gitignore`

**Interfaces:**
- Produces: working `python3 build.py` in `firmware/fpga/` and a nextpnr resource-usage baseline for later comparison.

- [ ] **Step 1: Download and unpack oss-cad-suite**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf"
mkdir -p tools
cd tools
curl -sL -o oss-cad-suite.tgz "$(curl -sL https://api.github.com/repos/YosysHQ/oss-cad-suite-build/releases/latest | python3 -c "import json,sys; d=json.load(sys.stdin); print([a['browser_download_url'] for a in d['assets'] if 'darwin-arm64' in a['name']][0])")"
tar xzf oss-cad-suite.tgz
rm oss-cad-suite.tgz
./oss-cad-suite/bin/yosys --version
```

Expected: yosys version prints. (Download is ~400 MB.)

- [ ] **Step 2: Create the Amaranth venv**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf"
python3 -m venv tools/venv-fpga
tools/venv-fpga/bin/pip install -r firmware/fpga/requirements.txt
```

Expected: `amaranth==v0.5.8` installed.

- [ ] **Step 3: Git-ignore the toolchain**

Add to `.gitignore` (repo root), one line:

```
tools/
```

- [ ] **Step 4: Build stock gateware and record the resource baseline**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf/firmware/fpga"
export PATH="/Volumes/Radiator 8TB/mac-archive/hackrf/tools/oss-cad-suite/bin:$PATH"
../../tools/venv-fpga/bin/python build.py 2>&1 | tee /tmp/gw_build_baseline.log
```

Expected: completes with `praline_fpga.bin` regenerated; no errors. Record the standard-image utilization for later comparison:

```bash
grep -A8 "Device utilisation" /tmp/gw_build_baseline.log | head -40
```

If the build fails on the stock tree, STOP — the toolchain is wrong, do not proceed.

- [ ] **Step 5: Commit**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf"
git add .gitignore
git commit -m "gitignore: exclude tools/ (FPGA toolchain lives there)"
```

---

### Task 2: Notch DSP module + Amaranth simulation

**Files:**
- Create: `firmware/fpga/dsp/notch.py`
- Create: `firmware/fpga/tests/test_notch.py`

**Interfaces:**
- Consumes: `dsp/nco.py` `NCO(phase_width, output_width, domain)` — `phase: In(phase_width)`, `en: In(1)`, `output: Out(IQSample(output_width))`, latency 2 cycles. `util.IQSample(width)` — struct with `.i`/`.q` signed fields; stream signatures are `stream.Signature(IQSample(w), always_ready=True)`.
- Produces: `Notch(width=8, ratio=14, phase_width=24, domain="sync")` with members `input`, `output` (IQSample streams), `enable: In(1)`, `pstep: In(signed(phase_width))`. Task 3 wires `enable`/`pstep` to SPI registers.

- [ ] **Step 1: Write the failing simulation test**

Create `firmware/fpga/tests/test_notch.py`:

```python
#
# This file is part of HackRF.
#
# Copyright (c) 2026 Great Scott Gadgets
# SPDX-License-Identifier: BSD-3-Clause

import numpy as np
from amaranth.sim import Simulator

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dsp.notch import Notch


def run_notch(pstep, n_samples=65536, tone_amp=60.0, seed=7, ratio=14):
    """Drive the notch with tone + noise, return (input_rms, output_rms)."""
    rng = np.random.default_rng(seed)
    tone_i = tone_amp * np.cos(2 * np.pi * pstep / 2**24 * np.arange(n_samples))
    tone_q = tone_amp * np.sin(2 * np.pi * pstep / 2**24 * np.arange(n_samples))
    noise_i = rng.normal(0, 3.0, n_samples)
    noise_q = rng.normal(0, 3.0, n_samples)
    xi = np.clip(np.round(tone_i + noise_i), -128, 127).astype(int)
    xq = np.clip(np.round(tone_q + noise_q), -128, 127).astype(int)

    dut = Notch(width=8, ratio=ratio)
    out_i, out_q = [], []

    async def drive(ctx):
        for k in range(n_samples):
            ctx.set(dut.input.p.i, int(xi[k]))
            ctx.set(dut.input.p.q, int(xq[k]))
            ctx.set(dut.input.valid, 1)
            await ctx.tick()
        ctx.set(dut.input.valid, 0)

    async def collect(ctx):
        while len(out_i) < n_samples:
            await ctx.tick()
            if ctx.get(dut.output.valid):
                out_i.append(ctx.get(dut.output.p.i).astype(np.int32))
                out_q.append(ctx.get(dut.output.p.q).astype(np.int32))

    sim = Simulator(dut)
    sim.add_process(drive)
    sim.add_process(collect)
    with sim.write_vcd("/tmp/notch_sim.vcd"):
        sim.run()

    # output p.i/p.q are signed Signals: ctx.get returns raw unsigned; fix up
    out_i = np.array([v - 256 if v > 127 else v for v in out_i])
    out_q = np.array([v - 256 if v > 127 else v for v in out_q])
    skip = 8192  # integrator settling
    in_rms = np.sqrt(np.mean(xi[skip:].astype(float) ** 2 + xq[skip:].astype(float) ** 2))
    out_rms = np.sqrt(np.mean(out_i[skip:] ** 2 + out_q[skip:] ** 2))
    return in_rms, out_rms


def test_notch_suppresses_tone():
    # pstep for a tone at fs * 0.02 (positive offset)
    pstep = int(0.02 * 2**24)
    in_rms, out_rms = run_notch(pstep)
    suppression_db = 20 * np.log10(in_rms / out_rms)
    print(f"tone suppression: {suppression_db:.1f} dB (in {in_rms:.1f} -> out {out_rms:.1f})")
    assert suppression_db > 40.0


def test_notch_passes_other_frequencies():
    # notch at +0.02 fs, tone at -0.03 fs (must survive)
    rng = np.random.default_rng(3)
    n = 65536
    f = -0.03
    xi = np.round(60 * np.cos(2 * np.pi * f * np.arange(n))).astype(int)
    xq = np.round(60 * np.sin(2 * np.pi * f * np.arange(n))).astype(int)
    dut = Notch(width=8, ratio=14)
    out_i = []

    async def drive(ctx):
        ctx.set(dut.pstep, int(0.02 * 2**24))
        ctx.set(dut.enable, 1)
        for k in range(n):
            ctx.set(dut.input.p.i, int(xi[k]))
            ctx.set(dut.input.p.q, int(xq[k]))
            ctx.set(dut.input.valid, 1)
            await ctx.tick()
        ctx.set(dut.input.valid, 0)

    async def collect(ctx):
        while len(out_i) < n:
            await ctx.tick()
            if ctx.get(dut.output.valid):
                v = ctx.get(dut.output.p.i)
                out_i.append(v - 256 if v > 127 else v)

    sim = Simulator(dut)
    sim.add_process(drive)
    sim.add_process(collect)
    sim.run()

    skip = 8192
    in_rms = np.sqrt(np.mean(xi[skip:].astype(float) ** 2))
    out_rms = np.sqrt(np.mean(np.array(out_i[skip:]) ** 2))
    loss_db = 20 * np.log10(in_rms / out_rms)
    print(f"passband loss: {loss_db:.2f} dB")
    assert loss_db < 1.0
```

Note: the first test must set `dut.pstep`/`dut.enable` in its drive process too (add `ctx.set(dut.pstep, pstep); ctx.set(dut.enable, 1)` at the top of `drive` in `run_notch`).

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf/firmware/fpga"
../../tools/venv-fpga/bin/python -m pytest tests/test_notch.py -v
```

Expected: FAIL — `ModuleNotFoundError: dsp.notch`.

- [ ] **Step 3: Implement `dsp/notch.py`**

Create `firmware/fpga/dsp/notch.py`:

```python
#
# This file is part of HackRF.
#
# Copyright (c) 2026 Great Scott Gadgets
# SPDX-License-Identifier: BSD-3-Clause

from amaranth               import Module, Signal, signed
from amaranth.lib           import wiring, stream
from amaranth.lib.wiring    import In, Out

from dsp.nco                import NCO
from util                   import IQSample


class Notch(wiring.Component):
    """
    Programmable narrowband notch filter.

    Mixes the input down by the notch frequency, estimates the tone with a
    leaky integrator (shift-only, leakage 2**-ratio), mixes the estimate
    back up, and subtracts it from the delayed input:

        y[n] = x[n-D] - est[n]

    Latency of the estimate path: NCO (2) + mix-down (1) + integrator (1)
    + mix-up (1) = 5 cycles.

    Parameters
    ----------
    width : int
        Bit width of the I/Q samples.
    ratio : int
        Integrator leakage shift. Notch bandwidth ~ fs * 2**-ratio / (2*pi).
    phase_width : int
        NCO phase accumulator width; notch frequency = pstep * fs / 2**phase_width.
    """

    ESTIMATE_DELAY = 5

    def __init__(self, width=8, ratio=14, phase_width=24, domain="sync"):
        self.width = width
        self.ratio = ratio
        self.phase_width = phase_width
        self._domain = domain
        sig = stream.Signature(IQSample(width), always_ready=True)
        super().__init__({
            "input":  In(sig),
            "output": Out(sig),
            "enable": In(1),
            "pstep":  In(signed(phase_width)),
        })

    def elaborate(self, platform):
        m = Module()
        w = self.width
        nco_w = 10  # NCO output width; cos/sin scaled to +-511

        m.submodules.nco = nco = NCO(phase_width=self.phase_width,
                                     output_width=nco_w,
                                     domain=self._domain)
        dom = self._domain

        # Phase accumulator.
        phase = Signal(self.phase_width)
        m.d[dom] += phase.eq(phase + self.pstep)
        m.d.comb += [
            nco.phase.eq(phase),
            nco.en.eq(self.enable & self.input.valid),
        ]

        # Delay the clean input to align with the estimate.
        x_i = Signal(signed(w))
        x_q = Signal(signed(w))
        m.d.comb += [
            x_i.eq(self.input.p.i),
            x_q.eq(self.input.p.q),
        ]
        for _ in range(self.ESTIMATE_DELAY):
            d_i = Signal(signed(w))
            d_q = Signal(signed(w))
            m.d[dom] += [d_i.eq(x_i), d_q.eq(x_q)]
            x_i, x_q = d_i, d_q

        # Mix down: xd = x * conj(cos, sin). NCO output has 2-cycle latency.
        c0 = Signal(signed(nco_w))
        s0 = Signal(signed(nco_w))
        m.d.comb += [c0.eq(nco.output.i), s0.eq(nco.output.q)]
        xd_i = Signal(signed(w + nco_w))
        xd_q = Signal(signed(w + nco_w))
        xi_d = Signal(signed(w))   # input delayed to NCO output time
        xq_d = Signal(signed(w))
        m.d[dom] += [
            xi_d.eq(self.input.p.i),
            xq_d.eq(self.input.p.q),
        ]
        xi_d2 = Signal(signed(w))
        xq_d2 = Signal(signed(w))
        m.d[dom] += [xi_d2.eq(xi_d), xq_d2.eq(xq_d)]
        m.d[dom] += [
            xd_i.eq((xi_d2 * c0 + xq_d2 * s0) >> (nco_w - 1)),
            xd_q.eq((xq_d2 * c0 - xi_d2 * s0) >> (nco_w - 1)),
        ]

        # Leaky integrator (tone estimate at DC), extended precision.
        ew = w + self.ratio
        avg_i = Signal(signed(ew))
        avg_q = Signal(signed(ew))
        m.d[dom] += [
            avg_i.eq(avg_i + ((Cat(xd_i, Signal(self.ratio)) - avg_i) >> self.ratio)),
            avg_q.eq(avg_q + ((Cat(xd_q, Signal(self.ratio)) - avg_q) >> self.ratio)),
        ]

        # NCO output delayed by (mix-down 1 + integrator 1) = 2 cycles
        # for the mix-up stage.
        c1 = Signal(signed(nco_w)); s1 = Signal(signed(nco_w))
        c2 = Signal(signed(nco_w)); s2 = Signal(signed(nco_w))
        m.d[dom] += [c1.eq(c0), s1.eq(s0), c2.eq(c1), s2.eq(s1)]

        # Mix up: est = avg * (cos, sin), dropping the fractional bits.
        est_i = Signal(signed(w + 2))
        est_q = Signal(signed(w + 2))
        avg_i_r = Signal(signed(w + 2))
        avg_q_r = Signal(signed(w + 2))
        m.d.comb += [
            avg_i_r.eq(avg_i >> self.ratio),
            avg_q_r.eq(avg_q >> self.ratio),
        ]
        m.d[dom] += [
            est_i.eq((avg_i_r * c2 - avg_q_r * s2) >> (nco_w - 1)),
            est_q.eq((avg_i_r * s2 + avg_q_r * c2) >> (nco_w - 1)),
        ]

        # Subtract, with saturation to the sample width.
        y_i = Signal(signed(w + 2))
        y_q = Signal(signed(w + 2))
        m.d.comb += [y_i.eq(x_i - est_i), y_q.eq(x_q - est_q)]
        lo = -(1 << (w - 1))
        hi = (1 << (w - 1)) - 1
        m.d[dom] += [
            self.output.p.i.eq(Mux(y_i > hi, hi, Mux(y_i < lo, lo, y_i))),
            self.output.p.q.eq(Mux(y_q > hi, hi, Mux(y_q < lo, lo, y_q))),
            self.output.valid.eq(self.input.valid),
        ]

        # Bypass when disabled: subtract zero.
        # (Estimator keeps running so re-enable is glitch-free.)

        return m
```

Add the missing `Mux` import: `from amaranth import Module, Signal, signed, Mux, Cat`.

Note for the implementer: the exact cycle alignment between the estimate and the clean-path delay is what the simulation verifies. If `test_notch_suppresses_tone` shows < 40 dB, adjust `ESTIMATE_DELAY` by ±1 and the `c2/s2` delay chain correspondingly until the sim passes — that is the intended tuning loop, not a plan defect.

- [ ] **Step 4: Run the simulation until it passes**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf/firmware/fpga"
../../tools/venv-fpga/bin/python -m pytest tests/test_notch.py -v -s
```

Expected: both tests PASS; "tone suppression: >40 dB", "passband loss: <1 dB".

- [ ] **Step 5: Commit**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf"
git add firmware/fpga/dsp/notch.py firmware/fpga/tests/test_notch.py
git commit -m "fpga/dsp: programmable heterodyne notch filter with simulation tests"
```

---

### Task 3: Wire the notch into the standard gateware

**Files:**
- Modify: `firmware/fpga/top/standard.py`

**Interfaces:**
- Consumes: `Notch` from Task 2. SPI register pattern from `SPIRegisterInterface.add_register(address, init=, size=)`; CDC pattern from `cdc.FFSynchronizer(sig_in, sig_out, o_domain=...)`.
- Produces: SPI registers `0x07` (enable, 1 bit), `0x08/0x09/0x0A` (pstep bytes, LSB first) — the firmware in Task 4 writes exactly these addresses.

- [ ] **Step 1: Modify `top/standard.py`**

In the imports, add:

```python
from dsp.notch              import Notch
```

In the `rx_chain` dict (currently: `dc_block`, `quarter_shift`, `hbfir5`..`hbfir1`, `clkconv`), insert the notch after `quarter_shift`:

```python
            "notch":         DomainRenamer(adc_clk)(Notch(width=8, ratio=14, domain=adc_clk)),
```

In the SPI register block (after the `tx_pstep` line), add:

```python
        rx_notch_ctrl  = spi_regs.add_register(0x07, init=0, size=1)
        rx_notch_pstep_l = spi_regs.add_register(0x08, init=0)
        rx_notch_pstep_m = spi_regs.add_register(0x09, init=0)
        rx_notch_pstep_h = spi_regs.add_register(0x0a, init=0)
```

After the existing `tx_pstep` CDC block, add the notch control CDC and wiring:

```python
        # RX notch filter control (sync -> adc_clk domain).
        notch_pstep_sync = Signal(24)
        m.d.comb += notch_pstep_sync.eq(Cat(rx_notch_pstep_l, rx_notch_pstep_m, rx_notch_pstep_h))
        notch_en_adclk = Signal()
        notch_pstep_adclk = Signal(signed(24))
        m.submodules.notch_en_cdc = cdc.FFSynchronizer(rx_notch_ctrl, notch_en_adclk, o_domain=adc_clk)
        m.submodules.notch_pstep_cdc = cdc.FFSynchronizer(notch_pstep_sync, notch_pstep_adclk, o_domain=adc_clk)
        m.d.comb += [
            rx_chain["notch"].enable .eq(notch_en_adclk),
            rx_chain["notch"].pstep  .eq(notch_pstep_adclk),
        ]
```

`Cat` and `signed` are already imported in `top/standard.py` via `from amaranth import ...` — verify and add if missing.

- [ ] **Step 2: Rebuild the gateware and check resources**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf/firmware/fpga"
export PATH="/Volumes/Radiator 8TB/mac-archive/hackrf/tools/oss-cad-suite/bin:$PATH"
../../tools/venv-fpga/bin/python build.py 2>&1 | tee /tmp/gw_build_notch.log
```

Expected: builds clean. Compare against the Task 1 baseline:

```bash
grep -A8 "Device utilisation" /tmp/gw_build_notch.log | head -40
```

PASS criterion: standard image still fits the iCE40UP5K (nextpnr reports success, no placement failure). If SB_MAC16 or LUT budget overflows, STOP and report the utilisation numbers — the fallback is a time-multiplexed mix (needs a design revisit, do not improvise).

- [ ] **Step 3: Commit**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf"
git add firmware/fpga/top/standard.py
git commit -m "fpga: wire programmable notch into standard RX chain (regs 0x07-0x0a)"
```

---

### Task 4: Firmware plumbing (radio register + FPGA register definitions)

**Files:**
- Modify: `firmware/common/fpga_regs.def` (append after REG 06 block)
- Modify: `firmware/common/fpga.h`, `firmware/common/fpga.c` (add setters, near `fpga_set_rx_quarter_shift_mode` at fpga.c:105)
- Modify: `firmware/common/radio.h` (register enum + `RADIO_NUM_REGS`)
- Modify: `firmware/common/radio.c` (`radio_update_rx_notch`, call site in `radio_update`, reapply list in `radio_reapply_fpga_state`)
- Modify: `firmware/hackrf_usb/usb_descriptor.c:31` (API version)

**Interfaces:**
- Consumes: Task 3's SPI register map (0x07 enable bit, 0x08-0x0A pstep LSB-first).
- Produces: radio register `RADIO_RX_NOTCH = 25` (signed int64 Hz offset from LO, 0 = disabled); firmware writes FPGA regs 0x07-0x0A. USB API version `0x0116`.

- [ ] **Step 1: FPGA register definitions**

Append to `firmware/common/fpga_regs.def` after the REG 06 block:

```c
/* REG 07 (7): RX_NOTCH_CTRL */
__MREG__(FPGA_STANDARD_RX_NOTCH_EN, 7, 0, 1)

/* REG 08-0A (8-10): RX_NOTCH_PSTEP (24-bit two's complement, LSB first) */
__MREG__(FPGA_STANDARD_RX_NOTCH_PSTEP_L, 8, 0, 8)
__MREG__(FPGA_STANDARD_RX_NOTCH_PSTEP_M, 9, 0, 8)
__MREG__(FPGA_STANDARD_RX_NOTCH_PSTEP_H, 10, 0, 8)
```

- [ ] **Step 2: FPGA driver setters**

In `firmware/common/fpga.h`, add prototypes next to `fpga_set_rx_quarter_shift_mode`:

```c
void fpga_set_rx_notch_enable(fpga_driver_t* const drv, const bool enable);
void fpga_set_rx_notch_pstep(fpga_driver_t* const drv, const uint32_t pstep);
```

In `firmware/common/fpga.c`, after `fpga_set_rx_quarter_shift_mode`:

```c
void fpga_set_rx_notch_enable(fpga_driver_t* const drv, const bool enable)
{
	set_FPGA_STANDARD_RX_NOTCH_EN(drv, enable ? 1 : 0);
	fpga_regs_commit(drv);
}

void fpga_set_rx_notch_pstep(fpga_driver_t* const drv, const uint32_t pstep)
{
	set_FPGA_STANDARD_RX_NOTCH_PSTEP_L(drv, pstep & 0xff);
	set_FPGA_STANDARD_RX_NOTCH_PSTEP_M(drv, (pstep >> 8) & 0xff);
	set_FPGA_STANDARD_RX_NOTCH_PSTEP_H(drv, (pstep >> 16) & 0xff);
	fpga_regs_commit(drv);
}
```

- [ ] **Step 3: Radio register**

In `firmware/common/radio.h`, add to the enum after `RADIO_TX_NCO = 24`:

```c
	/**
	 * RX notch filter tone offset from the LO in Hz (signed).
	 * 0 disables the notch. Praline only.
	 */
	RADIO_RX_NOTCH = 25,
```

and change `#define RADIO_NUM_REGS (25)` to `#define RADIO_NUM_REGS (26)`.

- [ ] **Step 4: `radio_update_rx_notch()` in radio.c**

Add after `radio_update_tx_nco` (mirrors its structure; note 24-bit signed pstep at the AFE rate):

```c
static uint32_t applied_rx_notch_pstep; /* shadow: pstep depends on AFE rate */

static uint32_t radio_update_rx_notch(radio_t* const radio, uint64_t* bank)
{
#ifdef IS_PRALINE
	if (IS_PRALINE) {
		const uint64_t requested = bank[RADIO_RX_NOTCH];
		if (requested == RADIO_UNSET) {
			return 0;
		}

		const int64_t freq_hz = (int64_t) requested;

		/* AFE clock = applied MCU sample rate shifted left by the RX
		 * resampling ratio (fp_28_36 units). */
		const uint64_t mcu_rate =
			radio->config[RADIO_BANK_APPLIED][RADIO_SAMPLE_RATE];
		const uint64_t n = radio->config[RADIO_BANK_APPLIED][RADIO_RESAMPLE_RX];
		if ((mcu_rate == RADIO_UNSET) || (n == RADIO_UNSET) || (n > 5)) {
			return 0;
		}
		const uint64_t afe_clk_hz = mcu_rate >> (36 - n);

		if (freq_hz == 0) {
			if (radio->config[RADIO_BANK_APPLIED][RADIO_RX_NOTCH] != 0) {
				fpga_set_rx_notch_enable(&fpga, false);
				radio->config[RADIO_BANK_APPLIED][RADIO_RX_NOTCH] = 0;
				applied_rx_notch_pstep = 0;
				return (1 << RADIO_RX_NOTCH);
			}
			return 0;
		}

		/* pstep = round(freq_hz * 2**24 / afe_clk_hz), signed. */
		const int64_t pstep = (2 * freq_hz * 16777216 +
				       (int64_t) afe_clk_hz) /
			(2 * (int64_t) afe_clk_hz);
		if ((pstep < -(1 << 23)) || (pstep > ((1 << 23) - 1))) {
			/* Offset not reachable at the current AFE clock. */
			return 0;
		}

		const bool rate_changed =
			(radio->config[RADIO_BANK_APPLIED][RADIO_RX_NOTCH] == requested) &&
			(applied_rx_notch_pstep != (uint32_t) (pstep & 0xffffff));
		if ((radio->config[RADIO_BANK_APPLIED][RADIO_RX_NOTCH] == requested) &&
		    !rate_changed) {
			return 0;
		}

		fpga_set_rx_notch_pstep(&fpga, (uint32_t) (pstep & 0xffffff));
		fpga_set_rx_notch_enable(&fpga, true);
		radio->config[RADIO_BANK_APPLIED][RADIO_RX_NOTCH] = requested;
		applied_rx_notch_pstep = (uint32_t) (pstep & 0xffffff);
		return (1 << RADIO_RX_NOTCH);
	}
#endif

	(void) radio;
	(void) bank;
	return 0;
}
```

In `radio_update()`, after the `RADIO_TX_NCO` block, add:

```c
	if (dirty & ((1 << RADIO_RX_NOTCH) | RADIO_REG_GROUP_RATE)) {
		changed |= radio_update_rx_notch(radio, &tmp_bank[0]);
	}
```

(The `RADIO_REG_GROUP_RATE` term makes the notch track AFE-rate changes, e.g. sample-rate changes.)

In `radio_reapply_fpga_state()`, add to both the invalidate list and the `mark_dirty` list:

```c
	radio->config[RADIO_BANK_APPLIED][RADIO_RX_NOTCH] = RADIO_UNSET;
	...
	mark_dirty(radio, RADIO_RX_NOTCH);
```

- [ ] **Step 5: USB API version bump**

In `firmware/hackrf_usb/usb_descriptor.c:31`:

```c
#define USB_API_VERSION (0x0116)
```

- [ ] **Step 6: Build firmware (space-in-path workaround) and verify zero warnings**

```bash
rsync -a --delete --exclude build --exclude '.git' \
  "/Volumes/Radiator 8TB/mac-archive/hackrf/firmware/" /tmp/hackrf-fw/
export PATH=/tmp/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin:$PATH
cd /tmp/hackrf-fw/hackrf_usb && rm -rf build
cmake -DBOARD=UNIVERSAL -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j8 2>&1 | tee /tmp/fw_build_notch.log
grep -c "warning:" /tmp/fw_build_notch.log
```

Expected: build completes, `0` warnings, `hackrf_usb.bin` produced. If the xpack toolchain in /tmp is gone, re-download per Task 1 Step 1 pattern (asset: `xpack-arm-none-eabi-gcc-15.2.1-1.1-darwin-arm64.tar.gz` from xpack-dev-tools releases).

- [ ] **Step 7: Commit**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf"
git add firmware/common/fpga_regs.def firmware/common/fpga.c firmware/common/fpga.h \
        firmware/common/radio.c firmware/common/radio.h firmware/hackrf_usb/usb_descriptor.c
git commit -m "firmware: RADIO_RX_NOTCH register, pstep from AFE clock, API 0x0116"
```

---

### Task 5: Host API + hackrf_pro CLI

**Files:**
- Modify: `host/libhackrf/src/hackrf.c` (near `hackrf_set_tx_nco`, line ~3898; register defines near line 3689)
- Modify: `host/libhackrf/src/hackrf.h` (prototypes + version table, lines ~93-156)
- Modify: `host/hackrf-tools/src/hackrf_pro.c`

**Interfaces:**
- Consumes: firmware `RADIO_RX_NOTCH = 25`, USB API 0x0116 (Task 4).
- Produces: `int hackrf_set_rx_notch(hackrf_device* device, int64_t freq_hz)` / `int hackrf_get_rx_notch(hackrf_device* device, int64_t* freq_hz)`; `hackrf_pro --rx-notch HZ`.

- [ ] **Step 1: libhackrf implementation**

In `host/libhackrf/src/hackrf.c` near the other register defines (~line 3689), add:

```c
#define HACKRF_RADIO_REG_RX_NOTCH         25
```

After `hackrf_get_tx_nco`, add:

```c
int ADDCALL hackrf_set_rx_notch(hackrf_device* device, const int64_t freq_hz)
{
	if (device == NULL) {
		return HACKRF_ERROR_INVALID_PARAM;
	}
	USB_API_REQUIRED(device, 0x0116)
	return hackrf_radio_write_register(
		device,
		HACKRF_RADIO_BANK_ALL,
		HACKRF_RADIO_REG_RX_NOTCH,
		(uint64_t) freq_hz);
}

int ADDCALL hackrf_get_rx_notch(hackrf_device* device, int64_t* const freq_hz)
{
	if ((device == NULL) || (freq_hz == NULL)) {
		return HACKRF_ERROR_INVALID_PARAM;
	}
	USB_API_REQUIRED(device, 0x0116)
	uint64_t value;
	const int result = hackrf_radio_read_register(
		device,
		HACKRF_RADIO_BANK_APPLIED,
		HACKRF_RADIO_REG_RX_NOTCH,
		&value);
	if (result != HACKRF_SUCCESS) {
		return result;
	}
	/* RADIO_UNSET means the notch has never been configured: report disabled. */
	*freq_hz = (value == UINT64_MAX) ? 0 : (int64_t) value;
	return HACKRF_SUCCESS;
}
```

- [ ] **Step 2: Public header**

In `host/libhackrf/src/hackrf.h`:
- Version table comment (~line 93): change "up to version 0x0115" to "up to version 0x0116" and add a `## 0x0116` section listing `hackrf_set_rx_notch`, `hackrf_get_rx_notch` (follow the existing `## 0x0115` block's format).
- After the `hackrf_get_tx_nco` prototype, add Doxygen-documented prototypes:

```c
/**
 * Set the RX notch filter tone offset (HackRF Pro only)
 *
 * Programs the FPGA notch filter to suppress a narrowband tone at
 * freq_hz relative to the current LO (negative = below LO).
 * 0 disables the notch. The setting persists across RX/TX/OFF switches.
 *
 * Requires USB API version 0x0116 or above!
 *
 * @param device desired device
 * @param freq_hz tone offset from the LO in Hz, 0 to disable
 * @return #HACKRF_SUCCESS on success,
 *         #HACKRF_ERROR_INVALID_PARAM on NULL device,
 *         #HACKRF_ERROR_USB_API_VERSION if the firmware is older than API 0x0116
 */
extern ADDAPI int ADDCALL hackrf_set_rx_notch(
	hackrf_device* device,
	const int64_t freq_hz);

/**
 * Read back the currently applied RX notch filter offset (HackRF Pro only)
 *
 * Reads the applied configuration bank: the returned value is in effect
 * only while receiving. 0 means the notch is disabled or never configured.
 *
 * Requires USB API version 0x0116 or above!
 *
 * @param device desired device
 * @param[out] freq_hz applied tone offset in Hz
 * @return #HACKRF_SUCCESS on success
 */
extern ADDAPI int ADDCALL hackrf_get_rx_notch(
	hackrf_device* device,
	int64_t* const freq_hz);
```

- [ ] **Step 3: hackrf_pro CLI**

In `host/hackrf-tools/src/hackrf_pro.c`, following the `--nco-freq` pattern:
- usage line: `printf("\t[--rx-notch HZ] # RX notch filter tone offset in Hz from LO (0 disables).\n");`
- add `int64_t rx_notch = 0; bool do_rx_notch = false;` to the option state
- long option `{"rx-notch", required_argument, 0, 5}` (next free val) and parse with `strtoll`
- action block after the NCO block:

```c
	if (do_rx_notch) {
		result = hackrf_set_rx_notch(device, rx_notch);
		if (result != HACKRF_SUCCESS) {
			fprintf(stderr,
				"RX notch failed: %s (%d)\n",
				hackrf_error_name(result),
				result);
		} else {
			int64_t applied = 0;
			hackrf_get_rx_notch(device, &applied);
			printf("RX notch set to %lld Hz (applied: %lld Hz)\n",
			       (long long) rx_notch,
			       (long long) applied);
		}
	}
```

- [ ] **Step 4: Build host and verify zero warnings**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf/host"
cmake --build build -j8 2>&1 | tee /tmp/host_build_notch.log
grep -c "warning:" /tmp/host_build_notch.log
```

Expected: builds clean, `0` warnings from compilation (two pre-existing macOS `ld:` linker notes about alignment/duplicate `-lm` are acceptable — they exist on upstream too).

- [ ] **Step 5: Commit**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf"
git add host/libhackrf/src/hackrf.c host/libhackrf/src/hackrf.h host/hackrf-tools/src/hackrf_pro.c
git commit -m "host: hackrf_set/get_rx_notch API and hackrf_pro --rx-notch (API 0x0116)"
```

---

### Task 6: Mock-libusb unit tests

**Files:**
- Modify: `host/tests/test_pro_fpga.c`

**Interfaces:**
- Consumes: `hackrf_set_rx_notch` / `hackrf_get_rx_notch` (Task 5); existing mock pattern (`create_device`, `queue_write_ok`, `mock_libusb_queue_transfer`, vendor requests 59/60).

- [ ] **Step 1: Write the failing tests**

In `host/tests/test_pro_fpga.c`, add after the defines:

```c
#define REG_RX_NOTCH 25
```

and add three tests following the existing style:

```c
/* hackrf_set_rx_notch writes the Hz value to register 25 in bank ALL */
static void test_rx_notch_write(void)
{
	hackrf_device* dev;

	mock_libusb_reset();
	dev = create_device(0x0116);
	queue_write_ok(REG_RX_NOTCH, BANK_ALL);
	assert(hackrf_set_rx_notch(dev, -327500) == HACKRF_SUCCESS);
	printf("PASS: hackrf_set_rx_notch write path\n");

	free(dev);
}

/* API gate: firmware older than 0x0116 is rejected before any transfer */
static void test_rx_notch_api_gate(void)
{
	hackrf_device* dev;

	mock_libusb_reset();
	dev = create_device(0x0115);
	assert(hackrf_set_rx_notch(dev, -327500) == HACKRF_ERROR_USB_API_VERSION);
	printf("PASS: hackrf_set_rx_notch API gate\n");

	free(dev);
}

/* get decodes RADIO_UNSET as disabled (0) */
static void test_rx_notch_get_unset(void)
{
	hackrf_device* dev;
	mock_transfer_t t;

	mock_libusb_reset();
	dev = create_device(0x0116);
	memset(&t, 0, sizeof(t));
	t.request = VENDOR_REQUEST_RADIO_READ_REG;
	t.index = BANK_APPLIED;
	t.value = REG_RX_NOTCH;
	t.return_code = 8;
	memset(t.response, 0xff, sizeof(t.response)); /* RADIO_UNSET */
	mock_libusb_queue_transfer(&t);
	int64_t hz = -1;
	assert(hackrf_get_rx_notch(dev, &hz) == HACKRF_SUCCESS);
	assert(hz == 0);
	printf("PASS: hackrf_get_rx_notch RADIO_UNSET decode\n");

	free(dev);
}
```

Check `mock_libusb.h` for the exact response-field name (`t.response` above) and the read-transfer queue pattern used by the existing `hackrf_get_*` tests — mirror them exactly if they differ.

Register the tests in the file's `main()` alongside the others.

- [ ] **Step 2: Run to verify they fail**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf/host/build"
cmake --build . --target test_pro_fpga && ./hackrf-tools/src/../../tests/test_pro_fpga || ctest -R pro_fpga
```

Expected: link error or test failure (functions not yet in the built libhackrf if Task 5 build artifacts are stale — rebuild host first if needed).

- [ ] **Step 3: Run to verify they pass**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf/host"
cmake --build build -j8
cd build && ctest
```

Expected: all tests pass (5 existing + the suite now includes the new ones).

- [ ] **Step 4: Commit**

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf"
git add host/tests/test_pro_fpga.c
git commit -m "host/tests: mock-libusb coverage for hackrf_set/get_rx_notch"
```

---

### Task 7: Flash + hardware-in-the-loop verification

**Files:**
- None modified (verification only)

**Interfaces:**
- Consumes: everything above. Newly built `hackrf_usb.bin` (Task 4 Step 6), new host tools (Task 5 build).
- Produces: hardware proof — FPGA register read-back, tone suppression measurement, GPS acquisition comparison.

- [ ] **Step 1: Flash the Pro**

```bash
hackrf_spiflash -d 0000000000000000977c64de2b557213 -w /tmp/hackrf-fw/hackrf_usb/build/hackrf_usb.bin
hackrf_spiflash -d 0000000000000000977c64de2b557213 -R
sleep 3
hackrf_info | grep -A3 977c64de
```

Expected: `Firmware Version: 2026.01.3+ (API:1.16)`.

- [ ] **Step 2: Register read-back test (no RF)**

```bash
export HACKRF_LIB="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/libhackrf/src/libhackrf.dylib"
python3 - <<'EOF'
import ctypes, os, time
lib = ctypes.CDLL(os.environ["HACKRF_LIB"])
lib.hackrf_init()
dev = ctypes.c_void_p()
lib.hackrf_open_by_serial(b"0000000000000000977c64de2b557213", ctypes.byref(dev))
hz = ctypes.c_int64(-327500)
assert lib.hackrf_set_rx_notch(dev, hz) == 0
lib.hackrf_set_freq(dev, ctypes.c_uint64(1575420000))  # triggers radio_update
time.sleep(0.2)
out = ctypes.c_int64(0)
assert lib.hackrf_get_rx_notch(dev, ctypes.byref(out)) == 0
print("applied notch:", out.value, "Hz (expect -327500)")
def fpga_rd(reg):
    v = ctypes.c_uint16(0); lib.hackrf_fpga_read_register(dev, ctypes.c_uint16(reg), ctypes.byref(v)); return v.value
pstep = fpga_rd(8) | (fpga_rd(9) << 8) | (fpga_rd(10) << 16)
if pstep & 0x800000: pstep -= 1 << 24
print("FPGA enable:", fpga_rd(7), "pstep:", pstep)
# default 10 Msps MCU rate, n=2 -> AFE 40 MHz:
expect = round(-327500 * 2**24 / 40e6)
print("expected pstep:", expect, "PASS" if pstep == expect else "FAIL")
assert out.value == -327500
lib.hackrf_set_rx_notch(dev, ctypes.c_int64(0))
lib.hackrf_close(dev); lib.hackrf_exit()
EOF
```

Expected: `applied notch: -327500 Hz`, `FPGA enable: 1`, pstep matches the computed value, final line PASS. (If the applied sample rate differs, recompute `expect` from the actual AFE rate.)

- [ ] **Step 3: Live tone suppression measurement**

```bash
PRO=0000000000000000977c64de2b557213
B="/Volumes/Radiator 8TB/mac-archive/hackrf/host/build/hackrf-tools/src"
# notch OFF baseline
"$B/hackrf_pro" -d $PRO --rx-notch 0
"$B/hackrf_transfer" -d $PRO -r /tmp/notch_off.iq -f 1575420000 -s 8000000 -n 8000000 -l 40 -g 46 -a 0 -p 1
# notch ON
"$B/hackrf_pro" -d $PRO --rx-notch -327500
"$B/hackrf_transfer" -d $PRO -r /tmp/notch_on.iq -f 1575420000 -s 8000000 -n 8000000 -l 40 -g 46 -a 0 -p 1
python3 - <<'EOF'
import numpy as np
for name in ["off", "on"]:
    d = np.fromfile(f"/tmp/notch_{name}.iq", dtype=np.int8).astype(np.float32)
    x = d[0::2] + 1j*d[1::2]; x -= x.mean()
    N = 1 << 18
    w = np.abs(np.fft.fftshift(np.fft.fft(x[:N] * np.hanning(N)))) ** 2
    f = np.fft.fftshift(np.fft.fftfreq(N, 1 / 8e6)) / 1e3
    db = 10 * np.log10(w + 1e-9)
    i = np.argmin(np.abs(f + 327.5))
    print(f"notch {name}: tone@-327.5kHz = {db[i]:.1f} dB, median {np.median(db):.1f} dB")
EOF
```

Expected: tone drops from ~30+ dB above the noise floor to within a few dB of it (≥25 dB suppression).

Note: the notch tracks the *programmed* offset, not the tone itself. If the tone drifts, reprogram with the current offset.

- [ ] **Step 4: GPS acquisition comparison**

```bash
cd "/Volumes/Radiator 8TB/gnss/hackrf_gnss"
"$B/hackrf_transfer" -d $PRO -r /tmp/l1_notched_hw.iq -f 1575420000 -s 8000000 -n 80000000 -l 40 -g 46 -a 0 -p 1
python3 -c "
import numpy as np
d = np.fromfile('/tmp/l1_notched_hw.iq', dtype=np.int8, count=2*8_000_000).astype(np.float32)
x = d[0::2] + 1j*d[1::2]; x -= x.mean()
out = np.empty(2*len(x), dtype=np.float32); out[0::2] = x.real; out[1::2] = x.imag
out.tofile('/tmp/l1_hw_bb.f32')
"
cargo run --release --example acquire_file -- /tmp/l1_hw_bb.f32 8000000 -10000 10000 250 800 2>/dev/null | python3 -c "
import json, sys
for e in json.load(sys.stdin)[:6]:
    print(f\"PRN {e['prn']:>3}  metric {e['metric']:.2f}  doppler {e['doppler']:+7.0f}\")
"
```

Expected: PRN 9 (and ideally PRN 4 / SBAS 131) above or near the 2.5 threshold **without any host-side notch** — comparable to the 2026-08-20 software-notch baseline (PRN 9 metric 3.4–3.6). If metrics are much worse, the notch is likely programmed to the wrong offset — re-measure the tone frequency and reprogram.

- [ ] **Step 5: Leave the device in a sane state + final commit**

```bash
"$B/hackrf_pro" -d $PRO --rx-notch 0   # disable notch
```

```bash
cd "/Volumes/Radiator 8TB/mac-archive/hackrf"
git status --short   # confirm only intended files were touched
git push galic1987 upstream-pr-submit
```

---

## Self-Review Notes

- Spec coverage: gateware module (Task 2), integration (Task 3), firmware register + API bump (Task 4), host API + CLI (Task 5), unit tests (Task 6), HIL incl. live-tone and GPS comparison (Task 7), toolchain + build flow (Tasks 1, 4, 7). All spec sections covered.
- Type consistency: `hackrf_set_rx_notch(hackrf_device*, int64_t)` used identically in Tasks 5, 6, 7. `RADIO_RX_NOTCH = 25` / `HACKRF_RADIO_REG_RX_NOTCH 25` consistent. SPI regs 0x07-0x0A consistent between Tasks 3, 4, 7.
- Deferred-at-build items with explicit stop conditions: `ESTIMATE_DELAY` tuning (Task 2 Step 4), resource overflow (Task 3 Step 2).
