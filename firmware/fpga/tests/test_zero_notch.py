#
# This file is part of HackRF.
#
# Copyright (c) 2026 Great Scott Gadgets
# SPDX-License-Identifier: BSD-3-Clause

import os
import sys

import numpy as np
from amaranth.sim import Simulator

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dsp.zero_notch import ZeroNotch


def _signed(v, w=8):
    return v - (1 << w) if v >= (1 << (w - 1)) else v


def run_zero_notch(tone_freq, n_samples=8192, tone_amp=60.0, seed=7):
    """Drive the one-zero notch with tone + noise; return tone suppression dB."""
    rng = np.random.default_rng(seed)
    ph = 2 * np.pi * tone_freq * np.arange(n_samples)
    xi = np.clip(np.round(tone_amp * np.cos(ph) + rng.normal(0, 3.0, n_samples)),
                 -128, 127).astype(int)
    xq = np.clip(np.round(tone_amp * np.sin(ph) + rng.normal(0, 3.0, n_samples)),
                 -128, 127).astype(int)

    cr = int(round(127 * np.cos(2 * np.pi * tone_freq)))
    ci = int(round(127 * np.sin(2 * np.pi * tone_freq)))

    dut = ZeroNotch(width=8)
    out_i, out_q = [], []

    async def drive(ctx):
        ctx.set(dut.coef_r, cr)
        ctx.set(dut.coef_i, ci)
        ctx.set(dut.enable, 1)
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
                out_i.append(_signed(ctx.get(dut.output.p.i)))
                out_q.append(_signed(ctx.get(dut.output.p.q)))

    sim = Simulator(dut)
    sim.add_clock(1e-8)
    sim.add_testbench(drive)
    sim.add_testbench(collect)
    sim.run()

    skip = 64  # no integrator to settle; skip only pipeline fill
    n = n_samples - skip
    e = np.exp(-2j * np.pi * tone_freq * np.arange(n))
    tone_in = np.abs(np.mean((xi[skip:] + 1j * xq[skip:]) * e))
    yo = np.array(out_i[skip:], dtype=float) + 1j * np.array(out_q[skip:], dtype=float)
    tone_out = np.abs(np.mean(yo * e))
    return 20 * np.log10(tone_in / max(tone_out, 1e-9))


def test_zero_notch_suppresses_tone():
    """Tone exactly at the null must be suppressed by >40 dB."""
    sup = run_zero_notch(0.02)
    print(f"zero-notch suppression: {sup:.1f} dB")
    assert sup > 38.0


def test_zero_notch_suppresses_negative_frequency():
    """Negative offsets (tone below LO) must work the same."""
    sup = run_zero_notch(-0.03)
    print(f"zero-notch suppression (negative offset): {sup:.1f} dB")
    assert sup > 38.0


def test_zero_notch_disabled_is_transparent():
    """With enable=0 the input passes through (checked via correlation)."""
    rng = np.random.default_rng(11)
    n = 2048
    xi = rng.integers(-128, 128, n)
    xq = rng.integers(-128, 128, n)

    dut = ZeroNotch(width=8)
    out_i = []

    async def drive(ctx):
        ctx.set(dut.enable, 0)
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
                out_i.append(_signed(ctx.get(dut.output.p.i)))

    sim = Simulator(dut)
    sim.add_clock(1e-8)
    sim.add_testbench(drive)
    sim.add_testbench(collect)
    sim.run()

    out_arr = np.array(out_i, dtype=float)
    best_corr = max(
        np.corrcoef(out_arr[lag:], xi[: len(out_arr) - lag].astype(float))[0, 1]
        for lag in range(0, 8)
        if len(out_arr) - lag > 100
    )
    print(f"passthrough correlation: {best_corr:.6f}")
    assert best_corr > 0.9999
