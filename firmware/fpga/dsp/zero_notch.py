#
# This file is part of HackRF.
#
# Copyright (c) 2026 Great Scott Gadgets
# SPDX-License-Identifier: BSD-3-Clause

from amaranth               import Module, Signal, signed, Mux
from amaranth.lib           import wiring, stream
from amaranth.lib.wiring    import In, Out

from dsp.fir_mac16          import iCE40Multiplier
from util                   import IQSample


class ZeroNotch(wiring.Component):
    """
    Programmable single-zero notch filter.

        y[n] = x[n] - c * x[n-1],   c = (coef_r + j*coef_i) / 127

    Places a null at f = atan2(coef_i, coef_r) / (2*pi) * fs.  The null
    depth is limited by coefficient quantization (~-48 dB); the response
    rises to +3 dB half a band away from the null (single-zero tilt).

    Fits where the full heterodyne notch (dsp/notch.py) does not: only
    four multipliers, no NCO, no integrator, no long delay line.

    Parameters
    ----------
    width : int
        Bit width of the I/Q samples.

    Signals
    -------
    coef_r, coef_i : Signal(signed(8)), in
        cos/sin of the notch angle, scaled to +-127.
    enable : Signal(1), in
        Notch enable. When disabled the input passes through unchanged.
    """

    def __init__(self, width=8, domain="sync", xdelay=4):
        self.width = width
        self.xdelay = xdelay
        self._domain = domain
        sig = stream.Signature(IQSample(width), always_ready=True)
        super().__init__({
            "input":  In(sig),
            "output": Out(sig),
            "enable": In(1),
            "coef_r": In(signed(8)),
            "coef_i": In(signed(8)),
        })

    def elaborate(self, platform):
        m = Module()
        w = self.width
        dom = self._domain

        # Sample delay.
        x_i = Signal(signed(w))
        x_q = Signal(signed(w))
        m.d[dom] += [x_i.eq(self.input.p.i), x_q.eq(self.input.p.q)]

        def mac(signame, a_sig, b_sig):
            """One DSP-block multiplier, always-ready, 3-cycle latency."""
            mm = iCE40Multiplier(a_width=16, b_width=16, p_width=0,
                                 o_width=32, always_ready=True)
            m.submodules[signame] = mm
            m.d.comb += [
                mm.a.eq(a_sig),
                mm.b.eq(b_sig),
                mm.valid_in.eq(1),
                mm.ready_out.eq(1),
            ]
            return mm.o  # signed 32, valid 3 cycles after inputs

        # c * x[n-1]:  (cr*xi - ci*xq) + j*(cr*xq + ci*xi)
        p_ri = mac("zn_ri", x_i, self.coef_r)
        p_iq = mac("zn_iq", x_q, self.coef_i)
        p_rq = mac("zn_rq", x_q, self.coef_r)
        p_ii = mac("zn_ii", x_i, self.coef_i)

        # Clean path delayed to match the MAC latency (4 cycles,
        # calibrated by simulation sweep).
        d_i, d_q = x_i, x_q
        for _ in range(self.xdelay - 1):
            n_i = Signal(signed(w))
            n_q = Signal(signed(w))
            m.d[dom] += [n_i.eq(d_i), n_q.eq(d_q)]
            d_i, d_q = n_i, n_q

        rnd = 1 << 6  # +0.5 LSB before /127 truncation
        t_i = Signal(signed(w + 10))
        t_q = Signal(signed(w + 10))
        m.d[dom] += [
            t_i.eq((p_ri - p_iq + rnd) >> 7),
            t_q.eq((p_rq + p_ii + rnd) >> 7),
        ]

        y_i = Signal(signed(w + 2))
        y_q = Signal(signed(w + 2))
        m.d.comb += [
            y_i.eq(d_i - Mux(self.enable, t_i, 0)),
            y_q.eq(d_q - Mux(self.enable, t_q, 0)),
        ]
        lo = -(1 << (w - 1))
        hi = (1 << (w - 1)) - 1
        m.d[dom] += [
            self.output.p.i.eq(Mux(y_i > hi, hi, Mux(y_i < lo, lo, y_i))),
            self.output.p.q.eq(Mux(y_q > hi, hi, Mux(y_q < lo, lo, y_q))),
            self.output.valid.eq(self.input.valid),
        ]

        return m
