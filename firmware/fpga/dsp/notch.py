#
# This file is part of HackRF.
#
# Copyright (c) 2026 Great Scott Gadgets
# SPDX-License-Identifier: BSD-3-Clause

from amaranth               import Module, Signal, signed, Mux
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

    Signals
    -------
    pstep : Signal(signed(phase_width)), in
        Phase increment per sample; negative values notch below the LO.
    enable : Signal(1), in
        Notch enable. When disabled the estimate is forced to zero.
    """


    def __init__(self, width=8, ratio=14, phase_width=24, domain="sync",
                 xdelay=5, cdelay=2):
        self.width = width
        self.ratio = ratio
        self.phase_width = phase_width
        self.xdelay = xdelay   # clean-path delay (registers)
        self.cdelay = cdelay   # NCO output delay into the mix-up stage
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
        nco_w = 9  # NCO output width; cos/sin scaled to +-255
        dom = self._domain

        m.submodules.nco = nco = NCO(phase_width=self.phase_width,
                                     output_width=nco_w,
                                     domain=dom)

        # Phase accumulator (free-running; two's-complement pstep handles
        # negative frequencies).
        phase = Signal(self.phase_width)
        m.d[dom] += phase.eq(phase + self.pstep)
        m.d.comb += [
            nco.phase.eq(phase),
            nco.en.eq(self.enable & self.input.valid),
        ]

        # Delay the clean input to align with the estimate path.
        x_i = Signal(signed(w))
        x_q = Signal(signed(w))
        m.d.comb += [
            x_i.eq(self.input.p.i),
            x_q.eq(self.input.p.q),
        ]
        for _ in range(self.xdelay):
            d_i = Signal(signed(w))
            d_q = Signal(signed(w))
            m.d[dom] += [d_i.eq(x_i), d_q.eq(x_q)]
            x_i, x_q = d_i, d_q

        # NCO output (2-cycle latency after phase).
        c0 = Signal(signed(nco_w))
        s0 = Signal(signed(nco_w))
        m.d.comb += [c0.eq(nco.output.i), s0.eq(nco.output.q)]

        # Input delayed to the NCO output time (2 cycles).
        xi_d = Signal(signed(w))
        xq_d = Signal(signed(w))
        xi_d2 = Signal(signed(w))
        xq_d2 = Signal(signed(w))
        m.d[dom] += [
            xi_d.eq(self.input.p.i),
            xq_d.eq(self.input.p.q),
            xi_d2.eq(xi_d),
            xq_d2.eq(xq_d),
        ]

        # Mix down: xd = x * conj(cos, sin), dropping the NCO scale bits.
        mw = w + 2
        xd_i = Signal(signed(mw))
        xd_q = Signal(signed(mw))
        rnd1 = 1 << (nco_w - 2)  # +0.5 LSB rounding before truncation
        m.d[dom] += [
            xd_i.eq((xi_d2 * c0 + xq_d2 * s0 + rnd1) >> (nco_w - 1)),
            xd_q.eq((xq_d2 * c0 - xi_d2 * s0 + rnd1) >> (nco_w - 1)),
        ]

        # Leaky integrator (tone estimate at DC), extended precision.
        ew = mw + self.ratio
        avg_i = Signal(signed(ew))
        avg_q = Signal(signed(ew))
        m.d[dom] += [
            avg_i.eq(avg_i + (((xd_i << self.ratio) - avg_i) >> self.ratio)),
            avg_q.eq(avg_q + (((xd_q << self.ratio) - avg_q) >> self.ratio)),
        ]

        # NCO output delayed by cdelay cycles so the mix-up phase matches
        # the sample the estimate is subtracted from.
        c_d = c0
        s_d = s0
        for _ in range(self.cdelay):
            c_n = Signal(signed(nco_w))
            s_n = Signal(signed(nco_w))
            m.d[dom] += [c_n.eq(c_d), s_n.eq(s_d)]
            c_d, s_d = c_n, s_n

        # Mix up: est = avg * (cos, sin), dropping integrator + NCO scale.
        avg_i_r = Signal(signed(mw))
        avg_q_r = Signal(signed(mw))
        rnd2 = 1 << (self.ratio - 1)  # +0.5 LSB rounding on integrator readout
        m.d.comb += [
            avg_i_r.eq((avg_i + rnd2) >> self.ratio),
            avg_q_r.eq((avg_q + rnd2) >> self.ratio),
        ]
        est_i = Signal(signed(mw + 2))
        est_q = Signal(signed(mw + 2))
        m.d[dom] += [
            est_i.eq((avg_i_r * c_d - avg_q_r * s_d + rnd1) >> (nco_w - 1)),
            est_q.eq((avg_i_r * s_d + avg_q_r * c_d + rnd1) >> (nco_w - 1)),
        ]

        # Subtract the estimate (zero when disabled), saturate to width.
        y_i = Signal(signed(mw + 3))
        y_q = Signal(signed(mw + 3))
        m.d.comb += [
            y_i.eq(x_i - Mux(self.enable, est_i, 0)),
            y_q.eq(x_q - Mux(self.enable, est_q, 0)),
        ]
        lo = -(1 << (w - 1))
        hi = (1 << (w - 1)) - 1
        m.d[dom] += [
            self.output.p.i.eq(Mux(y_i > hi, hi, Mux(y_i < lo, lo, y_i))),
            self.output.p.q.eq(Mux(y_q > hi, hi, Mux(y_q < lo, lo, y_q))),
            self.output.valid.eq(self.input.valid),
        ]

        return m
