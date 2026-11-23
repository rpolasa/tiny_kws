"""Fast bus-level check of the Pico PSRAM emulator against SCK glitches (no Verilog).

A Python QSPI master generates the exact kws_qspi.v waveforms (SCK half period = div clk of
16.3125 MHz, CS gap, cmd/addr/mode/dummy, read data captured rd_dly+1 clk after each falling edge,
single-byte writes) and drives pio_model.PicoPsram (the real pioasm opcodes) at 150 MHz.

Crosstalk model (GLITCH): when the Pico's SD outputs switch with >= GL_BITS lines rising, the SCK
level the Pico sees is forced high for GL_W sys cycles, GL_D cycles after the switch (only while SCK
is low) - the coupling of the 4 data wires into SCK on the ribbon / shared ground.

    python3 glitch_sim.py [header] [holdoff_out holdoff_rise]
Prints per div/rd: bytes captured wrong by the chip, and the raw sniffer's extra SCK edges.
"""
import os, sys, random
from pio_model import PicoPsram

HERE = os.path.dirname(os.path.abspath(__file__))
TC = 1e3 / 16.3125          # chip clk (ns)
TP = 1e3 / 150.0            # Pico sys clk (ns)
PAD = 4.0                   # FPGA pad + wire delay each way (ns)


def master_waveform(txs, div, rd):
    """txs: list of ('r', addr, nbytes_or_None_for_full_burst, cut_after_nibbles|None) / ('w', addr, byte).
    returns (events, captures): events = [(t_ns, sck, cs, sd, sd_oe)], captures = [(t_ns, tx_index, nibble_index)]"""
    ev, cap = [], []
    t = 10  # chip clock index
    ev.append((0.0, 0, 1, 0, 0))
    for ti, tx in enumerate(txs):
        t += 2 * div + 2                                   # S_GAP: CS high {hdiv,1}+1 clocks
        if tx[0] == 'r':
            _, addr, bcnt, cut = tx
            npulse = 19 + 2 * bcnt
        else:
            _, addr, data = tx
            npulse = 16; cut = None
        cmd = 0xEB if tx[0] == 'r' else 0x38
        t0 = t

        def nib(p):     # value driven for pulse p+1 (set at falling edge p, p=0 at CS fall)
            if p < 8: return (0b1100 | ((cmd >> (7 - p)) & 1)), 0b1101
            if p < 14: return (addr >> (4 * (13 - p))) & 15, 0b1111
            if p < 16:
                if tx[0] == 'w': return ((data >> 4) if p == 14 else data) & 15, 0b1111
                return 0, 0b1111
            return 0, 0
        v, oe = nib(0)
        ev.append((t0 * TC, 0, 0, v, oe))
        last_fall = None
        for p in range(1, npulse + 1):
            ev.append(((t0 + (2 * p - 1) * div) * TC, 1, 0, v, oe))          # rising edge p
            tf = t0 + 2 * p * div
            v, oe = nib(p)
            ev.append((tf * TC, 0, 0, v, oe))                                 # falling edge p
            if tx[0] == 'r' and p >= 20:
                k = p - 20
                cap.append(((tf + rd + 1) * TC, ti, k))
            last_fall = tf
            if cut is not None and p >= 20 + cut:                               # stream stopped mid-burst
                break
        t = last_fall + (rd + 2 if tx[0] == 'r' else 1) + 1
        ev.append((t * TC, 0, 1, 0, 0))                                         # CS high, bus released
    return ev, cap


def run(header, txs, mem, div, rd, glitch, holdoff=None, gl_bits=2, gl_w=3, gl_d=3, seed=0):
    pico = PicoPsram(header, mem, burst=3, holdoff=holdoff)
    ev, cap = master_waveform(txs, div, rd)
    t_end = ev[-1][0] + 2000
    ei, ci = 0, 0
    sck = cs = 1; sck = 0; sd_m = 0; oe_m = 0
    # Pico output history (for the FPGA capture with pad delay)
    pv, po = 0, 0
    hist = []               # (t_change_ns, val, oe)
    forced_until = -1; pending = []
    sniff = []              # raw samples at Pico-seen SCK rises (like qspi_sniff, no deglitch)
    prev_seen_sck = 0
    captured = {}
    n = 0
    while True:
        t = n * TP
        if t > t_end: break
        while ei < len(ev) and ev[ei][0] + PAD <= t:
            _, sck, cs, sd_m, oe_m = ev[ei]; ei += 1
        bus_pico = pv if po else 0
        sd = (sd_m & oe_m) | (bus_pico & po & ~oe_m & 15)
        seen_sck = sck
        if glitch:
            while pending and pending[0][0] <= n: forced_until = max(forced_until, pending.pop(0)[1])
            if n < forced_until and cs == 0: seen_sck = 1
        if seen_sck and not prev_seen_sck and cs == 0: sniff.append(sd)
        prev_seen_sck = seen_sck
        val, oe = pico.step((sd << 10) | (seen_sck << 14) | (cs << 15))
        if (val, oe) != (pv, po):
            rising = (val & ~pv & oe) & 15
            if glitch and oe and bin(rising).count("1") >= gl_bits:
                pending.append((n + gl_d, n + gl_d + gl_w))
            hist.append((t + PAD, val, oe)); pv, po = val, oe
        # FPGA captures (bus as seen at the FPGA = Pico outputs delayed by PAD)
        while ci < len(cap) and cap[ci][0] <= t:
            tc, ti, k = cap[ci]; ci += 1
            v = 0
            for th, hv, ho in hist:
                if th <= tc: v = hv if ho else 0
                else: break
            captured[(ti, k)] = v
        if len(hist) > 64: hist = hist[-32:]
        n += 1
    bad = 0; first = None
    for (ti, k), v in captured.items():
        tx = txs[ti]; a = tx[1] + k // 2
        e = (mem[a] >> 4) if k % 2 == 0 else (mem[a] & 15)
        if v != e:
            bad += 1
            if first is None: first = (ti, k)
    exp_samples = 0
    for tx in txs:
        if tx[0] == 'w': exp_samples += 16
        else: exp_samples += 20 + (2 * tx[2] - 1 if tx[3] is None else tx[3])
    return bad, len(captured), len(sniff) - exp_samples, first, pico.stats


def make_txs(mem_len, rng, n=60):
    txs = []; a = 0x1248
    for i in range(n):
        blen = 32; bcnt = blen - (a & (blen - 1))
        cut = rng.randrange(1, 2 * bcnt) if rng.random() < 0.15 else None
        txs.append(('r', a, bcnt, cut))
        a += bcnt if cut is None else 0
        if cut is not None: a = 0x1248 + rng.randrange(0, mem_len - 0x3000)
        if rng.random() < 0.2: txs.append(('w', 0x7FFF00 + rng.randrange(48), rng.randrange(256)))
    return txs


if __name__ == "__main__":
    hdr = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "qspi_psram.pio.h")
    hos = [int(v) for v in sys.argv[2].split(",")] if len(sys.argv) > 2 else [None]
    gl = [tuple(int(x) for x in v.split(":")) for v in os.environ.get("GL", "2:3:3").split(",")]   # bits:w:d
    rng = random.Random(1)
    mem = bytearray(rng.randrange(256) for _ in range(0x10000))
    txs = make_txs(len(mem), rng)
    cases = [tuple(int(x) for x in v.split(":")) for v in os.environ.get("CASES", "7:3,4:2,2:1,2:0,1:0,1:1").split(",")]
    for ho in hos:
        for g in [None] + gl:
            for div, rd in cases:
                kw = dict(gl_bits=g[0], gl_w=g[1], gl_d=g[2]) if g else {}
                bad, ncap, extra, first, st = run(hdr, txs, mem, div, rd, g is not None, ho, **kw)
                print(f"H {ho} glitch {g} div {div} rd {rd}: {bad:4d}/{ncap} nibbles wrong, sniffer extra edges {extra:+d}, "
                      f"first bad {first}, late {st['late']} stale {st['stale']}", flush=True)
