"""Host-side driver for tiny_kws (works under MicroPython on the TT demo board's RP2350, or CPython).

Bind the six pin functions to your board, then:
    k = KWS(pins); k.load_model(open("model.bin","rb").read()); k.start(); while True: print(k.poll())

Pin functions needed (pins = object with these methods):
    ui(bit, val)     drive ui_in[bit]           uo(bit) -> int   read uo_out[bit]
    uio_out(bit,val) drive uio[bit] (host side, only while CTRL.RUN = 0)
    uio_in(bit)      read uio[bit]              uio_dir(bit, output: bool)
On the TT demo board these map to the RP2350 GPIOs exposed by the ttboard SDK
(tt.ui_in / tt.uo_out / tt.uio_in); the design must be selected and clocked at 50 MHz.
"""

CTRL, PDMDIV, MEMCFG, BURST, BASE0, THR, DETCFG = 0x00, 0x01, 0x02, 0x03, 0x04, 0x07, 0x08
PCM, FEAT, STATUS, RESULT, ID = 0x10, 0x11, 0x20, 0x21, 0x26
RUN, SRC_PCM, FEAT_HOST, HOP512, CLEAR = 1, 2, 4, 8, 16
CLASSES = ["_silence_", "_unknown_", "yes", "no", "up", "down", "left", "right", "on", "off", "stop", "go"]
# QSPI Pmod pins on uio
CS0, SD0, SD1, SCK, SD2, SD3, CS1, CS2 = range(8)


class KWS:
    def __init__(self, p):
        self.p = p
        self.p.ui(3, 1); self.p.ui(1, 0)          # SPI CSn high, SCK low

    # ---------------- register interface (SPI mode 0 on ui[1]=SCK ui[2]=MOSI ui[3]=CSn, uo[1]=MISO)
    def _xfer(self, data, nread=0):
        p, out = self.p, []
        p.ui(3, 0)
        for byte in list(data) + [0] * nread:
            v = 0
            for b in range(7, -1, -1):
                p.ui(2, (byte >> b) & 1); p.ui(1, 1)
                v = (v << 1) | p.uo(1)
                p.ui(1, 0)
            out.append(v)
        p.ui(3, 1)
        return out[len(data):]

    def wr(self, addr, *vals):
        self._xfer([addr & 0x7F] + list(vals))

    def rd(self, addr, n=1):
        r = self._xfer([0x80 | addr], n)
        return r if n > 1 else r[0]

    # ---------------- model upload: SPI-mode write (0x02) to PSRAM A while the chip has released uio
    def load_model(self, blob, cs=CS1, base=0):
        assert self.rd(CTRL) & RUN == 0, "stop the engine first (CTRL.RUN=0 releases the QSPI bus)"
        p = self.p
        for b in (CS0, SD0, SCK, SD2, SD3, CS1, CS2):
            p.uio_dir(b, True)
        for c in (CS0, CS1, CS2):
            p.uio_out(c, 1)
        p.uio_out(SD2, 1); p.uio_out(SD3, 1); p.uio_out(SCK, 0)

        def spi(bytes_):
            for byte in bytes_:
                for b in range(7, -1, -1):
                    p.uio_out(SD0, (byte >> b) & 1); p.uio_out(SCK, 1); p.uio_out(SCK, 0)
        page = 1024
        for off in range(0, len(blob), 32):         # stay inside a PSRAM page, CS short
            a = base + off
            chunk = blob[off:off + min(32, page - (a % page))]
            p.uio_out(cs, 0); spi([0x02, a >> 16 & 255, a >> 8 & 255, a & 255]); spi(chunk); p.uio_out(cs, 1)
        for b in range(8):
            p.uio_dir(b, False)                      # hand the bus back
        self.wr(BASE0, base & 255, base >> 8 & 255, base >> 16 & 255)
        self.wr(MEMCFG, (2 << 5) | (2 << 2) | (1 if cs == CS1 else 2 if cs == CS2 else 0))

    def start(self, thr=16, n=3, nfill=2, pcm=False):
        assert self.rd(ID) == 0x4B, "tiny_kws not responding"
        self.wr(THR, thr, (nfill << 4) | n)
        self.wr(CTRL, RUN | HOP512 | (SRC_PCM if pcm else 0) | CLEAR)

    def configure(self, fclk, **kw):
        """Write PDM_DIV, MEMCFG, BURST, BASE, THR, DETCFG and finally CTRL (RUN | HOP512 | CLEAR | source)
        for clock fclk (Hz). Use instead of start() when the chip does not run at 50 MHz; BURST must follow the clock (tCEM)."""
        regs, info, err, warn = chip_setup(fclk, **kw)
        if err: raise ValueError("; ".join(err))
        assert self.rd(ID) == 0x4B, "tiny_kws not responding"
        for a, v in regs: self.wr(a, v)
        return info, warn

    def push_pcm(self, sample):
        v = sample & 0xFFFF
        self._xfer([PCM, v & 255, v >> 8])

    def poll(self):
        st = self.rd(STATUS)
        r, t1, t2 = self.rd(RESULT, 3)
        s8 = lambda v: v - 256 if v > 127 else v
        det = None
        if st & 0x10:
            det = CLASSES[r & 15]
            self.wr(STATUS, 0x10)
        return dict(top1=CLASSES[r >> 4], logit=s8(t1), margin=s8(t1) - s8(t2), detected=det,
                    overrun=bool(st & 4), model_error=bool(st & 8))


# ---------------- chip set-up for a given clock (mirror of app/kws_model.js::chipSetup; keep the two identical)
CHIPS = {"B": dict(rev="B", tiles="4x2", fe_clk=881, fe_clk_last=897, fmax_ss=49.1),
         "A": dict(rev="A", tiles="6x2", fe_clk=945, fe_clk_last=945, fmax_ss=46.2)}
T_CEM, RD_EARLY, SCRATCH, FS, HOP = 8000.0, 40.0, 0x7FFF00, 16000, 512


def _mem_timing(fclk, div, n, bytes_):
    tclk = 1e9 / fclk; tsck = 2 * div * tclk; gap = (2 * div + 1) * tclk
    B = 1 << (n + 2); Bs = min(B, 32)
    burst = lambda b: (20 + 2 * b) * tsck + gap
    frame = (-(-bytes_ // B) + 1) * burst(B) + 48 * (16 * tsck + gap) + (-(-48 // Bs) + 1) * burst(Bs)
    return tsck, (20 + 2 * B) * tsck, frame / 1e6


def chip_setup(fclk, src="pdm", mem="ramA", nbytes=10412, base=0, thr=12, n=4, nfill=2, rev="B", div=0, rd_dly=-1, pdm_edge=False):
    """Register values for clock fclk (Hz). src: pdm | pcm | feat; mem: flash | ramA | ramB.
    Returns (regs, info, errors, warnings); regs is a list of (addr, value) to write with RUN = 0, CTRL last."""
    chip = CHIPS[rev]; err, warn, d = [], [], {}
    tclk = 1e9 / fclk
    if fclk > 50e6: warn.append("above the 50 MHz design constraint")
    elif fclk > chip["fmax_ss"] * 1e6: warn.append(f"above the {chip['tiles']} all-corner limit ({chip['fmax_ss']} MHz)")
    pdm_div = 23
    if src == "pdm":
        min_div = -(-chip["fe_clk_last"] // 128) - 1
        pdm_div = min(255, max(min_div, int(fclk / (2 * FS * 64) + 0.5) - 1))
        d["fs"] = fclk / (2 * (pdm_div + 1)) / 64; d["clk_per_sample"] = 128 * (pdm_div + 1)
        dev = (d["fs"] / FS - 1) * 100
        if abs(dev) > 3: err.append(f"PDM gives f_s {d['fs']/1e3:.2f} kHz; with PDM_DIV >= {min_div} 16 kHz needs f_clk >= {128*(min_div+1)*FS/1e6:.3f} MHz")
        elif abs(dev) > 1: warn.append(f"f_s {dev:+.1f} % off 16 kHz")
    elif src == "pcm":
        d["fs"] = FS; d["clk_per_sample"] = fclk / FS
        if d["clk_per_sample"] < chip["fe_clk_last"]: err.append(f"PCM at 16 kHz needs f_clk >= {chip['fe_clk_last']*FS/1e6:.2f} MHz")
    else:
        d["fs"] = FS
    flash = mem == "flash"; nmax = 5 if flash else 3

    def pick(dv):
        for b in range(nmax, -1, -1):
            if flash or _mem_timing(fclk, dv, b, nbytes)[1] <= T_CEM: return b
        return -1
    if not div:
        div = 2; n2 = pick(2)
        if n2 < 0 or _mem_timing(fclk, 2, n2, nbytes)[2] > 8: div = 1
    burst = pick(div)
    if burst < 0: err.append("no burst length meets PSRAM tCEM at this SCK")
    tsck, cs_low, nn_ms = _mem_timing(fclk, div, max(burst, 0), nbytes)
    frame_ms = HOP / d["fs"] * 1e3
    if nn_ms / frame_ms > 0.9: err.append("model stream does not fit in the frame")
    elif nn_ms / frame_ms > 0.25: warn.append(f"NN uses {100*nn_ms/frame_ms:.0f} % of the frame (NN-3: <= 25 %)")
    if rd_dly < 0:
        lo, hi = RD_EARLY, tsck; mid = (lo + hi) / 2; best, bd = -1, 1e9
        for r in range(8):
            t = (r + 1) * tclk
            if t < lo or t > hi: continue
            e = abs(t - mid)
            if e < bd - 1e-6 or (abs(e - bd) < 1e-6 and r > best): bd, best = e, r
        if best < 0: best = 2; warn.append("no rd_dly inside the expected read window; sweep on the board")
        rd_dly = best
    cs = 0 if flash else 2 if mem == "ramB" else 1
    if base + nbytes > (1 << 24 if flash else 1 << 23): err.append("model does not fit the memory")
    if mem == "ramB" and base < SCRATCH + 48 and base + nbytes > SCRATCH: err.append("model overlaps the h' scratch in PSRAM B")
    ctrl = RUN | HOP512 | CLEAR | (SRC_PCM if src == "pcm" else 0) | (FEAT_HOST if src == "feat" else 0) | (32 if pdm_edge else 0)
    regs = [(PDMDIV, pdm_div), (MEMCFG, (rd_dly << 5) | (div << 2) | cs), (BURST, max(burst, 0)),
            (BASE0, base & 255), (BASE0 + 1, base >> 8 & 255), (BASE0 + 2, base >> 16 & 255),
            (THR, thr & 255), (DETCFG, (nfill & 15) << 4 | (n & 15)), (CTRL, ctrl)]
    d.update(div=div, rd_dly=rd_dly, burst=burst, cs_low_ns=cs_low, nn_ms=nn_ms, frame_ms=frame_ms)
    return regs, d, err, warn
