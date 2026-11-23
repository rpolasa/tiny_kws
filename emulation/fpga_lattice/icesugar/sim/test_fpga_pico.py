# SPDX-License-Identifier: Apache-2.0
"""Co-simulation: FPGA build (top_icesugar + unchanged tiny_kws RTL) <-> Pico 2 W PSRAM emulator.

The Pico is pio_model.PicoPsram executing the pioasm opcodes of pico_psram_host/src/qspi_psram.pio
at 150 MHz with a 2-cycle input synchroniser, DMA re-arm latency and core-1 latency.
The chip runs in FEAT_HOST mode (host features over SPI, model streamed from the emulated PSRAM B
on CS2, h' written to and read back from 0x7FFF00) and every frame is compared with the golden model.

    make -f cocotb.mk  (needs iverilog, cocotb 2.x, numpy, scipy, gcc; KWS_REPO = tiny_kws repo)
Environment: PICO_MHZ (150), KWS_FRAMES (3), KWS_CASES ("2:1,1:0" = div:rd_dly list),
             PICO_HOLDOFF (auto = 7*div-3 as the firmware, or a number; 0 = r12 PIO behaviour),
             PICO_SCK_GLITCH=bits:w:d (SCK crosstalk spikes, see pico_run), PICO_CS_GLITCH,
             PICO_DMA_LAT / PICO_CORE_LAT / PICO_WORD_COST (sys cycles: 8 / 12 / 10),
             KWS_EXPECT=report to only print the pass/fail map (read-window sweep)
"""
import os, sys
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer, FallingEdge

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.environ.get("KWS_REPO", os.path.join(HERE, "..", "..", "..", "..", "tapeouts", "TTsky26e", "tiny_kws"))
sys.path.insert(0, os.path.join(REPO, "test"))
sys.path.insert(0, HERE)
import golden                       # noqa: E402
from pio_model import PicoPsram     # noqa: E402

CTRL, MEMCFG, BURST, BASE0, THR, DETCFG = 0x00, 0x02, 0x03, 0x04, 0x07, 0x08
FEAT, STATUS, RESULT, ID = 0x11, 0x20, 0x21, 0x26
RUN, FEAT_HOST, CLEAR = 1, 4, 16
CLK_PS = 61302                       # 16.3125 MHz (PLL output)


class Host:
    """bit-banged SPI master on the FPGA's PMOD3 pins (mode 0, SCK = clk/8)"""
    def __init__(self, dut): self.d = dut

    async def xfer(self, data, nread=0):
        d = self.d
        d.spi_csn.value = 0; d.spi_sck.value = 0
        await ClockCycles(d.clk, 4)
        out = []
        for byte in list(data) + [0] * nread:
            rb = 0
            for b in range(7, -1, -1):
                d.spi_mosi.value = (byte >> b) & 1
                await ClockCycles(d.clk, 4)
                d.spi_sck.value = 1
                await ClockCycles(d.clk, 1)
                rb = (rb << 1) | int(d.spi_miso.value)
                await ClockCycles(d.clk, 3)
                d.spi_sck.value = 0
            out.append(rb)
        await ClockCycles(d.clk, 4)
        d.spi_csn.value = 1
        await ClockCycles(d.clk, 6)
        return out[len(data):]

    async def wr(self, a, *v): await self.xfer([a & 0x7F] + list(v))

    async def rd(self, a, n=1):
        r = await self.xfer([0x80 | a], n)
        return r if n > 1 else r[0]


def _bits(v):
    try:
        return int(v)
    except ValueError:
        return int(str(v).replace("z", "0").replace("Z", "0").replace("x", "0").replace("X", "0"), 2)


async def pico_run(dut, pico, period_ps):
    """PICO_CS_GLITCH=<cycles>: inject a 5-cycle (33 ns) high spike on CS2 as seen by the Pico, <cycles> sys
    cycles after each CS fall (mimics crosstalk on the CS wire; the FPGA does not see it)"""
    last = None
    glitch_at = int(os.environ.get("PICO_CS_GLITCH", "0"))
    # PICO_SCK_GLITCH=bits:w:d - crosstalk: when >= bits SD lines rise as the Pico switches them, the Pico sees
    # SCK high for w sys cycles starting d cycles later (only while SCK is low); the FPGA does not see it
    sg = os.environ.get("PICO_SCK_GLITCH", "")
    sg_bits, sg_w, sg_d = (int(v) for v in sg.split(":")) if sg else (0, 0, 0)
    pend, forced_until, ncyc, prev_val = [], -1, 0, 0
    pico.sck_glitches = 0
    since_fall, cs_prev = 0, 1
    pico.glitches = 0
    while True:
        if pico.idle() and _bits(dut.p_cs.value) == 1:
            await FallingEdge(dut.p_cs)
        await Timer(period_ps, unit="ps")
        sd, sck, cs = _bits(dut.p_sd.value), _bits(dut.p_sck.value), _bits(dut.p_cs.value)
        if glitch_at:
            since_fall = since_fall + 1 if (cs == 0 and cs_prev == 0) else 0
            cs_prev = cs
            if cs == 0 and glitch_at <= since_fall < glitch_at + 5:
                cs = 1
                if since_fall == glitch_at: pico.glitches += 1
        ncyc += 1
        if sg:
            while pend and pend[0][0] <= ncyc: forced_until = max(forced_until, pend.pop(0)[1])
            if ncyc < forced_until and cs == 0 and sck == 0: sck = 1
        val, oe = pico.step((sd << 10) | (sck << 14) | (cs << 15))
        if sg and oe and val != prev_val:
            if bin((val & ~prev_val) & oe & 15).count("1") >= sg_bits:
                pend.append((ncyc + sg_d, ncyc + sg_d + sg_w)); pico.sck_glitches += 1
        if oe: prev_val = val
        if (val, oe) != last:
            dut.pico_sd_out.value = val; dut.pico_sd_oe.value = oe; last = (val, oe)


async def wait_frame(dut, timeout_cycles=2_000_000):
    s = int(dut.frame.value)
    for _ in range(timeout_cycles // 32):
        await ClockCycles(dut.clk, 32)
        if int(dut.frame.value) != s: return True
    return False


@cocotb.test()
async def test_feat_host_pico_psram(dut):
    pico_mhz = float(os.environ.get("PICO_MHZ", "150"))
    nfr = int(os.environ.get("KWS_FRAMES", "3"))
    cases = [tuple(int(v) for v in c.split(":")) for c in os.environ.get("KWS_CASES", "2:1,1:0").split(",")]
    blob, ig = golden.load_model(48)
    pcm, _ = golden.vectors()
    feats = golden.fe_pcm(pcm[0], 512)[:nfr]
    exp = golden.nn_frames(ig, feats)
    thr, n, nfill = 4, 2, 2
    det_exp = golden.detect_seq(exp, nfill, thr, n)
    hdr = os.path.join(HERE, "qspi_psram.pio.h")

    cocotb.start_soon(Clock(dut.clk, CLK_PS, unit="ps").start())
    host = Host(dut)
    results = []
    # one Pico for the whole run, as on the bench (the chip is reset between settings)
    pico = PicoPsram(hdr, blob, burst=3, dma_lat=int(os.environ.get("PICO_DMA_LAT", "8")),
                     core_lat=int(os.environ.get("PICO_CORE_LAT", "12")),
                     word_cost=int(os.environ.get("PICO_WORD_COST", "10")))
    cocotb.start_soon(pico_run(dut, pico, int(round(1e6 / pico_mhz))))
    expect_pass = os.environ.get("KWS_EXPECT", "pass") == "pass"
    for div, rd in cases:
        for k in pico.stats: pico.stats[k] = 0
        # SCK hold-off as the firmware sets it (psram_holdoff_for_div at 150 MHz), PICO_HOLDOFF overrides
        ho = os.environ.get("PICO_HOLDOFF", "auto")
        pico.set_holdoff(max(0, min(31, 7 * div - 3)) if ho == "auto" else int(ho))
        c0 = int(dut.contention.value)
        dut.host_rst_n.value = 0
        await ClockCycles(dut.clk, 20)
        dut.host_rst_n.value = 1
        await ClockCycles(dut.clk, 200)                     # POR (SIM: 64 clk) + chip clear sequences
        assert await host.rd(ID) == 0x4B
        memcfg = (rd << 5) | (div << 2) | 2                  # model + scratch in PSRAM B (CS2)
        await host.wr(MEMCFG, memcfg)
        await host.wr(BURST, 3)
        base = int(os.environ.get("KWS_BASE", "0"), 0)
        if base:
            pico.mem_lo[base:base + len(blob)] = blob
        pico.trace = []
        await host.wr(BASE0, base & 255, (base >> 8) & 255, base >> 16)
        await host.wr(THR, thr, (nfill << 4) | n)
        await host.wr(CTRL, RUN | FEAT_HOST)
        await host.wr(CTRL, RUN | FEAT_HOST | CLEAR)
        await ClockCycles(dut.clk, 100)
        bad = 0
        t0 = cocotb.utils.get_sim_time("us")
        for t in range(nfr):
            await host.wr(FEAT, *[int(v) for v in feats[t]])
            ok = await wait_frame(dut)
            if not ok:
                assert not expect_pass, f"div {div} rd {rd} frame {t}: FRAME timeout (stats {pico.stats})"
                dut._log.info(f"div {div} rd {rd} frame {t}: FRAME timeout"); bad += 1; break
            r = await host.rd(RESULT, 3)
            s8 = lambda v: v - 256 if v > 127 else v
            got = (r[0] >> 4, s8(r[1]), s8(r[2]))
            det = int(dut.detect.value)
            if got != tuple(exp[t]) or det != int(det_exp[t]):
                bad += 1
                dut._log.info(f"div {div} rd {rd} frame {t}: got {got} det {det}, expected {tuple(exp[t])} det {int(det_exp[t])}")
        dt = (cocotb.utils.get_sim_time("us") - t0) / nfr
        st = await host.rd(STATUS)
        await host.wr(CTRL, 0)
        await ClockCycles(dut.clk, 50)
        cont = int(dut.contention.value) - c0
        dut._log.info("first words at the Pico: " + " ".join(f"{w:08x}" for w in pico.trace))
        if getattr(pico, "glitches", 0): dut._log.info(f"CS glitches injected so far: {pico.glitches}")
        if getattr(pico, "sck_glitches", 0): dut._log.info(f"SCK spikes injected so far: {pico.sck_glitches}")
        res = dict(div=div, rd=rd, holdoff=pico.holdoff, bad=bad, status=st, contention=cont, us_per_frame=round(dt, 1), **pico.stats)
        results.append(res)
        dut._log.info(f"MEMCFG 0x{memcfg:02x}: {res}")
    for r in results:
        dut._log.info(("PASS " if (r["bad"] == 0 and r["status"] & 0x0C == 0 and r["contention"] == 0
                                   and r["late"] == 0 and r["bad_cmd"] == 0) else "FAIL ") + str(r))
    if expect_pass:
        for r in results:
            assert r["bad"] == 0 and r["status"] & 0x0C == 0, f"mismatch / chip error: {r}"
            assert r["contention"] == 0, f"QSPI bus contention: {r}"
            assert r["late"] == 0 and r["bad_cmd"] == 0, f"PSRAM model: {r}"
