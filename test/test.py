# SPDX-License-Identifier: Apache-2.0
"""cocotb tests for tt_um_rakesh_tiny_kws.

Every check is bit-exact against the golden model (model/kws_fe.c + model/gru_int.py).
Set KWS_LONG=1 for the long regression (more clips/frames); default is CI-sized.
"""
import os, random
import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
import golden

H = int(os.environ.get("KWS_H", "48"))
NCH = 16
LONG = os.environ.get("KWS_LONG", "0") == "1"
GL = os.environ.get("GATES", "no") == "yes"
CLK_NS = 20

# register map
CTRL, PDMDIV, MEMCFG, BURST, BASE0, BASE1, BASE2, THR, DETCFG = range(9)
PCM, FEAT = 0x10, 0x11
STATUS, RESULT, TOP1, TOP2, FCNT0, FCNT1, ID, HREG = range(0x20, 0x28)
FEATRD = 0x30
RUN, SRC_PCM, FEAT_HOST, HOP512, CLEAR, PDM_EDGE = 1, 2, 4, 8, 16, 32


class Host:
    """bit-banged SPI master on ui_in[3:1] (mode 0, SCK = clk/8)"""
    def __init__(self, dut):
        self.dut = dut
        self.ui = 0b1000  # CSn high

    def _set(self):
        self.dut.ui_ctl.value = self.ui

    async def xfer(self, data, nread=0):
        d = self.dut
        out = []
        self.ui &= ~0b1000; self.ui &= ~0b0010; self._set()
        await ClockCycles(d.clk, 4)
        for byte in list(data) + [0] * nread:
            rb = 0
            for b in range(7, -1, -1):
                self.ui = (self.ui & ~0b0100) | (((byte >> b) & 1) << 2); self._set()
                await ClockCycles(d.clk, 4)
                self.ui |= 0b0010; self._set()
                await ClockCycles(d.clk, 1)
                rb = (rb << 1) | (int(d.uo_out.value) >> 1 & 1)
                await ClockCycles(d.clk, 3)
                self.ui &= ~0b0010; self._set()
            out.append(rb)
        await ClockCycles(d.clk, 4)
        self.ui |= 0b1000; self._set()
        await ClockCycles(d.clk, 6)
        return out[len(data):]

    async def wr(self, addr, *vals):
        await self.xfer([addr & 0x7F] + list(vals))

    async def rd(self, addr, n=1):
        r = await self.xfer([0x80 | addr], nread=n)
        return r if n > 1 else r[0]


def make_clock(sig, ns):
    try:
        return Clock(sig, ns, unit="ns")       # cocotb >= 2.0
    except TypeError:
        return Clock(sig, ns, units="ns")      # cocotb 1.x


async def setup(dut, memcfg=None):
    cocotb.start_soon(make_clock(dut.clk, CLK_NS).start())
    dut.ena.value = 1
    dut.ui_ctl.value = 0b1000
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 80)       # internal state clear sequences
    host = Host(dut)
    if memcfg is not None:
        await host.wr(MEMCFG, memcfg)
    return host


def load_mem(dut, blob, which="ramA", base=0):
    mem = getattr(dut, which).mem
    for i, b in enumerate(blob):
        mem[base + i].value = b


async def wait_frame(dut, host, timeout_cycles=2_000_000):
    """wait for the FRAME pin to toggle"""
    start = int(dut.uo_out.value) >> 3 & 1
    for _ in range(timeout_cycles // 64):
        await ClockCycles(dut.clk, 64)
        if (int(dut.uo_out.value) >> 3 & 1) != start:
            return
    raise AssertionError("frame timeout")


async def read_result(host):
    r = await host.rd(RESULT, 3)
    s8 = lambda v: v - 256 if v > 127 else v
    return (r[0] >> 4, s8(r[1]), s8(r[2])), r[0] & 15


def check_mem_models(dut):
    for m in ("flash", "ramA", "ramB"):
        mm = getattr(dut, m)
        assert int(mm.err_cem.value) == 0, f"{m}: tCEM violation"
        assert int(mm.err_cmd.value) == 0, f"{m}: bad command"
        assert int(mm.err_page.value) == 0, f"{m}: page crossing"
    assert int(dut.contention.value) == 0, "QSPI bus contention"
    assert int(dut.flash.nwrites.value) == 0 and int(dut.ramA.nwrites.value) == 0, "writes outside RAM B"


# ----------------------------------------------------------------------------------------------
@cocotb.test()
async def test_01_registers(dut):
    host = await setup(dut)
    assert await host.rd(ID) == 0x4B
    assert await host.rd(HREG) == H
    assert await host.rd(MEMCFG) == 0x49           # defaults: rd_dly=2 div=2 cs=1
    await host.wr(THR, 0x5A, 0x37)                   # auto-increment THR, DETCFG
    assert await host.rd(THR, 2) == [0x5A, 0x37]
    await host.wr(BASE0, 0x12, 0x34, 0x56)
    assert await host.rd(BASE0, 3) == [0x12, 0x34, 0x56]
    # uio released while not running
    assert int(dut.uio_oe.value) == 0
    await host.wr(CTRL, RUN | FEAT_HOST)
    assert int(dut.uio_oe.value) & 0b11001001 == 0b11001001   # CS0 SCK CS1 CS2 driven
    assert int(dut.uio_out.value) & 0b11000001 == 0b11000001  # all CS high when idle


async def run_bypass(dut, host, blob, feats_list, ig, memname="ramA", nfill=2, thr=16, n=3):
    load_mem(dut, blob, memname)
    await host.wr(THR, thr, (nfill << 4) | n)
    await host.wr(CTRL, RUN | FEAT_HOST)
    for feats in feats_list:
        await host.wr(CTRL, RUN | FEAT_HOST | CLEAR)        # fresh hidden state per clip
        await ClockCycles(dut.clk, 100)
        exp = golden.nn_frames(ig, feats)
        det_exp = golden.detect_seq(exp, nfill, thr, n)
        for t in range(feats.shape[0]):
            await host.wr(FEAT, *[int(v) for v in feats[t]])
            await wait_frame(dut, host)
            got, dcls = await read_result(host)
            assert got == tuple(exp[t]), f"frame {t}: got {got} expected {exp[t]}"
            assert (int(dut.uo_out.value) >> 2 & 1) == int(det_exp[t]), f"frame {t}: DETECT pin"
            if det_exp[t]:
                assert dcls == exp[t][0] and (int(dut.uo_out.value) >> 4) == exp[t][0]
        dut._log.info(f"clip ok: final top1={exp[-1]}  detections={sum(det_exp)}")
    st = await host.rd(STATUS)
    assert st & 0b1100 == 0, f"status {st:#x}"


@cocotb.test()
async def test_02_nn_bitexact_bypass(dut):
    """host-supplied features -> GRU/FC output identical to golden every frame"""
    host = await setup(dut)
    blob, ig = golden.load_model(H)
    pcm, y = golden.vectors()
    nclip = 4 if LONG else (1 if GL else 2)
    feats = [golden.fe_pcm(pcm[i], 512) for i in range(nclip)]
    if GL:
        feats = [f[:6] for f in feats]
    await run_bypass(dut, host, blob, feats, ig, thr=4, n=2)
    check_mem_models(dut)


@cocotb.test(skip=GL)
async def test_03_qspi_variants(dut):
    """different SCK dividers, capture delays, burst lengths and chip selects"""
    blob, ig = golden.load_model(H)
    pcm, _ = golden.vectors()
    feats = [golden.fe_pcm(pcm[0], 512)[:4]]
    #            memcfg(rd_dly<<5 | div<<2 | cs), burst, memory
    cases = [((1 << 5) | (1 << 2) | 1, 3, "ramA"),     # SCK = clk/2
             ((3 << 5) | (2 << 2) | 2, 2, "ramB"),     # RAM B, 16-byte bursts
             ((4 << 5) | (3 << 2) | 1, 1, "ramA"),     # slow SCK, 8-byte bursts (tCEM)
             ((2 << 5) | (2 << 2) | 0, 5, "flash")]    # flash, 128-byte bursts (h' scratch stays in RAM B)
    for memcfg, burst, mem in cases:
        host = await setup(dut, memcfg)
        await host.wr(BURST, burst)
        await run_bypass(dut, host, blob, feats, ig, memname=mem)
        check_mem_models(dut)
        dut._log.info(f"memcfg={memcfg:#04x} burst={burst} {mem}: ok")


@cocotb.test(skip=GL)
async def test_04_model_error(dut):
    host = await setup(dut)
    blob, ig = golden.load_model(H)
    bad = bytearray(blob); bad[7] ^= 1               # wrong H in header
    load_mem(dut, bad)
    await host.wr(CTRL, RUN | FEAT_HOST)
    await host.wr(FEAT, *([100] * NCH))
    await ClockCycles(dut.clk, 3000)
    st = await host.rd(STATUS)
    assert st & 0b1000, "model error not flagged"
    assert st & 1 == 0, "engine must return to idle"
    await host.wr(STATUS, 0b1000)
    assert (await host.rd(STATUS)) & 0b1000 == 0


@cocotb.test()
async def test_05_pcm_frontend_bitexact(dut):
    """host PCM -> filterbank features + classifier bit-exact (features read back every frame)"""
    host = await setup(dut)
    blob, ig = golden.load_model(H)
    load_mem(dut, blob)
    pcm, _ = golden.vectors()
    hop = 256 if GL else 512
    nfr = 2 if GL else (31 if LONG else 6)
    x = pcm[1][: nfr * hop].astype(np.int64)
    gfe = golden.fe_pcm(x, hop)
    gnn = golden.nn_frames(ig, gfe)
    await host.wr(CTRL, RUN | SRC_PCM | (HOP512 if hop == 512 else 0))
    t = 0
    for i in range(len(x)):
        v = int(x[i]) & 0xFFFF
        await host.xfer([PCM, v & 255, v >> 8])
        await ClockCycles(dut.clk, 880)          # >= 1041 clk per sample incl. SPI time
        if (i + 1) % hop == 0:
            await wait_frame(dut, host)
            f = await host.rd(FEATRD, NCH)
            assert f == [int(v) for v in gfe[t]], f"frame {t}: features {f} != {list(gfe[t])}"
            got, _ = await read_result(host)
            assert got == tuple(gnn[t]), f"frame {t}: nn {got} != {gnn[t]}"
            t += 1
    st = await host.rd(STATUS)
    assert st & 0b1100 == 0, f"status {st:#x} (overrun/model error)"
    dut._log.info(f"{t} PCM frames bit-exact")


@cocotb.test(skip=GL)
async def test_06_pdm_frontend_bitexact(dut):
    """PDM microphone -> CIC -> features bit-exact (hop 256)"""
    host = await setup(dut)
    blob, ig = golden.load_model(H)
    load_mem(dut, blob)
    pcm, _ = golden.vectors()
    nfr = 6 if LONG else 3
    bits = golden.pdm_modulate(pcm[2][: (nfr + 1) * 256], 64)
    with open("pdm_bits.mem", "w") as fh:
        fh.write("\n".join(str(int(b)) for b in bits))
    dut.pdm_on.value = 1
    dut.pdm_load.value = 1
    await ClockCycles(dut.clk, 2)
    # captured stream = [initial mic level 0] + bits
    gfe, _ = golden.fe_pdm(np.concatenate([[0], bits]).astype(np.uint8), 256)
    gnn = golden.nn_frames(ig, gfe)
    await host.wr(PDMDIV, 8)                     # PDM clk = clk/18 (sim speed); fs = clk/1152
    await host.wr(CTRL, RUN)
    for t in range(nfr):
        await wait_frame(dut, host, 10_000_000)
        f = await host.rd(FEATRD, NCH)
        assert f == [int(v) for v in gfe[t]], f"frame {t}: features {f} != {list(gfe[t])}"
        got, _ = await read_result(host)
        assert got == tuple(gnn[t])
    st = await host.rd(STATUS)
    assert st & 0b1100 == 0, f"status {st:#x}"
    dut._log.info(f"{nfr} PDM frames bit-exact")
