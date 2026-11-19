import cocotb
from cocotb.triggers import ClockCycles
import test as T, golden

@cocotb.test()
async def test_run_toggle_midframe(dut):
    host = await T.setup(dut)
    blob, ig = golden.load_model(T.H)
    T.load_mem(dut, blob)
    await host.wr(T.CTRL, T.RUN | T.FEAT_HOST)
    await host.wr(T.FEAT, *([100] * T.NCH))          # frame starts
    await ClockCycles(dut.clk, 3000)                 # mid-frame
    st0 = await host.rd(T.STATUS)
    await host.wr(T.CTRL, T.FEAT_HOST)               # RUN = 0 mid-frame
    await ClockCycles(dut.clk, 20000)
    st1 = await host.rd(T.STATUS)
    await host.wr(T.CTRL, T.RUN | T.FEAT_HOST)       # RUN = 1 again, no CLEAR
    await ClockCycles(dut.clk, 300000)               # > 2 frame times
    st2 = await host.rd(T.STATUS)
    await host.wr(T.FEAT, *([100] * T.NCH))
    await ClockCycles(dut.clk, 300000)
    st3 = await host.rd(T.STATUS)
    fc = await host.rd(T.FCNT0)
    dut._log.info(f"RESULT status mid-frame={st0:#04x} after RUN=0={st1:#04x} after RUN=1={st2:#04x} after new features={st3:#04x} frames={fc}")
    await host.wr(T.CTRL, T.RUN | T.FEAT_HOST | T.CLEAR)
    await ClockCycles(dut.clk, 200)
    await host.wr(T.FEAT, *([100] * T.NCH))
    await ClockCycles(dut.clk, 300000)
    st4 = await host.rd(T.STATUS); fc2 = await host.rd(T.FCNT0)
    dut._log.info(f"RESULT after CLEAR + features: status={st4:#04x} frames={fc2}")
