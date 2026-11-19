# SPDX-License-Identifier: Apache-2.0
"""Extra bit-exact stress tests for the area-optimised RTL (rev B).

test_07: random full-range models (int8 weights -128..127, 24-bit biases up to the limits, C = 16,
         several shift sets) with random full-range features.  Exercises the single-accumulator
         n row (clamp of the parked x part on both sides), saturations and the 16-class path.
"""
import os, sys
import numpy as np
import cocotb
from cocotb.triggers import ClockCycles
import test as T
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "model"))
import gru_int

HERE = os.path.dirname(os.path.abspath(__file__))
NFR = int(os.environ.get("KWS_STRESS_FRAMES", "10"))


@cocotb.test()
async def test_07_nn_stress_random_models(dut):
    for i, name in enumerate(("a", "b", "c")):
        blob = open(os.path.join(HERE, "..", "stress", f"blob_{name}.bin"), "rb").read()
        ig = gru_int.IntGRU(gru_int.unpack_blob(blob, T.H))
        rng = np.random.default_rng(100 + i)
        feats = rng.integers(0, 256, (NFR, T.NCH)).astype(np.uint8)
        feats[0] = 0; feats[1] = 255
        # statistics of the parked value so the log shows the clamp is really exercised
        p = ig.p; h = ig.init_state(1); lo = hi = mid = 0
        for t in range(NFR):
            x = np.clip(feats[t:t + 1].astype(np.int64) - p["XOFF"], -128, 127)
            ax = (p["Bnx"][None, :] + x @ p["Wn"][:, :gru_int.F].T) >> p["Sn"]
            lo += int((ax < -256).sum()); hi += int((ax > 255).sum()); mid += int(((ax >= -256) & (ax <= 255)).sum())
            h, _ = ig.step(feats[t:t + 1].astype(np.int64), h)
        dut._log.info(f"model {name}: parked x-part below/inside/above clamp = {lo}/{mid}/{hi}")
        host = await T.setup(dut)
        await T.run_bypass(dut, host, blob, [feats], ig, thr=4, n=2)
        T.check_mem_models(dut)
