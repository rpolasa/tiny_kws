"""Golden reference for the cocotb tests: wraps the bit-exact C front end and integer GRU."""
import os, sys, numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "model"))
from kwsfe import FrontEnd, pdm_modulate  # noqa: E402
import gru_int  # noqa: E402

MODEL_BIN = os.environ.get("KWS_MODEL", os.path.join(HERE, "..", "model", "deploy", "model.bin"))
VECTORS = os.path.join(HERE, "vectors.npz")


def load_model(H):
    blob = open(MODEL_BIN, "rb").read()
    return blob, gru_int.IntGRU(gru_int.unpack_blob(blob, H))


def nn_frames(ig, feats):
    """feats (T,16) u8 -> list of (top1_idx, top1, top2) per frame"""
    h = ig.init_state(1)
    out = []
    for t in range(feats.shape[0]):
        h, lg = ig.step(feats[t:t + 1].astype(np.int64), h)
        out.append(gru_int.top2(lg[0]))
    return out


def detect_seq(tops, nfill, thr, n):
    last, run, res = 0, 0, []
    for (i, t1, t2) in tops:
        kw = i >= nfill and (t1 - t2) >= thr
        inc = (min(run + 1, 15) if i == last else 1)
        req = n if n else 1
        fire = kw and inc == req and not (i == last and run == 15)
        run = inc if kw else 0
        last = i
        res.append(fire)
    return res


def fe_pcm(x, hop):
    fe = FrontEnd()
    st = fe.new_state()
    return fe.stream_pcm(st, np.asarray(x, np.int16), hop)


def fe_pdm(bits, hop):
    fe = FrontEnd()
    st = fe.new_state()
    pcm = fe.cic(st, bits)
    st2 = fe.new_state()
    return fe.stream_pcm(st2, pcm, hop), pcm


def vectors():
    """test clips (int16 PCM, label) saved by model/make_vectors.py"""
    d = np.load(VECTORS)
    return d["pcm"], d["y"]
