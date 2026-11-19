"""ctypes wrapper around the bit-exact C front-end (kws_fe.c)."""
import ctypes, os, subprocess, numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
NCH = 16


def _build(defs=()):
    tag = "_".join(f"{k}{v}" for k, v in defs) or "default"
    so = os.path.join(_HERE, f"libkwsfe_{tag}.so")
    src = os.path.join(_HERE, "kws_fe.c")
    hdr = os.path.join(_HERE, "fbank_coeffs.h")
    if not os.path.exists(so) or os.path.getmtime(so) < max(os.path.getmtime(src), os.path.getmtime(hdr)):
        cmd = ["gcc", "-O2", "-shared", "-fPIC", "-o", so, src] + [f"-D{k}={v}" for k, v in defs]
        subprocess.check_call(cmd)
    return ctypes.CDLL(so)


class FrontEnd:
    def __init__(self, defs=()):
        self.lib = _build(tuple(defs))
        self.lib.fe_state_size.restype = ctypes.c_int
        self.ssize = self.lib.fe_state_size()
        self.lib.fe_log2u8.restype = ctypes.c_uint8

    def batch_pcm(self, x, hop):
        """x: int16 array (N, L). returns uint8 (N, L//hop, NCH)"""
        x = np.ascontiguousarray(x, dtype=np.int16)
        N, L = x.shape
        out = np.zeros((N, L // hop, NCH), dtype=np.uint8)
        self.lib.fe_batch_pcm(x.ctypes.data_as(ctypes.c_void_p), N, L, hop, out.ctypes.data_as(ctypes.c_void_p))
        return out

    def new_state(self):
        st = ctypes.create_string_buffer(self.ssize)
        self.lib.fe_reset(st)
        return st

    def stream_pcm(self, st, x, hop):
        x = np.ascontiguousarray(x, dtype=np.int16)
        out = np.zeros((len(x) // hop + 1, NCH), dtype=np.uint8)
        n = self.lib.fe_run_pcm(st, x.ctypes.data_as(ctypes.c_void_p), len(x), hop,
                                out.ctypes.data_as(ctypes.c_void_p), out.shape[0])
        return out[:n]

    def cic(self, st, bits):
        bits = np.ascontiguousarray(bits, dtype=np.uint8)
        out = np.zeros(len(bits) // 64 + 1, dtype=np.int16)
        n = self.lib.fe_cic_pdm(st, bits.ctypes.data_as(ctypes.c_void_p), len(bits), out.ctypes.data_as(ctypes.c_void_p))
        return out[:n]


def pdm_modulate(x16, osr=64, seed=0):
    """2nd-order sigma-delta modulator (float) emulating a PDM MEMS mic. x16: int16 at fs -> bits at osr*fs."""
    from scipy.signal import resample_poly
    x = resample_poly(x16.astype(np.float64) / 32768.0, osr, 1)
    x = np.clip(x, -0.7, 0.7)
    bits = np.zeros(len(x), dtype=np.uint8)
    i1 = i2 = 0.0
    y = 0.0
    rng = np.random.default_rng(seed)
    dith = rng.uniform(-1e-3, 1e-3, len(x))
    for n in range(len(x)):
        i1 += x[n] - y
        i2 += i1 - y
        b = 1 if (i2 + dith[n]) >= 0 else 0
        y = 1.0 if b else -1.0
        bits[n] = b
    return bits
