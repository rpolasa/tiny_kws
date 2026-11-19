"""Integer (bit-exact) reference of the tiny_kws GRU engine + model-blob packer.

This is the golden model for src/kws_nn.v. All arithmetic is integer.

Per frame:
  v = [x(F) ; h(H)]          x_k = clamp(feat_k - XOFF, -128, 127),  h int8 (h_real = h/128)
  for j in 0..H-1:
     r   : a = Br_j + sum Wr_j . v ; q = sat8(a >>> Sr) ; r7 = clamp(q + 64, 0, 128)
     z   : same with Wz, Bz, Sz                         -> z7
     n   : ax = Bnx_j + sum Wn_j[0:F] . x ; ah = Bnh_j + sum Wn_j[F:] . h
           t  = (ax >>> Sn) + ((r7 * sat8(ah >>> Sn)) >>> 7) ; n8 = clamp(t, -128, 127)
     h'_j = n8 + ((z7 * (h_j - n8)) >>> 7)
  h <- h'
  for c in 0..C-1: logit_c = sat8((Bc + sum Wc . h) >>> Sfc)
  top1/top2 over logit8 (lowest index wins ties)
Accumulators are 26-bit signed with saturation; biases are 24-bit signed.

Blob layout (bytes, little-endian biases):
  hdr[8] = {0xA5, C, XOFF, Sr, Sz, Sn, Sfc, H}
  for j: Br(3) Wr(F+H) | Bz(3) Wz(F+H) | Bnx(3) Bnh(3) Wn(F+H)
  for c: Bc(3) Wc(H)
"""
import numpy as np

F = 16   # number of filterbank features (= front-end channels)
ACC_W = 26
MAGIC = 0xA5


def sat(v, w):
    lo, hi = -(1 << (w - 1)), (1 << (w - 1)) - 1
    return np.clip(v, lo, hi)


def asr(v, s):
    return np.right_shift(v, s)  # numpy >> on signed ints is arithmetic (floor)


class IntGRU:
    def __init__(self, p):
        """p: dict with int arrays Wr,Wz,Wn (H,F+H) Br,Bz,Bnx,Bnh (H,) Wfc (C,H) Bfc (C,) and ints XOFF,Sr,Sz,Sn,Sfc"""
        self.p = {k: (np.asarray(v, np.int64) if not isinstance(v, int) else v) for k, v in p.items()}
        self.H = self.p["Wr"].shape[0]
        self.C = self.p["Wfc"].shape[0]

    def init_state(self, n=1):
        return np.zeros((n, self.H), np.int64)

    def step(self, feat, h):
        """feat: (N,F) uint8, h: (N,H) int64 -> (h_new, logits (N,C), top1, top2val, top1val)"""
        p = self.p
        x = np.clip(feat.astype(np.int64) - p["XOFF"], -128, 127)
        v = np.concatenate([x, h], 1)
        aw = lambda W, B, vv: sat(B[None, :] + vv @ W.T, ACC_W)
        r = np.clip(sat(asr(aw(p["Wr"], p["Br"], v), p["Sr"]), 8) + 64, 0, 128)
        z = np.clip(sat(asr(aw(p["Wz"], p["Bz"], v), p["Sz"]), 8) + 64, 0, 128)
        ax = aw(p["Wn"][:, :F], p["Bnx"], x)
        ah = aw(p["Wn"][:, F:], p["Bnh"], h)
        t = asr(ax, p["Sn"]) + asr(r * sat(asr(ah, p["Sn"]), 8), 7)
        n = np.clip(t, -128, 127)
        hn = n + asr(z * (h - n), 7)
        lg = sat(asr(aw(p["Wfc"], p["Bfc"], hn), p["Sfc"]), 8)
        return hn, lg

    def run(self, feats):
        """feats (N,T,F) -> logits (N,T,C) int"""
        N, T, _ = feats.shape
        h = self.init_state(N)
        out = np.zeros((N, T, self.C), np.int64)
        for t in range(T):
            h, out[:, t] = self.step(feats[:, t], h)
        return out


def top2(lg):
    """lg (C,) int -> (idx1, v1, v2). lowest index wins ties, as in RTL."""
    i1 = int(np.argmax(lg))
    v1 = int(lg[i1])
    rest = np.delete(lg, i1)
    v2 = int(rest.max()) if len(rest) else -128
    return i1, v1, v2


def b24(v):
    v = int(v)
    assert -(1 << 23) <= v < (1 << 23), v
    v &= 0xFFFFFF
    return bytes([v & 255, (v >> 8) & 255, (v >> 16) & 255])


def pack_blob(p):
    H = p["Wr"].shape[0]; C = p["Wfc"].shape[0]
    hdr = bytes([MAGIC, C, p["XOFF"], p["Sr"], p["Sz"], p["Sn"], p["Sfc"], H])
    out = bytearray(hdr)
    i8 = lambda W: bytes((np.asarray(W, np.int64) & 255).astype(np.uint8))
    for j in range(H):
        out += b24(p["Br"][j]) + i8(p["Wr"][j])
        out += b24(p["Bz"][j]) + i8(p["Wz"][j])
        out += b24(p["Bnx"][j]) + b24(p["Bnh"][j]) + i8(p["Wn"][j])
    for c in range(C):
        out += b24(p["Bfc"][c]) + i8(p["Wfc"][c])
    return bytes(out)


def unpack_blob(b, H):
    b = np.frombuffer(b, np.uint8)
    assert b[0] == MAGIC and b[7] == H
    C = int(b[1])
    p = dict(XOFF=int(b[2]), Sr=int(b[3]), Sz=int(b[4]), Sn=int(b[5]), Sfc=int(b[6]))
    o = 8
    def rd24():
        nonlocal o
        v = int(b[o]) | int(b[o + 1]) << 8 | int(b[o + 2]) << 16; o += 3
        return v - (1 << 24) if v & 0x800000 else v
    def rdw(n):
        nonlocal o
        w = b[o:o + n].astype(np.int8).astype(np.int64); o += n
        return w
    D = F + H
    Wr = np.zeros((H, D), np.int64); Wz = Wr.copy(); Wn = Wr.copy()
    Br = np.zeros(H, np.int64); Bz = Br.copy(); Bnx = Br.copy(); Bnh = Br.copy()
    for j in range(H):
        Br[j] = rd24(); Wr[j] = rdw(D)
        Bz[j] = rd24(); Wz[j] = rdw(D)
        Bnx[j] = rd24(); Bnh[j] = rd24(); Wn[j] = rdw(D)
    Wfc = np.zeros((C, H), np.int64); Bfc = np.zeros(C, np.int64)
    for c in range(C):
        Bfc[c] = rd24(); Wfc[c] = rdw(H)
    p.update(Wr=Wr, Wz=Wz, Wn=Wn, Br=Br, Bz=Bz, Bnx=Bnx, Bnh=Bnh, Wfc=Wfc, Bfc=Bfc)
    assert o == len(b)
    return p


def blob_size(H, C):
    return 8 + H * (3 * (F + H) + 12) + C * (H + 3)
