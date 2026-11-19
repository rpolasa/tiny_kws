"""Stress stimulus for the front-end equivalence test (full-scale noise, squares that saturate
the biquads, DC extremes, tiny signals).  Writes stim.txt (hex int16, one per line)."""
import numpy as np
rng = np.random.default_rng(1)
segs = [rng.integers(-32768, 32768, 3000),
        np.where(np.arange(2000) % 2 == 0, 32767, -32768)]
for p in (4, 8, 16, 37, 75, 150):
    segs.append(np.where((np.arange(1500) // p) % 2 == 0, 32767, -32768))
segs += [np.full(600, -32768), np.full(600, 32767), rng.integers(-3, 4, 2000),
         rng.normal(0, 2000, 4000).clip(-32768, 32767).astype(int)]
t = np.arange(4000)
segs.append((30000 * np.sin(2 * np.pi * 213 / 16000 * t)).astype(int))
segs.append((32767 * np.sign(np.sin(2 * np.pi * 5870 / 16000 * t))).astype(int))
x = np.concatenate(segs)
x = x[:(len(x) // 512) * 512]
open("stim.txt", "w").write("\n".join("%04x" % (int(v) & 0xFFFF) for v in x) + "\n")
print(len(x), "samples")
