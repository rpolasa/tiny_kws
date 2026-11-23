# kws_pico: tiny_kws rev B on a Raspberry Pi Pico 2 W

This project runs the tiny_kws chip's whole signal chain on a Pico 2 W:

PDM mic → PIO at 1.024 MHz → DMA ring → CIC3 ÷64 → 16-channel IIR filter bank (512-sample hop) → log2 features → integer GRU (H = 48) plus classifier (12 classes) → detector → pins, USB and LED

Every arithmetic step is **bit-exact with the chip**:
- **Front end:** the build compiles the repo's own `model/kws_fe.c` unchanged; the copy lives in `src/ref/`.
- **CIC:** a fast word-wise CIC matches `fe_cic_pdm()` bit for bit.
- **GRU, classifier and detector:** ported line for line from `model/gru_int.py` and `test/golden.py::detect_seq`.

Those are the references the cocotb tests check the RTL against. The model blob that the chip reads from QSPI PSRAM sits in the Pico's flash here.

Model: `model/deploy/model.bin`, 10,412 B, md5 `e838370dc00e61f07f49e6c1cb506bd1`.
Classes: `_silence_ _unknown_ yes no up down left right on off stop go`.

## Wiring

| Adafruit 3492 (MP34DT01-M) | Pico 2 W | Chip pin it emulates |
|---|---|---|
| 3V | pin 36, 3V3(OUT) | |
| GND | pin 38, GND | |
| SEL | GND (data driven in the CLK-low phase) | |
| CLK | **GP2**, pin 4 | uo[0] PDM_CLK |
| DAT | **GP3**, pin 5 | ui[0] PDM_DAT |

Optional outputs for a logic analyser or LEDs (3.3 V):

| Output | Pico pin | Chip pin | Behaviour |
|---|---|---|---|
| GP6 | pin 9 | uo[3] FRAME | toggles every 32 ms frame |
| GP7 | pin 10 | uo[2] DETECT | high for the frame a keyword is detected |
| GP8–GP11 | pins 11, 12, 14, 15 | uo[4..7] DET_CLASS | class of the last detection |

The on-board LED lights for 0.4 s on each detection. It is on the CYW43 chip, hence `pico_cyw43_arch_none`.

## Build and flash (VS Code, Raspberry Pi Pico extension)
1. Open this folder in VS Code. In the Pico panel, run **Clean CMake** (or *Configure CMake*), because CMakeLists.txt has changed.
2. **Compile Project.** The output is `build/kws_pico.uf2`.
3. Hold **BOOTSEL**, plug in USB, then **Run Project** (picotool). You can also copy the `.uf2` onto the RP2350 drive.
4. Open the serial monitor (VS Code *Serial Monitor*, or PuTTY or Tera Term on the Pico's COM port; the baud rate doesn't matter for USB). The board waits up to 4 s for the terminal, blinking the LED three times (M0), then prints:

```
=== kws_pico: tiny_kws rev B emulation on Pico 2 W (sys clk 150 MHz) ===
model: 10412 bytes, md5 e838370dc00e61f07f49e6c1cb506bd1, H=48, 12 classes: _silence_ _unknown_ yes no ...
---- self-test vs golden model ----
M1 GRU+FC+detector : 12 clips x 31 frames, 9 detections, last-frame top1 = label in 12/12 clips
                     ... -> PASS (bit-exact)
M2 front end (PCM) : 4 clips, 124 frames, features bit-exact, PCM->NN top1/top2 bit-exact  -> PASS
M3 PDM path        : 1024000 bits -> 16000 samples; CIC (word-wise) bit-exact, ... features bit-exact -> PASS
---- self-test PASSED: Pico == golden model == RTL, bit for bit ----
live: PDM_CLK GP2 1024000.0 Hz (f_s 16000.00 Hz), PDM_DAT GP3, rising edge (mic SEL = GND) ...
>>> DETECT  yes     (class 2, margin 25, frame 412)
```

If the terminal connects late, press `t` to repeat the self-tests.

## Serial commands
| Key | Action |
|---|---|
| `t` | self-tests M1–M3 (bit-exact against the golden vectors) |
| `l` | live detection on or off |
| `c` | CLEAR: hidden state, detector and filters |
| `v` | per-frame top-1 class, logits and margin |
| `f` | per-frame 16 features (compare with the web app) |
| `m` | microphone level meter every 0.5 s |
| `e` | PDM capture edge: rising (SEL = GND, default) or falling (SEL = 3V) |
| `+` / `-` | THR ± 1 (default 12) |
| `]` / `[` | N ± 1 (default 4) consecutive frames |
| `s` | statistics: CPU load, NN µs per frame, ring lag, overruns, actual f_PDM and f_s |
| `h` | help |

## Bring-up plan
| Step | What | Pass when |
|---|---|---|
| M0 | LED blinks, banner on USB serial | you see both |
| M1 | GRU + classifier + detector on golden features | `M1 ... PASS (bit-exact)`: all 12 logits, top-1/top-2 and DETECT match on 372 frames |
| M2 | `kws_fe.c` on golden PCM, then NN end to end | `M2 ... PASS` |
| M3 | golden PDM bit stream (PIO word format) → CIC → features | `M3 ... PASS` |
| M4 | live mic | `m` shows the level rising when you speak; spoken keywords print `>>> DETECT` |
| M5 | real-time margin | `s`: CPU load well under 100 %, NN ≪ 32 ms per frame, overruns 0 |

M1–M3 run at every boot, before the microphone starts.

## Tuning in M4
- **Edge:** with `m` on, a flat level (min ≈ max) means you are sampling the wrong phase or the mic isn't wired. Press `e` and check again. The MP34DT01-M drives DAT while CLK is low when SEL = GND.
- **Detector:** a higher THR or N gives fewer false accepts but needs clearer speech. THR 12 with N 4 is the chip's balanced default. N 2–3 reacts to short words such as "up" and "on".
- **Distance and level:** speak at 0.3–1 m. Use `v` to watch the margin, and `f` to compare features with what the web app shows for the same word.
- **Clock:** the PIO runs at 4 × 1.024 MHz from 150 MHz through a fractional divider (36.62), so f_s is exact on average with about 7 ns of edge jitter. The mic accepts 1.0–3.25 MHz.

## Files
| Path | What |
|---|---|
| `src/main.c` | PIO and DMA capture, live loop, pins, LED, console |
| `src/pdm_mic.pio` | PDM_CLK generation and data capture (two variants for the two SEL settings) |
| `src/kws_fe_port.c/.h` | includes `src/ref/kws_fe.c`; word-wise CIC3 |
| `src/kws_nn.c/.h` | blob parser, integer GRU, classifier, top-2, detector |
| `src/kws_selftest.c/.h` | M1–M3 tests, shared by the Pico build and the PC build |
| `src/ref/` | unchanged copies of `model/kws_fe.c` and `model/fbank_coeffs.h` from the tiny_kws repo |
| `src/gen/` | model blob and golden vectors, generated (don't edit) |
| `tools/gen_headers.py` | regenerates `src/gen/` from the tiny_kws repo, after a model change for example |
| `host/host_main.c` | the same self-tests built for a PC |

## Regenerating the headers (only after a model change)
```
python tools/gen_headers.py --repo D:/MY/asic/tapeouts/TTsky26e/tiny_kws
```
This needs numpy, scipy, and gcc on PATH, because `test/golden.py` compiles `kws_fe.c`. Run it in WSL, MSYS2 or Linux.

## PC check (optional, needs gcc)
```
cd host
gcc -O2 -I../src -o kws_host host_main.c ../src/kws_nn.c ../src/kws_fe_port.c ../src/kws_selftest.c
./kws_host
```
