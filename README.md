![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# tiny_kws — streaming keyword spotter for Tiny Tapeout (SKY130, 4x2 tiles)

Always-on keyword spotting on a Tiny Tapeout tile: PDM microphone → CIC3 → 16-channel IIR filterbank →
int8 GRU (48 units) + linear classifier (12 classes, up to 16), weights streamed every 32 ms frame from the
Tiny Tapeout QSPI Pmod. Top module `tt_um_rakesh_tiny_kws`, target shuttle TTSKY26e.

- [Project documentation (datasheet page)](docs/info.md)
- [Design review, rev B changes and sign-off](docs/REVIEW_REVB.md)

| | rev B (this repository) |
|---|---|
| Tiles | 4x2 |
| Ring storage | 1 376 × `sky130_fd_sc_hd__sdfxtp_1` (scan flip-flops), `src/kws_sreg.v` |
| Utilisation | 73.9 % (reference run C, LibreLane 3.0.14) |
| Clock | 50 MHz typical (94 MHz f_max at tt) / 49.1 MHz worst corner (ss, 100 °C, 1.6 V) |
| Power | 9.4 mW (tt, 50 MHz, vectorless) |
| Function | Bit-exact with the integer golden model (`model/gru_int.py`, `model/kws_fe.c`) |

## Repository layout

| Path | Content |
|---|---|
| `src/` | RTL (`project.v`, `kws_*.v`), `config.json` (LibreLane), `kws.sdc` |
| `test/` | cocotb testbench: 6 regression tests (`test.py`), NN stress (`test_stress.py`), RUN-toggle (`test_runtoggle.py`) |
| `model/` | golden front end (`kws_fe.c`, `kwsfe.py`), integer GRU (`gru_int.py`), deployed model (`deploy/model.bin`) |
| `stress/`, `fe_stress/` | random full-range models for the stress test; front-end equivalence bench against `kws_fe.c` |
| `app/` | browser bench: bit-exact model, chip set-up calculator, experiences (`app/kws_live.html`) |
| `scripts/kws_host.py` | host driver (MicroPython/CPython) incl. `chip_setup()` register calculator |
| `signoff_runC/` | reference 4x2 run C: metrics |

## Running the tests locally

```sh
cd test
make                                                   # 6 RTL tests (plain-flop rings)
make SCANFF=yes                                        # same tests with the scan-flop rings (stub model)
make COCOTB_TEST_MODULES=test_stress                   # NN stress, 3 random full-range models
make COCOTB_TEST_MODULES=test_runtoggle                # RUN 1->0->1 mid-frame behaviour
cd ../fe_stress && bash run.sh                         # front-end equivalence (needs gcc, iverilog)
```

The GitHub Actions build the GDS (`gds`: harden, precheck, gate-level test, viewer), the datasheet (`docs`),
run the RTL tests (`test`) and, on demand, an iCE40UP5K bitstream (`fpga`).

## What is Tiny Tapeout?

Tiny Tapeout is an educational project that makes it easier and cheaper than ever to get your digital and
analog designs manufactured on a real chip. See https://tinytapeout.com.
