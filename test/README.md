# tiny_kws testbench

cocotb testbench for `tt_um_rakesh_tiny_kws`. `tb.v` connects the design to three QSPI memory models
(`psram_model.v`: flash on CS0, PSRAM A on CS1, PSRAM B on CS2). Expected results come from the bit-exact
golden models in `../model` (`golden.py`), so the tests need `numpy` and `gcc` (the C front end is compiled on
first use).

```sh
make                                          # RTL, 6 tests
make SCANFF=yes                               # RTL with KWS_SKY130_SCANFF and the scan-flop stub
make COCOTB_TEST_MODULES=test_stress          # NN stress
make COCOTB_TEST_MODULES=test_runtoggle       # RUN toggle
make GATES=yes                                # gate level: copy the hardened netlist to gate_level_netlist.v first
```
