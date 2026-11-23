# Co-simulation: FPGA build + Pico 2 W PSRAM emulator

`tb_fpga.v` instantiates `top_icesugar` (with `-DSIM`, PLL bypassed) and the unchanged chip RTL. It adds board delays and a bus-contention monitor.

`pio_model.py` runs the PIO opcodes that pioasm generated from `pico_psram_host/src/qspi_psram.pio`, read from `qspi_psram.pio.h`; regenerate that header after editing the `.pio`. It models the 2-cycle input synchroniser, side-set pindirs, autopush/autopull, FIFO depths, SMx_INSTR injection by DMA and core-1 latency.

`test_fpga_pico.py` drives FEAT_HOST frames over SPI, with the model streamed from the emulated PSRAM B, and compares every frame with the golden model from the tiny_kws repo.

    make -f cocotb.mk KWS_REPO=/path/to/tiny_kws                                   # div 2/rd 1 and div 1/rd 0, 3 frames each
    make -f cocotb.mk KWS_EXPECT=report KWS_FRAMES=2 KWS_CASES=2:0,2:1,2:2,2:3,2:4,1:0,1:1,1:2,1:3   # read-window map
    make -f cocotb.mk KWS_CASES=1:0 PICO_DMA_LAT=20 PICO_CORE_LAT=60 PICO_WORD_COST=25           # slow-Pico stress

Results on 2026-11-22: see `../../README.md` section 5.
