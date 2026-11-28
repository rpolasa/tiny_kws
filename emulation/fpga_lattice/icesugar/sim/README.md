# Co-simulation: FPGA build + Pico 2 W Tiny Tapeout QSPI Pmod emulator

`tb_fpga.v` instantiates `top_icesugar` (with `-DSIM`, PLL bypassed) and the unchanged chip RTL. It adds board delays and a bus-contention monitor. The Pico pins are CS0 GP7, CS1 GP8, CS2 GP9, SD0..3 GP10..13 and SCK GP14 (r15).

`pio_model.py` runs the PIO opcodes that pioasm generated from `pico_psram_host/src/qspi_psram.pio`, read from `qspi_psram.pio.h`. Regenerate that header after editing the `.pio`. The model covers:
- the 2-cycle input synchroniser;
- IN_COUNT masking, `mov x, pins`, IRQ set/wait (visible to the other SMs the next cycle) and delays;
- autopush/autopull and FIFO depths;
- SMx_INSTR injection by DMA;
- the core-1 server: memory chosen from the CS pins, flash bursts up to 128 B, PSRAM bursts up to 32 B, cut bursts dropped.

`test_fpga_pico.py` drives FEAT_HOST frames over SPI and compares every frame with the golden model from the tiny_kws repo. The model is streamed from the memory selected by MEMCFG.cs; the other two memories are blanked. It also checks that the model is read only on its CS, that writes appear only on CS2, and that no transaction has a bad CS.

    make -f cocotb.mk KWS_REPO=/path/to/tiny_kws                                   # div 2/rd 1 and div 1/rd 0, model in flash, PSRAM A, PSRAM B
    make -f cocotb.mk KWS_CS=0 KWS_BURST=5                                         # flash with 128 B bursts
    make -f cocotb.mk KWS_EXPECT=report KWS_FRAMES=2 KWS_CASES=2:0,2:1,2:2,2:3,2:4,1:0,1:1,1:2,1:3   # read-window map
    make -f cocotb.mk KWS_CASES=1:0 PICO_DMA_LAT=20 PICO_CORE_LAT=60 PICO_WORD_COST=25           # slow-Pico stress
    make -f cocotb.mk PICO_SCK_GLITCH=4:2:2 PICO_CS_GLITCH=40                      # SCK crosstalk spikes, CS spikes

`glitch_sim.py` runs without Verilog. A Python master generates the kws_qspi waveforms with reads on all three CS and h′ writes, and the script counts wrong nibbles, reads served by the wrong memory and lost writes:

    CASES=7:3,2:1,1:0 GL=4:2:2 python3 glitch_sim.py qspi_psram.pio.h 0,4,11       # hold-off 0 / 4 / 11

Results: see `../../README.md` section 5.
