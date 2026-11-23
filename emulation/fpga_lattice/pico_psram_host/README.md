# kws_fpga_host: Pico 2 W as QSPI PSRAM and board-B host for the tiny_kws FPGA emulation

See `../README.md` for wiring, tools and bring-up steps F0–F6.

**Open:** VS Code, Raspberry Pi Pico panel, **Import Project**, this folder. Board Pico 2 W, SDK 2.3.1. Then **Compile** and **Run** (BOOTSEL). A ready image is in `prebuilt/kws_fpga_host.uf2`.

**What runs where:**
- `src/qspi_psram.pio`:
  - PIO0 SM0 receives the command and address; PIO0 SM1 drives read data (side-set pindirs).
  - Two watchers on PIO1 see CS2 go high and stay high for ~100 ns (glitch filter), then, through 2 DMA channels, inject `jmp start` into PIO0 SM0 and SM1. This re-arms them after normal and cut-short bursts with no CPU involved, and ignores crosstalk spikes on CS2.
  - PIO1 also holds the diagnostic bus sniffer and a raw CS-edge counter used by `d`.
- `src/psram_emu.c`, on core 1 (runs from RAM):
  - decodes 0xEB reads and 0x38 writes;
  - queues exactly one burst (≤ 32 B = 8 words) into the joined TX FIFO;
  - flushes words left over from a burst the chip cut short.
  - Memory map: model at 0x000000 (64 KB area), h′ scratch at 0x7FFF00.
- `src/main.c` + `src/kws_chip.c`, on core 0: SPI0 host (1 MHz), tests F1/F3/F4, read-window sweep, live mode, USB console (`h` for help).
- `src/gen/`: model blob and golden vectors (same files as `kws_pico`).

**Options:**
- `-DSYS_KHZ=200000` (CMake `add_compile_definitions`) runs the Pico at 200 MHz for more PSRAM timing margin.
- The default QSPI preset is div 2 (4.08 MHz SCK); press `m` for div 1 (8.16 MHz).
