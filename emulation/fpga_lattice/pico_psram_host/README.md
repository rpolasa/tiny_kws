# kws_fpga_host: Pico 2 W as the TT QSPI Pmod (flash + 2 PSRAMs) and board-B host for the tiny_kws FPGA emulation

See `../README.md` for wiring, tools and bring-up steps F0–F6.

**Open:** VS Code, Raspberry Pi Pico panel, **Import Project**, this folder. Board Pico 2 W, SDK 2.3.1. Then **Compile** and **Run** (BOOTSEL). A ready image is in `prebuilt/kws_fpga_host.uf2`.

**What runs where (r15: Tiny Tapeout QSPI Pmod emulation, flash CS0 / PSRAM A CS1 / PSRAM B CS2):**
- `src/qspi_psram.pio`:
  - **PIO0 SM2 `qspi_csdet`:** reads CS0..CS2 (GP7..9) as one 3-bit value (IN_COUNT 3) and raises IRQ 4 when a CS falls.
  - **PIO0 SM0 `qspi_rx`:** takes 16 nibbles (8 command + 6 address + 2), raises IRQ 5 and parks.
  - **PIO0 SM1 `qspi_tx`:** counts the 4 dummy edges and then drives the read data, with the SCK hold-off.
  - **PIO1 watchers:** when all three CS have been high for ~95 ns (glitch filter), they inject `jmp start` into rx and tx through 2 DMA channels. This re-arms them after normal and cut-short bursts, with no CPU involved.
  - **PIO1 diagnostics:** the bus sniffer (GP7..14 per SCK edge) and a raw CS-edge counter, used by `d`.
- `src/psram_emu.c`, on core 1 (runs from RAM):
  - identifies the memory from the CS pins when the command word arrives;
  - decodes 0xEB reads and 0x38 writes;
  - queues the burst, up to 128 B for flash and 32 B for PSRAM, into the joined 8-word TX FIFO, waiting for space;
  - drops the rest of a burst the chip cut short, and flushes left-over words.
  - **Memory:** a 64 KB model window at 0x000000 for each memory, plus the PSRAM B h′ scratch at 0x7FFF00.
- `src/main.c` + `src/kws_chip.c`, on core 0: SPI0 host (1 MHz), tests F1/F3/F4, read-window sweep, live mode, USB console (`h` for help).
- `src/gen/`: model blob and golden vectors (same files as `kws_pico`).

**Options:**
- `-DSYS_KHZ=200000` (CMake `add_compile_definitions`) runs the Pico at 200 MHz for more PSRAM timing margin.
- The default QSPI preset is div 2 (4.08 MHz SCK); press `m` for div 1 (8.16 MHz).
