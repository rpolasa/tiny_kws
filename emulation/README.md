# tiny_kws emulation

Two ways to run the tiny_kws rev B design before silicon. Neither is part of the Tiny Tapeout submission: the GDS is built from `../src` only.

| Folder | What | Bench status (2026-11-23) |
|---|---|---|
| [`mcu_pico2w/kws_pico`](mcu_pico2w/kws_pico) | MCU-only emulation on a Raspberry Pi Pico 2 W. The front end and GRU are a bit-exact C port of the chip, running on a live PDM mic. | Live detection gives decent accuracy. |
| [`fpga_lattice`](fpga_lattice) | The unchanged chip RTL (`icesugar/rtl` = `../src`, see MD5SUMS) on an iCESugar v1.5 (iCE40UP5K). A Pico 2 W emulates the QSPI PSRAM and acts as the SPI host for tests. | All passed: F1 registers; F3, 372 of 372 golden frames bit-exact; the read-window sweep, which matches co-sim; F4, PCM through front end and NN bit-exact; live mic detection matching the MCU emulation. |

## Prebuilt images

| File | md5 | Board | Bench-tested (2026-11-23) |
|---|---|---|---|
| `fpga_lattice/icesugar/prebuilt/kws.bin` | `1f05561f02a8162453686e32443f46e1` | iCESugar v1.5: unchanged rev B RTL, PLL at 16.3125 MHz, nextpnr seed 4 (Fmax 17.55 MHz) | yes: F1, F3 372/372 bit-exact, `w` sweep, F4, live mic |
| `fpga_lattice/pico_psram_host/prebuilt/kws_fpga_host_r13.uf2` | `0c79c5713b207ff2aa8ed22f451bdd0d` | Pico 2 W PSRAM B emulator + test host, r13 (old wiring: CS2 on GP15) | yes, together with `kws.bin` |
| `fpga_lattice/pico_psram_host/prebuilt/kws_fpga_host.uf2` | `69174ceb1246c904da14dacaeeccb058` | r15 (matches the source): emulates the whole TT QSPI Pmod, with flash on CS0 (GP7), PSRAM A on CS1 (GP8) and PSRAM B on CS2 (GP9). CS2 moved from GP15 to GP9. | yes (2026-11-28): `d` passes, and each of the three CS carries the model stream with a bit-exact frame |
| `mcu_pico2w/kws_pico/kws_pico_prebuilt.uf2` | `3e508fa025d5319c20b65b98b3866a02` | Pico 2 W MCU-only emulation | yes: live detection with decent accuracy |

How to program them: drag a `.bin` onto the iCELink drive (iCESugar); for a `.uf2`, hold BOOTSEL while plugging in the Pico 2 W and drag the file onto the drive that appears.

Build folders and `.vscode/` are not tracked. In VS Code, use the Raspberry Pi Pico extension's **Import Project** on a Pico folder to regenerate them. See each folder's README for tools, wiring and bring-up.
