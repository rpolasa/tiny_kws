# tiny_kws FPGA emulation: iCESugar v1.5 + Pico 2 W as PSRAM

The chip's RTL (`tt_um_rakesh_tiny_kws`, rev B, unchanged) runs on the iCE40UP5K. A Pico 2 W does two jobs:
- **PSRAM:** it emulates the QSPI PSRAM on the chip's real uio pins.
- **Board-B host:** it configures the chip over SPI, runs the golden-vector tests and prints results over USB.

The FPGA's uio connector uses the Tiny Tapeout QSPI-Pmod pin order, so a real TT QSPI Pmod can replace the Pico later.

```
 PDM mic ──► iCESugar v1.5: PLL 12 → 16.3125 MHz, tt_um_rakesh_tiny_kws
               PMOD3: PDM, SPI slave, DETECT, FRAME      PMOD2: uio[7:0] QSPI (CS2, SCK, SD0-3)
                         │ SPI (registers)                      │ QSPI
               Pico 2 W: core 0 = USB console + SPI host + tests │ core 1 + PIO0 + DMA = PSRAM B
```

| Folder | What |
|---|---|
| `icesugar/` | FPGA project: `top_icesugar.v` (PLL, uio tristates, reset, RGB LED), `icesugar.pcf`, `build.py` / `build.bat`, `rtl/` (unchanged copy of tiny_kws `src/*.v`, see `rtl/MD5SUMS_from_tiny_kws_src.txt`), `prebuilt/kws.bin` |
| `icesugar/sim/` | cocotb co-simulation: FPGA build + Pico PSRAM (PIO model running the real `.pio` opcodes) |
| `pico_psram_host/` | Pico 2 W VS Code project `kws_fpga_host` (PSRAM emulator + host), `prebuilt/kws_fpga_host.uf2` |

## 1. Tools

**Pico:** the VS Code Raspberry Pi Pico extension you already use. In the Pico panel choose **Import Project**, select `pico_psram_host`, board Pico 2 W, SDK 2.3.1. The extension creates `.vscode/` for it. Then **Compile** and **Run**.

**FPGA (open source; the iCE40UP5K needs no vendor tools):**
1. **Toolchain:** download **OSS CAD Suite** for Windows (x64) from YosysHQ's GitHub releases (`oss-cad-suite-windows-x64-<date>.exe`) and unpack it, e.g. to `C:\Program Files\oss-cad-suite` or `C:\oss-cad-suite`. It contains yosys, nextpnr-ice40, icepack, iverilog and gtkwave.
   - Building: double-click `icesugar\build.bat` or run it from any terminal. It finds OSS CAD Suite in those folders by itself, so there's no need to run `environment.bat` first. Elsewhere: `build.bat --tools "D:\path\oss-cad-suite"` (the folder that contains `bin\yosys.exe`).
   - The first lines printed (and `build\build.log`) show which tools were used; any tool error is printed with its last log lines.
   - Alternative with no installer: `pip install yowasp-yosys yowasp-nextpnr-ice40`, then `build.bat`.
2. **Editor (optional):** VS Code with *Verilog-HDL/SystemVerilog/Bluespec SystemVerilog* (`mshr-h.veriloghdl`). Set its linter to iverilog from OSS CAD Suite for syntax checks.
3. **Programming:** plug the iCESugar's USB-C (iCELink) into the PC. It appears as a USB drive named `iCELink`, plus a COM port. Drag `build\kws.bin` (or `prebuilt\kws.bin`) onto the drive, or run `build.bat --prog E:` where E: is that drive. The iCELink programs the FPGA's SPI flash, so the design persists across power cycles. Muse Lab's `icesprog` (in the iCESugar GitHub repo, `tools/`) is a command-line alternative.
4. **Co-simulation (optional, Linux/WSL):** `pip install cocotb==2.1.0 numpy scipy`, iverilog and gcc, then `cd icesugar/sim && make -f cocotb.mk KWS_REPO=<tiny_kws repo>`.

Lattice Radiant also supports the UP5K (free licence) but isn't needed. These scripts target the open-source flow.

## 2. Wiring

Short wires (< 10 cm) and a **common GND**. Power each board from its own USB and don't connect the 3V3 rails together.

| Signal | Chip pin | iCESugar header pin (FPGA pin) | Pico 2 W |
|---|---|---|---|
| CS0 flash | uio[0] | PMOD2 pin 1 (FPGA 46) | not connected |
| SD0 | uio[1] | PMOD2 pin 2 (FPGA 44) | GP10 |
| SD1 | uio[2] | PMOD2 pin 3 (FPGA 42) | GP11 |
| QSPI SCK | uio[3] | PMOD2 pin 4 (FPGA 37) | GP14 |
| SD2 | uio[4] | PMOD2 pin 12 (FPGA 45) | GP12 |
| SD3 | uio[5] | PMOD2 pin 11 (FPGA 43) | GP13 |
| CS1 RAM A | uio[6] | PMOD2 pin 10 (FPGA 38) | GP8 (monitor only: must stay high) |
| CS2 RAM B | uio[7] | PMOD2 pin 9 (FPGA 36) | GP15 |
| PDM_DAT | ui[0] | 34 (P3_1) | mic DAT |
| PDM_CLK | uo[0] | 31 (P3_2) | mic CLK |
| SPI SCK | ui[1] | 27 (P3_3) | GP18 |
| SPI MOSI | ui[2] | 25 (P3_4) | GP19 |
| SPI CSn | ui[3] | 23 (P3_9) | GP17 |
| SPI MISO | uo[1] | 26 (P3_10) | GP16 |
| DETECT | uo[2] | 28 (P3_11) | GP20 |
| FRAME | uo[3] | 32 (P3_12) | GP21 |
| DET_CLASS[3:0] | uo[7:4] | 2, 47, 48, 3 (P1_10, P1_9, P1_4, P1_3) | optional (LEDs or logic analyser) |
| rst_n | rst_n | 21 (PMOD4 pin 1 / DIP SW1) | GP22 |
| GND | | any PMOD GND | any GND |

- **Mic:** 3V and GND from a PMOD3 power pin; **SEL to GND**.
- **Header pin numbering (iCESugar v1.5 schematic):** each PMOD has signal pins 1-4 on one row and 12, 11, 10, 9 on the other, with **pin 12 opposite pin 1** and pin 9 opposite pin 4. Pins 5 and 8 are GND, 6 and 7 are 3V3. A standard 12-pin Pmod numbers that second row 7-10 (pin 7 opposite pin 1), so the TT QSPI Pmod's uio[4..7] land on iCESugar pins 12, 11, 10, 9; the pcf is set up that way.
- **Wire length:** keep the QSPI wires short (< 10-15 cm), keep SCK away from the SD lines in a ribbon, and run at least two GND wires between the boards (one next to the QSPI bundle).
- **DIP switch 1 must be OFF.** It shares FPGA pin 21 (reset) and would hold the chip in reset.
- **PMOD1:** use only the four DET_CLASS pins. Its other pins are shared with the iCELink USB and UART.

**RGB LED:**
- **Red:** chip not running (CTRL.RUN = 0, memory bus released). Red on after programming means the bitstream is alive.
- **Blue:** frame heartbeat.
- **Green:** detection.

## 3. Chip set-up used here
- **Model location:** model blob (10,412 B, md5 `e838370dc00e61f07f49e6c1cb506bd1`) at **PSRAM B 0x000000**, next to the chip's h′ scratch at 0x7FFF00. So **MEMCFG.cs = 2**: the Pico emulates one PSRAM on one chip-select. Model-in-RAM-B is a supported configuration (cocotb `test_03`); on the real board it is the same register field.
- **Clock:** 16.3125 MHz (PLL DIVR 0, DIVF 86, DIVQ 6).
- **PDM:** PDM_DIV 7 gives PDM_CLK 1.0195 MHz and fs 15.93 kHz (−0.44 %).
- **SPI host:** 1 MHz (limit f_clk/8 = 2.04 MHz).

| QSPI preset (`m` key) | MEMCFG | SCK | Read window (co-sim) | NN time / 32 ms frame (co-sim) |
|---|---|---|---|---|
| default | 0x2A (div 2, rd_dly 1) | 4.08 MHz | rd_dly 0–3 pass, 4 fails | 7.3 ms (23 %) |
| fast | 0x06 (div 1, rd_dly 0) | 8.16 MHz | rd_dly 0–1 pass, 2–3 fail | 3.8 ms (12 %) |

## 4. Bring-up
| Step | Do | Pass when |
|---|---|---|
| F0 | Program `kws.bin` with no wires attached | red LED on |
| F1 | Wire SPI, rst_n and GND; flash `kws_fpga_host.uf2`; open the serial monitor (or press `1`) | `F1 registers: ID 0x4b H 48 NCH 16 MEMCFG 0x49 ... PASS` |
| F2 | Wire the QSPI lines; press `d` (wiring diagnostic, runs automatically if F3 fails at boot) | first word `eb001248`, every FPGA SD line reported `ok` on its own GPIO, `hold-off on ... result bit-exact` in the A/B lines |
| F3 | `3`: golden features → NN, all 12 clips | `F3 ... 372 frames, 0 mismatches ... PASS (bit-exact)` |
| F3b | `w`: read-window sweep at SCK div 7, 4, 2, 1 | the pass/fail map matches section 3; on the bench the window may shift by one step, since Pico pad delays are estimates |
| F4 | `4`: golden PCM → front end → NN, 4 clips | `F4 ... features bit-exact, NN bit-exact -> PASS` |
| F5 | Mic on PMOD3; `l` for live | `>>> DETECT yes ...`, green LED; compare with the MCU-only kws_pico; tune `+`/`-` THR and `]`/`[` N |
| F6 | Leave live running for 1 h, then `s` | STATUS without OVERRUN/MODEL_ERR, data late "no", bad cmd 0 |

The Pico runs F1 and F3 automatically at boot and prints the result.

## 5. What has been verified (before hardware)
- **FPGA build:** yosys + nextpnr-ice40.
  - Logic cells: 3971/5280 (75 %); 1 DSP, PLL, 22 I/O.
  - Timing (after the 2026-11-23 PMOD2 pin fix): 5 of 6 seeds meet 16.31 MHz, best 17.55 MHz (seed 4, 7.6 % margin) on nextpnr's worst-case delay model; `build.py` tries 6 seeds and keeps the best one.
- **Co-simulation** (`icesugar/sim`): the FPGA top with the unchanged RTL runs against a Pico model that executes the pioasm opcodes of `qspi_psram.pio` at 150 MHz. The model includes the 2-cycle input synchroniser, DMA re-arm and core-1 latency, plus board delays of 6 ns (FPGA → Pico) and 8 ns (Pico → FPGA).
  - Every FEAT_HOST frame is bit-exact with the golden model at div 2 / rd 1 and div 1 / rd 0: 329 burst reads plus 48 h′ writes per frame, the mid-burst abort at the end of each model pass recovered, no bus contention, no late data.
  - It still passes at div 1 with 2.5× slower DMA and 5× slower core 1.
  - The window map is in section 3. Out-of-window settings fail cleanly (STATUS model error), and the next setting recovers.
- **CS glitch immunity (added 2026-11-23 after the first bench run):** on the bench a short spike on the CS2 wire mid-transaction re-armed the Pico's data state machine, so it never drove the read data (MODEL_ERR at every speed). The CS watchers now run on PIO1 and act only when CS stays high for ~100 ns, and qspi_rx no longer waits on the CS pin. Co-sim with a 33 ns CS spike injected into every transaction (658-1412 spikes per run): bit-exact at div 1, 2 and 7, 0 bad commands, 0 late data. The `d` diagnostic reports raw CS rises vs transactions.
- **PULL fence fix (2026-11-23, found on the bench):** with autopull enabled, PULL is a no-op while the OSR is full (RP2040/RP2350 datasheet, PULL), so after `mov osr, null` the first data word was never loaded and the Pico drove 0000 (MODEL_ERR at every speed). The first co-sim model lacked this rule; it now implements it and reproduces the bench failure exactly. qspi_tx now runs without autopull and pulls each word explicitly; with that, the whole co-sim regression above passes again.
- **SCK glitch hold-off (r13, 2026-11-23, found on the bench):** with F1 and the wiring check passing, every frame still came out wrong (h′ 37 of 48 bytes off). The r12 full-frame bus check showed reads with more SCK rising edges than the chip sends, and data past the end of a burst stuck at one nibble (`88`, `11`, `aa` …). When the Pico switches its 4 SD outputs just after a falling edge, the switching couples into the SCK wire (crosstalk / ground bounce through the jumper wires) and the Pico sees a short false SCK pulse. The data state machine took each pulse for a clock edge, drove the next nibble one edge early and ran out of data before the end of the burst. The chip therefore captured shifted weights from that point onwards, while the bus as the Pico saw it still looked right.
  - **Fix:** every PIO wait on an SCK edge (and the instruction that switches SD) now has a hold-off delay, so the state machines ignore SCK for H sys cycles after an edge. H is set to 7·div − 3 cycles at 150 MHz (div 1: 4, div 2: 11, div ≥ 5: 31) whenever the firmware configures the chip.
  - `g` switches the hold-off off and on. `d` runs the same frame both ways (A/B) and counts the extra SCK edges the Pico saw.
  - **Verification:** `sim/glitch_sim.py` drives the real pioasm opcodes with kws_qspi waveforms and injects SCK spikes whenever the Pico raises ≥ 2–4 SD lines. Without the hold-off this reproduces the bench signature: 10–90 % of nibbles wrong, extra sniffer edges and held nibbles. With the hold-off, 0 wrong at div 2–7, and at div 1 with mild spikes. Full RTL co-sim (`PICO_SCK_GLITCH=4:2:2`, about 870 spikes per 2 frames): with the hold-off off, every frame is wrong with STATUS clean, and at div 2 it gives `1 127/127` on every frame, the same as F3 on the bench. With the hold-off on (31 at div 7, 11 at div 2), every frame is bit-exact, with no late data and no bad commands.
  - **Bench result (r13, 2026-11-23):** F3 passes, 372 of 372 frames bit-exact at 4.08 MHz with a hold-off of 11. In the `d` A/B run, the Pico saw about 550 extra SCK edges per frame, in 281 of 377 transactions. With the hold-off off the result was wrong (`7 127/42`, 37 of 48 h′ bytes wrong); with it on (31) the result was bit-exact.
  - **Bench read-window sweep (`w`, r13):**
    - div 7 (1.17 MHz) and div 4 (2.04 MHz): rd_dly 0–7 all pass.
    - div 2 (4.08 MHz): rd_dly 0–3 pass, 4–7 fail.
    - div 1 (8.16 MHz): rd_dly 0–1 pass, 2–7 fail.
    - At div 2 and div 1 this is the same window the co-sim predicted in section 3, so the presets (div 2 / rd 1 and div 1 / rd 0) both sit inside it.
  - **Bench F4 and F5 (r13):** F4 passed: 124 frames, with features and NN both bit-exact. In F5 (live mic), detection accuracy was the same as the MCU-only kws_pico.
  - Wiring helps as well: use short wires, run a GND wire next to SCK, and connect several GND pins between the boards.
- **Remaining hardware assumption:** a DMA write to `SMx_INSTR` preempts a state machine stalled on `wait`. The PIO register description says a write "executes an instruction immediately (including jumps) and then resume[s] execution". F2 and F3 confirm it on the bench: every transaction after the first depends on it.

## 6. If something fails
| Symptom | Check |
|---|---|
| F1 ID reads 0x00 or 0xFF | SPI wiring (MISO ↔ GP16), GND, DIP SW1 off, red LED on |
| F3 MODEL_ERR, PSRAM reads = 0 | QSPI wiring, CS2 on GP15, SCK on GP14 |
| F3 wrong results, `d` reports extra SCK edges | SCK crosstalk: keep the hold-off on (`g`), shorten wires, add GND wires (one next to SCK) |
| F3 MODEL_ERR, reads > 0 | press `d`: it names any SD line that is open, swapped or shorted. If `d` passes, run `w`: ok only at slow speeds means signal integrity (shorter wires, more GND) |
| data late "YES" | core 1 is too slow: build with `-DSYS_KHZ=200000` (Pico at 200 MHz) or use the div 2 preset |
| F5 no detections | mic SEL to GND, PDM_CLK at 1.02 MHz on P3_2 (logic analyser), `e` for the other PDM edge |
