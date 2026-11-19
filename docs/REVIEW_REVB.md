# tiny_kws — design review and tile optimisation (rev B)

2026-11-12 · reviewed: repo `tt_kws` at head `45ba20a` (RTL, golden models, tests, flow config, docs, sign-off views)
Work was done on copies in a cloud workspace. Nothing in `src/`, `test/`, `model/` or `docs/` was changed;
everything new is in `opt_revB/` (an overlay for the repo root) and this file.

## 1. Result

| | Signed-off (rev A) | Rev B, plain flops | Rev B, scan-FF rings |
|---|---|---|---|
| Tiles | 6x2 (12) | 4x2 (8), did not close | **4x2 (8)** |
| Flip-flops | 2 380 | 2 163 | 2 163 (1 376 as `sdfxtp_1`) |
| Yosys area, flat (µm², same script for all) | 98 072 | 89 887 (−8.3 %) | 83 464 (−14.9 %) |
| Placed std-cell area / utilisation | 132 971 / 58.9 % | 118.9 k µm² / 79.7 % (before routing) | **110 295 µm² / 73.9 %** |
| Detailed routing / Magic DRC / LVS / antenna | 0 / 0 / match / 0 | not closed (see §5) | **0 / 0 / match / 0** |
| Hold, worst of 9 corners | +0.097 ns | — | +0.077 ns |
| Setup at 50 MHz: tt worst / ss worst | +9.62 / −1.64 ns | — | +8.97 / −0.36 ns |
| Guaranteed f_max (ss, 100 °C, 1.6 V) | 46.2 MHz | — | **49.1 MHz** |
| Power (tt, 50 MHz, vectorless) | 10.4 mW | — | 9.4 mW |
| Model, blob, registers, pinout, accuracy | — | unchanged, bit-exact | unchanged, bit-exact |
| Front-end clocks per sample | 945 | 881 (897 on the last sample of a hop) | same |

The 4x2 columns are real runs of the Tiny Tapeout flow (tt-support-tools main of 2026-11-12, LibreLane 3.0.14
docker image, open_pdks `8afc8346`) on a 2-vCPU machine, not estimates. They are single runs with the repo's
`config.json` plus the changes listed in §5.

**Recommendation.** Rev B with scan-FF rings fits 4x2 and passed everything I could run on it: 73.9 % utilisation,
routing and Magic DRC 0, LVS clean, antenna 0, hold clean at all nine corners, gate-level simulation 3/3 and
TT precheck 15/15. Its slow-corner speed is better than the signed-off 6x2 (49.1 against 46.2 MHz). Rev B with
plain flops does not close routing in 4x2, so the scan-FF rings are required for the smaller size.

The 4x2 is therefore a real option, 4 tiles smaller. It is one run of each step on one machine, at 74 %
utilisation, with hand-instantiated library cells and one extra flow setting. Before it replaces the signed-off
6x2 it needs your container cross-check and the TT GitHub Action build (§8); if either disagrees, the 6x2 as
signed off remains the submission. I would not move the 6x2 itself to rev B: it gains nothing in tiles and
re-opens a finished sign-off.

## 2. Where the area is

Per-register inventory of the sign-off netlist (`signoff/views/*.nl.v`) and LibreLane `metrics.csv`:

| Item | Flops | Share of placed area |
|---|---|---|
| Front-end state ring, 16 × 54 bit | 864 | |
| Hidden-state ring, 48 B | 384 | |
| Feature ring, 16 B | 128 | |
| **Ring storage (flop + enable mux, 32.5 µm² per bit)** | **1 376** | **≈ 45 k µm², about 39 % of the logic** |
| Everything else | 1 004 | |
| Sequential cells / combinational cells | 2 380 | 50.6 k / 65.5 k µm² |
| Timing-repair buffers / clock tree / taps | | 7.3 k / 4.7 k / 4.1 k µm² |

Two consequences:

1. The design is storage-bound. The rings are the algorithm's minimum state (two filter states and one envelope
   per channel, the hidden vector, the feature vector), so the bit count cannot fall without changing the model.
   What can fall is the cost per bit and the registers around the rings.
2. In a 4x2 (core 149.2 k µm²) the signed-off netlist would sit at 87.7 %, which matches the "87 %, detailed
   placement fails" entry in ARCHITECTURE §8. The next size below 6x2 is 4x2; there is no 10-tile shape.

## 3. Rev B changes

All are exact: same golden models, same blob, same register map, same pins.

| # | Change | Flops | Why it is exact |
|---|---|---|---|
| F1 | Front-end multiply-accumulate rewritten. Old: three 18-cycle shift-add products into a 38-bit accumulator, with a 38-bit operand shifter and an 18-bit coefficient shifter. New: a 22-bit right-shifting accumulator, one product term per clock, operand taken directly from the x registers / ring head and the coefficient bit directly from the ROM | −72 | `floor((floor(p/2)+t)/2) = floor((p+2t)/4)`: bits shifted out can never change the result again. The 2^15 rounding constant is the carry-in at bit 15; bit 16 is added unshifted, bit 17 (the sign bit) subtracted doubled. Worst-case magnitude 1 048 569 < 2^21, so 22 bits never overflow. Header comment of `kws_frontend.v` has the derivation |
| F2 | Write-back staging registers `wb_entry` (54) and `e_r` (20) removed. The ring is written directly; features are emitted in one extra turn of the ring after the last sample of a hop (16 clk), which also clears `e` | −75 | Same values in the same order; the NN sees 16 consecutive `feat_valid` cycles instead of 16 spaced ones |
| F3 | `cic_out` register removed; the comb result register `cv` is stable until the next sample and is read directly | −17 | `cv` changes only once per 64 PDM bits |
| F4 | `|y| >> 4` computed as `y<0 ? ~(y>>4) + (y[3:0]==0) : y>>4`, with the +1 as carry-in of the envelope adder | 0 | Identity for two's complement; removes a 17-bit negation chain. This was the only path that failed timing at the slow corner in the first 4x2 run |
| N1 | Second accumulator `acc2` removed. In the n row the x part is finished first, its shifted value is parked in a 9-bit register, and the accumulator is reloaded with `Bnh`, which is still in the bias shift register | −16 | `n = clamp(ax + (r·qh >>> 7))` with `|r·qh >>> 7| ≤ 128`, so clamping `ax` to [−256, 255] cannot change `n`. The next weight byte is at least 4 clk away, the reload takes 2. The 26-bit shifter-input mux (old critical path) disappears |
| N2 | `m_addr` register replaced by a 3-way mux (base / scratch / base + FC_OFF); `w_addr` derived from the scratch write index, which now increments on the write acknowledge | −21 | All three sources are static during a frame |
| T1 | `pcm_hi` and `hf_data` copies removed; the SPI slave's `wdata` register still holds the byte when the strobe arrives | −16 | Next SPI byte is ≥ 64 clk later |
| R1 | The three rings moved into one small module, `kws_sreg` (register with a common enable). Default: ordinary enable flops. With `KWS_SKY130_SCANFF` defined: one `sky130_fd_sc_hd__sdfxtp_1` per bit, whose scan mux is used as the enable mux (D = hold, SCD = new data, SCE = enable) | 0 | 26.3 µm² per bit instead of 21.3 + 11.3. `sdfxtp_1` is in the PDK's `no_synth.cells` (Yosys will not pick it by itself) but not in `drc_exclude.cells`, so it is allowed in place and route |

Net: 2 380 → 2 163 flip-flops. Front-end logic outside the ring 22.5 k → 16.9 k µm² (Yosys), NN logic outside
the rings 22.3 k → 20.0 k µm².

Side effects that are improvements: the front end needs 881 clk per sample (897 on the last sample of a hop)
instead of 945, so the real-time limit moves from 15.2 to 14.4 MHz at 16 kHz; and the slow-corner critical path
of rev A (NN shifter → 27-bit add → saturate) is gone.

## 4. Verification done on rev B

| Check | Result |
|---|---|
| Verilator 5.020 `-Wall` | 0 warnings |
| Repo regression `test/test.py`, cocotb 2.1 + Icarus 12, RTL (plain flops) | **6/6 pass**, bit-exact (same 297 ms of simulated time as rev A) |
| TT precheck (tt-support-tools `precheck.py`, PDK `8afc8346`) on the final 4x2 submission | **pass, 15/15** (Magic DRC, KLayout FEOL/BEOL/offgrid/zero-area/pin-label, pin, boundary, power-pin, layer, cell-name, urpm/nwell, analog-pin, Verilog syntax) |
| Front-end stress vs `model/kws_fe.c` (`opt_revB/fe_stress`): 29 184 samples of full-scale noise, full-scale squares, DC extremes, ±3 LSB noise; hop 256 and 512 | 466 944 filter outputs and all features identical in both runs; 1 607 outputs hit the ±saturation rails, features reach 159 |
| NN stress (`test_stress.py`): three random models with full-range int8 weights, biases up to ±2^23, C = 16, random features 0…255 | pass, bit-exact every frame; the parked x part was below / inside / above the 9-bit clamp 445 / 476 / 519 times |
| Gate-level simulation of the routed 4x2 netlist with scan flops (first run, before F4), sky130 functional models | **3/3 pass** (test_01, test_02, test_05; 03/04/06 are skipped at gate level as in the repo) |
| Gate-level simulation of the final routed 4x2 netlist | **3/3 pass** (test_01, test_02, test_05), 1 376 `sdfxtp_1` cells in the netlist |

Not done here: the RTL regression with `KWS_SKY130_SCANFF` and a behavioural stand-in for the cell
(`SCANFF=yes make`) was stopped after test_01 passed, because 1 376 single-bit instances make Icarus roughly
ten times slower. The gate-level runs cover that configuration with the real cell models.

## 5. The 4x2 runs

Changes against the repo's flow input, all in `info.yaml` and `src/config.json`:

* `tiles: "4x2"`, `kws_sreg.v` added to `source_files`
* `PL_TARGET_DENSITY_PCT`: 60 → 80 (must be above the utilisation)
* scan-FF variant only: `"VERILOG_DEFINES": ["KWS_SKY130_SCANFF"]`
* final run only: `GRT_RESIZER_HOLD_SLACK_MARGIN` 0.05 → 0.1, and `"EXTRA_EXCLUDED_CELLS": ["sky130_fd_sc_hd__a2111oi_*"]`

| Run | RTL | Config beyond the list above | Outcome |
|---|---|---|---|
| A | rev B without F4, scan-FF rings | — | Complete in 57 min. 74.2 % utilisation, routing/Magic DRC 0, LVS match, antenna 0, hold +0.042 ns, setup tt +8.63 / ss −2.74 ns (44.0 MHz), 9.4 mW. Gate-level sim 3/3 |
| B | rev B without F4, plain flops | density 86 | 79.7 % utilisation. Detailed routing reached 0 violations after 22 iterations, but each antenna repair (38, then 11 diodes) forced a re-route; the third pass was stuck at 67 violations when I stopped the run after 2.8 h in detailed routing. Not a usable configuration |
| C | **rev B final (with F4), scan-FF rings** | hold margin 0.1, `a2111oi` excluded | **Complete. 73.9 % utilisation, routing/Magic DRC 0, LVS match, antenna 0, hold +0.077 ns, setup tt +8.97 / ss −0.36 ns (49.1 MHz), 9.4 mW, precheck 15/15.** Routing converged in 10 iterations, then 7 more after 20 antenna diodes |

Notes on these runs:

* **Run A's only failing path group** at the slow corner was the front-end write-back (accumulator → saturate →
  negate → envelope add → ring), 20 endpoints at −2.74 ns; every other group had ≥ +0.50 ns. F4 removes the
  negation chain. In run C the slow corner is +0.07 ns at nominal extraction and −0.36 ns at the worst (max)
  extraction, against −1.40 / −1.64 ns for the signed-off 6x2.
* **OpenROAD resizer runaway.** Two attempts at run C died in `OpenROAD.RepairDesignPostGPL`: `repair_design`
  grew to 6.7 GB and was killed by the kernel, reproducibly. With resizer debug output the loop is on one
  fan-out-of-2 net driven by a `sky130_fd_sc_hd__a2111oi_2`, with both loads within 6 µm. The cell cannot meet
  the slew target that `DESIGN_REPAIR_MAX_SLEW_PCT = 40` sets, and this OpenROAD build (2026-02-17) keeps
  buffering. Run B's netlist contains three such cells and passed, so it depends on the instance. Whether ABC
  emits this cell changes with any small RTL edit, so the same failure can appear in the 6x2 flow after an
  unrelated change. `"EXTRA_EXCLUDED_CELLS": ["sky130_fd_sc_hd__a2111oi_*"]` removes the cell from synthesis
  and place-and-route and is in `opt_revB/flow_4x2/src/config.json`. On a machine with more memory the symptom
  would be a step that never ends rather than a crash.
* **Hold.** Run A's +0.042 ns is clean (the 0.10 ns hold uncertainty is already subtracted), but thinner than
  the 6x2. Raising `GRT_RESIZER_HOLD_SLACK_MARGIN` to 0.1 brought run C to +0.077 ns without a visible cost
  in utilisation.
* **Max-slew flags** at the slow corner are 737 (6x2: 422), against the 0.75 ns design target; max-cap is 0.
* The 1 001 `EST-0026` and 10 `DRT-0349` warnings also appear in the signed-off flow log.

## 6. Measured and not adopted

| Idea | Gain | Why not |
|---|---|---|
| Shift amounts limited to 4 bits | −0.2 k µm² | Narrows the blob format for 0.2 % |
| BASE register reduced to its top byte (64 kB model slots) | −0.5 k µm² | Changes the register map and test_01 for 0.6 % |
| QSPI burst counters made combinational | −0.2 k µm² | Touches the block with the tightest external timing for 0.2 % |
| Latches or clock gating for the rings | up to −15 k µm² | Against IMP-2; reintroduces the hold problem recorded for the free-running ring |
| Front-end state in PSRAM | −27 k µm² | 216 B of traffic per 62.5 µs sample: about 73 % bus occupancy at SCK 12.5 MHz, incompatible with the weight stream and with the 16–25 MHz operating points |
| Narrower filter state or envelope, fewer channels, smaller H | ≈ 0.5 k µm² per ring bit-column | Each needs retraining and costs accuracy that is already 0.3 pp short of APP-2; about 2 points of 4x2 utilisation for 6 bit-columns |

## 7. Review findings

Ordered by importance. None blocks the signed-off 6x2.

1. **RUN cleared in mid-frame leaves the NN engine busy for ever** (reproduced on rev A with
   `opt_revB/test/test_runtoggle.py`). `~run` stops the QSPI stream, but `kws_nn` stays in the weight state
   waiting for a byte. After RUN = 1 again, STATUS stays 0x03, no frame completes and the front end reports
   overruns until CTRL.CLEAR is written; CLEAR recovers it. Either document "write CLEAR whenever RUN is set"
   in ARCHITECTURE §7 and `docs/info.md`, or let the engine return to idle when `run` falls. Rev B behaves the
   same as rev A here.
2. **MEM-2 is worded more strongly than the hardware guarantees.** The burst clamp is in bytes, not time: 32 B is
   83 SCK periods, which is under 8 µs only for SCK ≥ 10.4 MHz. The reset values (BURST 3, divider 2) therefore
   violate tCEM at any clock below 41.5 MHz until the host lowers BURST. SIGNOFF §4 has the right settings; the
   requirement and the register description should say that BURST must follow the clock, and the FPGA plan at
   16.384 MHz depends on it.
3. **FE-4 is not met as written.** REQUIREMENTS says "< 1.1 LSB(log) rms", while ARCHITECTURE §3, VERIFICATION and
   SIGNOFF §3 report 1.2 LSB. It is also missing from the compliance table in SIGNOFF §1.
4. **Reset default is hop 256, the deployed model is trained for hop 512.** `docs/info.md` says to write
   CTRL = 0x09, but ARCHITECTURE §7 and SIGNOFF §4 do not; a bring-up that only sets RUN runs the model at the
   wrong frame length. Either make HOP512 the reset value in a future revision or state it next to the register.
5. **Accumulator bound.** NN-5 and ARCHITECTURE §4 say "|bias| < 2^23 plus 64 × 2^14 < 2^25". 64 × 2^14 is 2^20, the
   sum is below 2^24, and 25 bits would be enough. The conclusion (no overflow in 26 bits) is right, the
   arithmetic in the sentence is not. `gru_int.py` says the accumulators saturate; the RTL wraps, which is
   equivalent only because the bound holds.
6. **"1 hold buffer after CTS" in SIGNOFF §2** is the count of one flow step. The netlist contains 395
   `clkdlybuf4s25` cells and 1 076 timing-repair buffers (7.3 k µm²) in total.
7. **PDM full scale.** A PDM input stuck at 1 for 192 bits gives +2^18 in the 19-bit CIC, which wraps to −2^18
   (output −32768 instead of +32767). A real microphone never reaches 100 % density and the band-pass removes
   DC, so there is no functional effect; the comment "no saturation needed" should say why.
8. Stale text: `kws_frontend.v` header says "N (12)" channels and "CIC4"; `kws_fe.c` header says "CIC4";
   VERIFICATION says "12-ch biquads"; `config.json` and SIGNOFF §2 speak of "~2 900 shift-ring flip-flops" (the
   design has 2 380 flops, 1 376 of them in rings); REQUIREMENTS IMP-1 and `harden_docker.sh` name the ttsky26d
   flow for a TTSKY26e submission.

Architecture: streaming the weights in consumption order, the h′ scratch in PSRAM B and the single-multiplier
datapath are sound, and I found no functional error in the RTL. The bit-exact golden-model method is the
strongest part of the project and is what made rev B checkable in a day.

## 8. Using the overlay

```
opt_revB/src/            all seven RTL files (project.v, kws_frontend.v, kws_nn.v changed; kws_sreg.v new)
opt_revB/test/           Makefile.revB (replaces test/Makefile: kws_sreg.v, SCANFF=yes, test-module override), sky130_scanff_stub.v,
                         test_stress.py, test_runtoggle.py
opt_revB/stress/         three random stress blobs for test_stress.py
opt_revB/fe_stress/      front-end equivalence bench (run.sh)
opt_revB/flow_4x2/       info.yaml and src/config.json exactly as used for the final 4x2 run (run C), its metrics.csv,
                         GDS, powered netlist and precheck log; metrics of run A
```

On a branch: copy `opt_revB/src`, `opt_revB/test` and `opt_revB/stress` over the repo root, rename
`test/Makefile.revB` to `test/Makefile`, add `kws_sreg.v`
to `SRCS` in `scripts/iic_verify.sh` and to `build/*.ys`, take `info.yaml` and `src/config.json` from
`opt_revB/flow_4x2`, then run your normal sequence (lint, RTL, harden, GL, DRC, LVS, STA) in the container
and the TT GitHub Action.

## 9. Not verified here

* The precheck here used PDK `8afc8346`; your recorded precheck used `0536d02d`. The TT GitHub Action is the authority.
* The IIC-OSIC-TOOLS cross-check (dev27 left 2–4 antenna nets on rev A; a denser 4x2 may leave more there).
* Lint under Verilator 5.038 (5.020 used here) and Yosys 0.62 warnings beyond what the flow's checkers report.
* FPGA build with the new source list.
* Each 4x2 result is one run; I did not test run-to-run or seed variation.
