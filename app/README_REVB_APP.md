# tiny_kws Bench — rev B (4x2) update

`kws_live.html` is the built, self-contained page: open it in Chrome or Edge.
`app/kws_live_template.html` and `app/kws_model.js` are its sources.

- The header shows the chip revision: rev B, 4x2 tiles, ≈2.3 ms of NN time per 32 ms frame at 50 MHz.
- New **Chip setup** tab: register values (PDM_DIV, MEMCFG, BURST, BASE, THR, DETCFG, CTRL) for any clock,
  audio input (PDM / PCM / features) and model memory (PSRAM A/B, flash), with front-end and NN load, PSRAM tCEM,
  read-capture timing, ready-to-paste Python and raw SPI frames.
  - Rev B limits: 897 front-end clocks per sample (PCM ≥ 14.35 MHz; PDM 16 kHz needs ≥ 16.384 MHz, PDM_DIV ≥ 7);
    guaranteed 49.1 MHz at ss/100 °C/1.6 V (rev A: 945 clocks, 46.2 MHz).
  - CTRL always carries CLEAR (REVIEW_REVB finding 1: RUN 1→0→1 mid-frame needs CLEAR).
- `kws_model.js::chipSetup` and `scripts/kws_host.py::chip_setup` are the same calculator; `KWS.configure(fclk, …)`
  writes the registers on hardware.
- The network, front end and model blob are unchanged (rev B is bit-exact with rev A).

Tests: `node app/test_node.js` (bit-exact model, 372 frames, 0 mismatches), `node app/test_setup_node.js`
(reference operating points + JS vs Python on 1 206 cases, 0 differences), self-test tab in the page: PASS.

## Experiences panel

Four modes driven only by what leaves the silicon (DETECT edge + DET_CLASS nibble), using the THR/N detector from Settings.
Only the experience on screen reacts to detections.

| mode | words | silicon mapping |
|---|---|---|
| Voice maze | up/down/left/right run to the next junction, go repeats, no undoes | DETECT + DET_CLASS into any MCU (TT demo board RP2350) |
| Spoken lock | 4-word sequence, 5 s per word, 3 wrong words → 10 s lock-out | a few flip-flops on DET_CLASS clocked by DETECT; convenience only, not speaker verification |
| Hands-free speller | directions move a 6×5 letter grid, yes/go types, no deletes, stop twice clears | switch-scanning AAC pattern; text on the host |
| Inside the chip | live 48-unit GRU hidden state, FRAME/DETECT/DET_CLASS waveforms, audio-bits-in vs class-bits-out | matches what a logic analyser on uo[3], uo[2], uo[7:4] shows; hidden state is internal |

Checked in Chrome (desktop light, 390 px dark) with simulated detections through the page's frame queue: maze solved by
breadth-first commands, lock opens / rejects / locks out / times out, speller types the expected text, no script errors,
no horizontal scroll. Not yet tried with live speech.
