# Reference run C (rev B, 4x2, scan-flop rings)

Tiny Tapeout flow (tt-support-tools main of 2026-11-12, LibreLane 3.0.14, open_pdks 8afc8346) with this
repository's `info.yaml` and `src/config.json`, under the earlier top-module name. TT precheck passed (15/15).
Single run; the GitHub Action build of this repository is the
authority for tape-out.

- `metrics_4x2_scanff.csv`: final metrics (utilisation 73.9 %, routing/Magic DRC 0, LVS clean, antenna 0,
  hold +0.077 ns, setup ss −0.356 ns → 49.1 MHz worst corner)
- `metrics_4x2_scanff_first_run.csv`: first run before the final settings

The GDS and netlist of this run are not kept here; the GitHub Action builds them under the current name.
