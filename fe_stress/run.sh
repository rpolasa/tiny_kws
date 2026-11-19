#!/usr/bin/env bash
# Front-end equivalence: every filter output y (all channels, all samples) and every feature of
# the RTL against model/kws_fe.c, for hop 256 and 512.  Run from this directory inside the repo
# (run from fe_stress/ so that ../model and ../src exist).
set -e
python3 gen_stim.py
gcc -O2 -I../model -o ref ref.c
iverilog -g2005 -o fe.vvp tb_fe.v ../src/kws_frontend.v ../src/kws_coef_rom.v ../src/kws_sreg.v
for hop in 256 512; do
  ./ref stim.txt $hop
  if [ $hop = 512 ]; then vvp -n fe.vvp +HOP512 > /dev/null; else vvp -n fe.vvp > /dev/null; fi
  echo "hop $hop: y mismatches $(diff ref_y.txt dut_y.txt | wc -l) of $(wc -l < ref_y.txt), feature mismatches $(diff ref_f.txt dut_f.txt | wc -l) of $(wc -l < ref_f.txt)"
done
