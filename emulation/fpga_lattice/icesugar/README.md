# icesugar: tiny_kws on the iCESugar v1.5 (iCE40UP5K-SG48)

See `../README.md` for tools, wiring and bring-up.

    build.bat                 (or: python build.py)  ->  build\kws.bin, log in build\build.log
    build.bat --prog E:       also copies kws.bin to the iCELink drive E:
    build.bat --tools "D:\x\oss-cad-suite"   if OSS CAD Suite is not in C:\Program Files or C:\

- `top_icesugar.v`: PLL 12 → 16.3125 MHz, uio tristate pads (pull-ups on CS0/1/2), power-on and host reset, RGB LED.
- `icesugar.pcf`: pin map (PMOD2 = uio in TT QSPI-Pmod order: pins 1-4 = uio0-3, pins 12-9 = uio4-7; PMOD3 = mic/SPI/status, PMOD1 = DET_CLASS, pin 21 = reset).
- `rtl/`: **unchanged** copies of the tiny_kws repo `src/*.v`. After an RTL change in the repo, copy the files again and check `rtl/MD5SUMS_from_tiny_kws_src.txt`. `KWS_SKY130_SCANFF` is not defined, so the ring registers are plain flip-flops.
- `prebuilt/kws.bin`: bitstream built from these sources (seed 4, Fmax 17.55 MHz, md5 1f05561f02a8162453686e32443f46e1).
- `sim/`: cocotb co-simulation with the Pico PSRAM emulator (`make -f cocotb.mk KWS_REPO=<tiny_kws repo>`; Linux or WSL).
