#!/usr/bin/env python3
"""Build the tiny_kws FPGA bitstream for the iCESugar v1.5 (iCE40UP5K-SG48).

    python build.py              synth + place&route (best of several seeds) + pack -> build/kws.bin
    python build.py --seeds 1    single seed (faster)
    python build.py --prog E:    also copy kws.bin to the iCELink drive (drag-and-drop programming)

    python build.py --tools "D:\\tools\\oss-cad-suite"    OSS CAD Suite somewhere else

Tools: yosys, nextpnr-ice40, icepack from OSS CAD Suite. Found automatically in --tools / %OSS_CAD_SUITE%,
in an oss-cad-suite folder beside this project or any parent folder (e.g. D:\\MY\\asic\\oss-cad-suite),
on PATH, or in C:\\Program Files\\oss-cad-suite, C:\\oss-cad-suite, %LOCALAPPDATA%\\oss-cad-suite, %USERPROFILE%\\oss-cad-suite
(no need to run environment.bat first). Fallback: pip install yowasp-yosys yowasp-nextpnr-ice40.
Everything is logged to build/build.log.
"""
import argparse, os, re, shutil, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
RTL = ["kws_sreg.v", "kws_spi_slave.v", "kws_coef_rom.v", "kws_frontend.v", "kws_qspi.v", "kws_nn.v", "project.v"]
FREQ = "16.3125"


LOG = None


def say(msg):
    print(msg, flush=True)
    if LOG:
        with open(LOG, "a") as fh:
            fh.write(msg + "\n")


def fail(msg):
    say("ERROR: " + msg)
    sys.exit(1)


def find_oss(user_dir):
    """locate an OSS CAD Suite root (folder containing bin/yosys[.exe]) and put bin + lib on PATH"""
    cands = [user_dir, os.environ.get("OSS_CAD_SUITE"), os.environ.get("YOSYSHQ_ROOT")]
    d = HERE                                                       # next to any parent folder, e.g. D:\\MY\\asic
    while True:
        cands.append(os.path.join(d, "oss-cad-suite"))
        up = os.path.dirname(d)
        if up == d:
            break
        d = up
    for base in (os.environ.get("ProgramFiles"), "C:\\Program Files", "C:\\", os.environ.get("LOCALAPPDATA"),
                 os.environ.get("USERPROFILE"), os.path.expanduser("~")):
        if base:
            cands.append(os.path.join(base, "oss-cad-suite"))
    for c in cands:
        if not c:
            continue
        for root in (c, os.path.join(c, "oss-cad-suite")):          # also a doubly-nested unpack
            for exe in ("yosys.exe", "yosys"):
                if os.path.isfile(os.path.join(root, "bin", exe)):
                    os.environ["PATH"] = os.pathsep.join([os.path.join(root, "bin"), os.path.join(root, "lib"),
                                                          os.environ.get("PATH", "")])
                    return root
    return None


def tool(name):
    p = shutil.which(name)
    if p:
        return [p]
    p = shutil.which("yowasp-" + name)
    if p:
        return [p]
    fail(f"'{name}' not found. Install OSS CAD Suite (e.g. C:\\Program Files\\oss-cad-suite) and/or pass "
         f"--tools <folder>, or: pip install yowasp-yosys yowasp-nextpnr-ice40")


def run(cmd, log):
    with open(log, "w") as fh:
        try:
            r = subprocess.run(cmd, cwd=HERE, stdout=fh, stderr=subprocess.STDOUT)
        except OSError as e:
            fh.write(f"cannot start {cmd[0]}: {e}\n")
            return 255
    if r.returncode:
        tail = open(log, errors="replace").read().splitlines()[-15:]
        say(f"{os.path.basename(cmd[0])} exited with {r.returncode}; last lines of {os.path.relpath(log, HERE)}:")
        for line in tail:
            say("    " + line)
    return r.returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=6, help="place&route seeds to try (best Fmax kept)")
    ap.add_argument("--prog", help="iCELink drive to copy kws.bin to, e.g. E: or /media/$USER/iCELink")
    ap.add_argument("--tools", help="OSS CAD Suite folder (the one that contains bin\\yosys.exe)")
    a = ap.parse_args()
    global LOG
    os.makedirs(os.path.join(HERE, "build"), exist_ok=True)
    LOG = os.path.join(HERE, "build", "build.log")
    open(LOG, "w").close()
    t0 = time.time()
    root = find_oss(a.tools)
    say(f"OSS CAD Suite: {root or 'not found in the usual places - using PATH / yowasp'}")
    tools = {n: tool(n) for n in ("yosys", "nextpnr-ice40", "icepack")}
    for n, c in tools.items():
        say(f"  {n:14s} {c[0]}")
    src = " ".join(f"rtl/{f}" for f in RTL) + " top_icesugar.v"     # relative paths (yowasp sandbox)
    say("synthesis (yosys synth_ice40) ...")
    if run(tools["yosys"] + ["-q", "-l", "build/synth.log", "-p",
                            f"read_verilog {src}; synth_ice40 -dsp -top top_icesugar -json build/kws.json"],
           os.path.join(HERE, "build", "yosys.out")):
        fail("synthesis failed - see build/synth.log and build/yosys.out")
    best = None
    for seed in range(1, a.seeds + 1):
        log = f"build/pnr_seed{seed}.log"
        rc = run(tools["nextpnr-ice40"] + ["--up5k", "--package", "sg48", "--json", "build/kws.json",
                                          "--pcf", "icesugar.pcf", "--freq", FREQ, "--seed", str(seed),
                                          "--asc", f"build/kws_seed{seed}.asc", "-l", log, "-q"],
                 os.path.join(HERE, "build", f"pnr_seed{seed}.out"))
        txt = open(os.path.join(HERE, log)).read() if os.path.exists(os.path.join(HERE, log)) else ""
        f = re.findall(r"Max frequency for clock 'clk': ([\d.]+) MHz", txt)
        fmax = float(f[-1]) if f else 0.0
        ok = rc == 0 and fmax >= float(FREQ)
        say(f"  seed {seed}: Fmax {fmax:.2f} MHz {'PASS' if ok else 'FAIL'}")
        if ok and (best is None or fmax > best[1]):
            best = (seed, fmax, txt)
    if best is None:
        fail(f"no seed met {FREQ} MHz - see build/pnr_seed*.log; try more seeds (--seeds 12)")
    seed, fmax, txt = best
    lc = re.findall(r"ICESTORM_LC:\s+(\d+)/\s*(\d+)", txt)[-1]
    shutil.copy(os.path.join(HERE, f"build/kws_seed{seed}.asc"), os.path.join(HERE, "build/kws.asc"))
    if run(tools["icepack"] + ["build/kws.asc", "build/kws.bin"], os.path.join(HERE, "build", "icepack.out")):
        fail("icepack failed")
    say(f"build/kws.bin: seed {seed}, Fmax {fmax:.2f} MHz (target {FREQ}), {lc[0]}/{lc[1]} LCs, "
          f"{time.time() - t0:.0f} s")
    if a.prog:
        drive = a.prog + "\\" if a.prog.endswith(":") else a.prog     # "E:" -> "E:\\"
        shutil.copy(os.path.join(HERE, "build", "kws.bin"), os.path.join(drive, "kws.bin"))
        say(f"copied to {a.prog} - iCELink programs the FPGA within a few seconds")


if __name__ == "__main__":
    main()
