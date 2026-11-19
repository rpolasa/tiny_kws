// Chip set-up calculator: (1) reference operating points from SIGNOFF §4 and the FPGA plan,
// (2) app/kws_model.js::chipSetup identical to scripts/kws_host.py::chip_setup over a sweep.   node app/test_setup_node.js
const fs = require("fs"), path = require("path"), cp = require("child_process");
const m = { exports: {} }; new Function("module", fs.readFileSync(path.join(__dirname, "kws_model.js"), "utf8"))(m); const K = m.exports;
let bad = 0; const chk = (c, msg) => { if (!c) { bad++; console.log("FAIL", msg); } };
const R = (r, name) => r.regs.find(x => x.name === name).v;
// reference points: [MHz, PDM_DIV, SCK div, rd_dly, BURST]
for (const [mhz, pd, dv, rd, bu] of [[50, 23, 2, 2, 3], [40, 19, 2, 2, 2], [25, 11, 2, 2, 1], [16.384, 7, 1, 0, 2]]) {
  const r = K.chipSetup({ fclk: mhz * 1e6, src: "pdm" });
  chk(r.ok && R(r, "PDM_DIV") === pd && r.d.div === dv && r.d.rdDly === rd && R(r, "BURST") === bu, `${mhz} MHz: got PDM_DIV ${R(r, "PDM_DIV")} div ${r.d.div} rd ${r.d.rdDly} BURST ${R(r, "BURST")}`);
  chk(r.d.burstBytes === 4 << bu && (r.d.csLowNs <= 8000), `${mhz} MHz tCEM`);
}
chk(Math.abs(K.chipSetup({ fclk: 50e6 }).d.nnMs - 2.3) < 0.1, "NN 2.3 ms at 50 MHz (NN-3 measurement)");
chk(Math.abs(K.chipSetup({ fclk: 25e6 }).d.nnMs - 8) < 0.3, "NN about 8 ms at 25 MHz (SIGNOFF §4)");
chk(R(K.chipSetup({ fclk: 50e6 }), "MEMCFG") === 0x49, "MEMCFG reset value at 50 MHz");
chk(R(K.chipSetup({ fclk: 50e6 }), "CTRL") === (1 | 8 | 16), "CTRL = RUN|HOP512|CLEAR");
chk(K.chipSetup({ fclk: 14.36e6, src: "pcm" }).ok && !K.chipSetup({ fclk: 14.34e6, src: "pcm" }).ok, "rev B PCM limit 14.352 MHz");
chk(!K.chipSetup({ fclk: 15.1e6, src: "pcm", rev: "A" }).ok && K.chipSetup({ fclk: 15.13e6, src: "pcm", rev: "A" }).ok, "rev A PCM limit 15.12 MHz");
chk(!K.chipSetup({ fclk: 15.5e6, src: "pdm" }).ok && K.chipSetup({ fclk: 16.384e6, src: "pdm" }).ok && K.chipSetup({ fclk: 16.384e6, src: "pdm" }).d.fs === 16000, "PDM: PDM_DIV >= 7, exactly 16 kHz at 16.384 MHz");
chk(!K.chipSetup({ fclk: 50e6, mem: "ramB", base: 0x7FF000 }).ok, "scratch overlap detected");
chk(R(K.chipSetup({ fclk: 50e6, mem: "flash" }), "BURST") === 5, "flash takes 128 B bursts");
// JS == Python over a sweep
const cases = [];
for (let mhz = 10; mhz <= 60; mhz += 0.75) for (const src of ["pdm", "pcm", "feat"]) for (const mem of ["ramA", "ramB", "flash"]) for (const rev of ["A", "B"]) cases.push({ fclk: Math.round(mhz * 1e6), src, mem, rev });
const py = `import json,sys; sys.path.insert(0, ${JSON.stringify(path.join(__dirname, "..", "scripts"))}); import kws_host as k
out=[]
for c in json.load(sys.stdin):
    regs, d, e, w = k.chip_setup(c["fclk"], src=c["src"], mem=c["mem"], rev=c["rev"])
    out.append([regs, len(e) == 0, round(d["nn_ms"], 6)])
print(json.dumps(out))`;
const pyOut = JSON.parse(cp.execFileSync("python3", ["-c", py], { input: JSON.stringify(cases) }).toString());
let diff = 0;
cases.forEach((c, i) => {
  const r = K.chipSetup(c), js = [r.regs.map(x => [x.a, x.v]), r.ok, Math.round(r.d.nnMs * 1e6) / 1e6];
  if (JSON.stringify(js) !== JSON.stringify(pyOut[i])) { diff++; if (diff < 4) console.log("JS/Python differ", c, JSON.stringify(js), JSON.stringify(pyOut[i])); }
});
console.log(`set-up calculator: reference points ${bad ? "FAIL" : "ok"} · JS vs Python: ${cases.length} cases, ${diff} differences`);
process.exit(bad || diff ? 1 : 0);
