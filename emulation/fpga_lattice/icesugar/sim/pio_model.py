"""Cycle model of the RP2350 PIO, DMA re-arm and core-1 server used by pico_psram_host.

It executes the opcodes pioasm generated from src/qspi_psram.pio (read from the generated
qspi_psram.pio.h), so the co-simulation checks the real program bits, not a re-description.
Modelled: 2-cycle input synchroniser, side-set (applied even when the instruction stalls),
autopush/autopull, FIFO depths (TX joined to 8 for qspi_tx), stalls, SMx_INSTR exec injection
by DMA (latency DMA_LAT cycles), core-1 latency per word.  One call of step() = one sys clock.
"""
import re
from collections import deque


def load_programs(header_path):
    txt = open(header_path).read()
    progs = {}
    for name in ("qspi_csdet", "qspi_rx", "qspi_tx", "qspi_cs"):
        arr = re.search(r"static (?:__pio_const|const) uint16_t %s_program_instructions\[\] = \{(.*?)\};" % name, txt, re.S).group(1)
        ops = [int(v, 16) for v in re.findall(r"0x([0-9a-f]{4})", arr)]
        wt = int(re.search(r"#define %s_wrap_target (\d+)" % name, txt).group(1))
        wr = int(re.search(r"#define %s_wrap (\d+)" % name, txt).group(1))
        body = re.search(r"%s_program_get_default_config\(uint offset\) \{(.*?)return c;" % name, txt, re.S).group(1)
        m = re.search(r"sm_config_set_sideset\(&c, (\d+), (true|false), (true|false)\)", body)
        ss = (int(m.group(1)), m.group(2) == "true", m.group(3) == "true") if m else (0, False, False)
        start = re.search(r"#define %s_offset_start (\d+)u?" % name, txt)
        labels = {m.group(1): int(m.group(2)) for m in re.finditer(r"#define %s_offset_(\w+) (\d+)u?" % name, txt)}
        progs[name] = dict(ops=ops, wrap_target=wt, wrap=wr, sideset=ss, start=int(start.group(1)) if start else 0,
                           labels=labels)
    return progs


class SM:
    def __init__(self, prog, base, in_base, out_base=0, ss_base=0, push_thr=32, pull_thr=32,
                 autopush=False, autopull=False, rx_depth=4, tx_depth=4, name="", jmp_pin=None, in_count=32):
        self.jmp_pin = jmp_pin
        self.in_mask = (1 << in_count) - 1         # RP2350 SHIFTCTRL.IN_COUNT (IN PINS, WAIT PIN, MOV x, PINS)
        self.name = name
        self.ops = list(prog["ops"]); self.base = base
        self.wrap_target = base + prog["wrap_target"]; self.wrap = base + prog["wrap"]
        self.ss_bits, self.ss_opt, self.ss_pindirs = prog["sideset"]
        self.pc = base + prog["start"]
        self.in_base, self.out_base, self.ss_base = in_base, out_base, ss_base
        self.push_thr, self.pull_thr, self.autopush, self.autopull = push_thr, pull_thr, autopush, autopull
        self.x = self.y = 0
        self.isr = 0; self.isr_cnt = 0
        self.osr = 0; self.osr_cnt = 32            # empty
        self.rx = deque(); self.tx = deque(); self.rx_depth = rx_depth; self.tx_depth = tx_depth
        self.exec_op = None
        self.tx_stall = False
        self.delay = 0                          # delay cycles left of the last instruction
        self.set_base = out_base

    def step(self, pio, pins):
        if self.delay:
            self.delay -= 1
            return
        if self.exec_op is not None:
            op, from_exec = self.exec_op, True
        else:
            op, from_exec = self.ops[self.pc - self.base], False
        # ---- side-set (5 bits field split: delay/side-set)
        dss = (op >> 8) & 0x1F
        if self.ss_bits:
            side_field = dss >> (5 - self.ss_bits)
            if self.ss_opt:
                en = side_field >> (self.ss_bits - 1) & 1
                val = side_field & ((1 << (self.ss_bits - 1)) - 1)
                nbits = self.ss_bits - 1
            else:
                en, val, nbits = 1, side_field, self.ss_bits
            if en:
                for i in range(nbits):
                    if self.ss_pindirs: pio.dirs[self.ss_base + i] = (val >> i) & 1
                    else: pio.outs[self.ss_base + i] = (val >> i) & 1
        self._reloc = 0 if from_exec else self.base     # pio_add_program relocates program jmps
        stalled = self._exec(op, pio, pins)
        if stalled:
            return
        self.delay = dss & ((1 << (5 - self.ss_bits)) - 1)   # [n] delay, after the instruction completes
        if from_exec:
            self.exec_op = None
            if self._jumped:
                return
            return                                  # exec'd non-jump: PC unchanged
        if not self._jumped:
            self.pc = self.wrap_target if self.pc == self.wrap else self.pc + 1

    def _exec(self, op, pio, pins):
        self._jumped = False
        cls = op >> 13
        a = (op >> 5) & 7
        b = op & 0x1F
        if cls == 0:                                   # JMP
            cond, addr = a, b
            take = {0: True, 1: self.x == 0, 2: self.x != 0, 3: self.y == 0, 4: self.y != 0,
                    5: self.x != self.y, 6: bool((pins >> (self.jmp_pin or 0)) & 1),
                    7: self.osr_cnt < self.pull_thr}[cond]
            if cond == 2: self.x = (self.x - 1) & 0xFFFFFFFF
            if cond == 4: self.y = (self.y - 1) & 0xFFFFFFFF
            if take: self.pc = (addr + self._reloc) & 31; self._jumped = True
            return False
        if cls == 1:                                   # WAIT
            pol, src, idx = (op >> 7) & 1, (op >> 5) & 3, op & 0x1F
            if src == 2:                               # 'wait 1 irq n' (this PIO's flags, index mode 0)
                assert pol == 1 and (idx >> 3) == 0, "only 'wait 1 irq n' modelled"
                if (idx & 7) in pio.irq:
                    pio.irq.discard(idx & 7); return False
                return True
            assert src == 1, "only 'wait pin' / 'wait irq' modelled"
            return ((((pins >> self.in_base) & self.in_mask) >> idx) & 1) != pol
        if cls == 2:                                   # IN
            src, n = a, (b or 32)
            if self.autopush and self.isr_cnt >= self.push_thr:
                if len(self.rx) >= self.rx_depth: return True
                self.rx.append(self.isr); self.isr = 0; self.isr_cnt = 0
            v = {0: (pins >> self.in_base) & self.in_mask, 1: self.x, 2: self.y, 3: 0}[src] & ((1 << n) - 1)
            self.isr = ((self.isr << n) | v) & 0xFFFFFFFF
            self.isr_cnt = min(32, self.isr_cnt + n)
            if self.autopush and self.isr_cnt >= self.push_thr:
                if len(self.rx) < self.rx_depth:
                    self.rx.append(self.isr); self.isr = 0; self.isr_cnt = 0
            return False
        if cls == 3:                                   # OUT
            dst, n = a, (b or 32)
            if self.autopull and self.osr_cnt >= self.pull_thr:
                if not self.tx: self.tx_stall = True; return True
                self.osr = self.tx.popleft(); self.osr_cnt = 0
            v = (self.osr >> (32 - n)) & ((1 << n) - 1)
            self.osr = (self.osr << n) & 0xFFFFFFFF
            self.osr_cnt = min(32, self.osr_cnt + n)
            assert dst == 0, "only 'out pins' modelled"
            for i in range(n): pio.outs[self.out_base + i] = (v >> i) & 1
            return False
        if cls == 4:                                   # PUSH / PULL
            is_pull, blk = (op >> 7) & 1, (op >> 5) & 1
            if not is_pull:
                if len(self.rx) >= self.rx_depth:
                    if blk: return True
                else:
                    self.rx.append(self.isr)
                self.isr = 0; self.isr_cnt = 0
            else:
                # datasheet (PULL): "When autopull is enabled, the behaviour of PULL is altered: it becomes a
                # no-op if the OSR is full" (shift count 0) - a fence against an autopull that already happened
                if self.autopull and self.osr_cnt == 0:
                    return False
                if not self.tx:
                    if blk: self.tx_stall = True; return True
                    self.osr = self.x
                else:
                    self.osr = self.tx.popleft()
                self.osr_cnt = 0
            return False
        if cls == 5:                                   # MOV
            dst, srcsel = a, op & 7
            assert ((op >> 3) & 3) == 0, "mov op not modelled"
            v = {0: (pins >> self.in_base) & self.in_mask, 1: self.x, 2: self.y, 3: 0, 6: self.isr, 7: self.osr}[srcsel]
            if dst == 1: self.x = v
            elif dst == 2: self.y = v
            elif dst == 6: self.isr = v; self.isr_cnt = 0
            elif dst == 7: self.osr = v; self.osr_cnt = 0
            else: raise AssertionError("mov dst not modelled")
            return False
        if cls == 6:                                   # IRQ (set only, this PIO): visible to the others next cycle
            clr, wt, idx = (op >> 6) & 1, (op >> 5) & 1, op & 0x1F
            assert not clr and not wt and (idx >> 3) == 0, "only 'irq set n' modelled"
            pio.irq_next.add(idx & 7)
            return False
        if cls == 7:                                   # SET
            if a == 1: self.x = b
            elif a == 2: self.y = b
            elif a == 0:
                for i in range(4): pio.outs[self.set_base + i] = (b >> i) & 1
            elif a == 4:
                for i in range(4): pio.dirs[self.set_base + i] = (b >> i) & 1
            else: raise AssertionError("set dst not modelled")
            return False
        raise AssertionError(f"opcode {op:#06x} not modelled")


class _Pio:
    def __init__(self):
        self.outs = [0] * 32; self.dirs = [0] * 32
        self.irq = set(); self.irq_next = set()


class PicoPsram:
    """Tiny Tapeout QSPI Pmod emulation of src/psram_emu.c (r15): flash on CS0, PSRAM A on CS1,
    PSRAM B on CS2.  PIO0: qspi_csdet, qspi_rx, qspi_tx; PIO1: two qspi_cs watchers whose pushes a
    DMA channel writes into rx / tx INSTR; core 1 serves the transactions.
    Pins: CS0 GP7, CS1 GP8, CS2 GP9, SD0..3 GP10..13, SCK GP14."""
    CS0, SD0 = 7, 10
    MEM = 0x10000                                       # model window of each memory (bytes at 0..64K)

    def __init__(self, header, mem_lo=b"", burst=3, dma_lat=8, core_lat=12, word_cost=10, flush_cost=6,
                 holdoff=None, model_cs=2):
        p = load_programs(header)
        # PIO0 load order as in psram_start(): csdet, rx, tx
        o_det = 0
        o_rx = o_det + len(p["qspi_csdet"]["ops"])
        o_tx = o_rx + len(p["qspi_rx"]["ops"])
        self.o_rx, self.o_tx = o_rx, o_tx
        self.det = SM(p["qspi_csdet"], o_det, self.CS0, in_count=3, name="det"); self.det.y = 7
        self.rx = SM(p["qspi_rx"], o_rx, self.SD0, autopush=True, name="rx")
        self.tx = SM(p["qspi_tx"], o_tx, self.SD0, out_base=self.SD0, autopull=False, tx_depth=8, name="tx")
        jmp = lambda addr: 0x0000 | (addr & 0x1F)    # pio_encode_jmp
        self.csa = SM(p["qspi_cs"], 0, self.CS0, in_count=3, name="csa")
        self.csb = SM(p["qspi_cs"], 0, self.CS0, in_count=3, name="csb")
        for w, o in ((self.csa, o_rx + p["qspi_rx"]["start"]), (self.csb, o_tx + p["qspi_tx"]["start"])):
            w.y = 7; w.osr = jmp(o); w.osr_cnt = 0
        self.pio0, self.pio1 = _Pio(), _Pio()
        self.sms0 = [self.det, self.rx, self.tx]
        self.sms1 = [self.csa, self.csb]
        self.labels = {"qspi_rx": p["qspi_rx"]["labels"], "qspi_tx": p["qspi_tx"]["labels"]}
        # where rx / tx park between transactions (rx: 'wait 1 irq 7' after the header, tx: 'wait 1 irq 5')
        self.park = {self.rx: o_rx + [op & ~0x1F00 for op in self.rx.ops].index(0x20c7),
                     self.tx: o_tx + [op & ~0x1F00 for op in self.tx.ops].index(0x20c5)}
        self.holdoff = 0
        if holdoff is not None: self.set_holdoff(holdoff)
        self.sync = deque([0x380, 0x380])               # 2-stage input synchroniser (CS high)
        self.mem = [bytearray(self.MEM) for _ in range(3)]
        self.mem_hi = bytearray(256)                    # PSRAM B 0x7FFF00..: h' scratch
        if mem_lo: self.load(model_cs, 0, mem_lo)
        self.burst = burst
        self.dma = []
        self.dma_lat, self.core_lat, self.word_cost, self.flush_cost = dma_lat, core_lat, word_cost, flush_cost
        self.cyc = 0
        self.core_busy_until = 0; self.core_queue = deque(); self.w0 = None; self.dev = None
        self.trace = []
        self.stats = dict(reads=0, writes=0, stale=0, bad_cmd=0, late=0, aborts=0, cs_bad=0,
                          rd_cs0=0, rd_cs1=0, rd_cs2=0, wr_not_cs2=0, out_of_range=0)

    @property
    def mem_lo(self):                                   # back-compat: PSRAM B model window
        return self.mem[2]

    def load(self, dev, addr, data):
        self.mem[dev][addr:addr + len(data)] = bytes(data)

    def set_holdoff(self, h):
        """rewrite the [delay] at the public h_* labels, as psram_set_holdoff() does on the Pico"""
        for sm, name in ((self.rx, "qspi_rx"), (self.tx, "qspi_tx")):
            for lab, i in self.labels[name].items():
                if lab.startswith("h_"): sm.ops[i] = (sm.ops[i] & ~0x1F00) | ((h & 31) << 8)
        self.holdoff = h

    def _mem(self, dev, a):
        if a < self.MEM: return self.mem[dev], a
        if dev == 2 and 0x7FFF00 <= a < 0x800000: return self.mem_hi, a - 0x7FFF00
        return None, 0

    def idle(self):
        """rx / tx parked on their IRQ wait, watchers / detector polling, nothing queued -> safe to
        skip time while all CS are high"""
        return (not self.dma and not self.core_queue and not self.rx.rx and self.w0 is None and
                self.rx.pc == self.park[self.rx] and self.tx.pc == self.park[self.tx] and
                not self.pio0.irq and not self.pio0.irq_next and
                all(not sm.delay and sm.exec_op is None for sm in self.sms0 + self.sms1))

    def step(self, pin_word):
        self.cyc += 1
        self.sync.append(pin_word)
        pins = self.sync.popleft()
        for sm in self.sms0: sm.step(self.pio0, pins)
        for sm in self.sms1: sm.step(self.pio1, pins)
        for pio in (self.pio0, self.pio1):
            pio.irq |= pio.irq_next; pio.irq_next = set()
        # ---- DMA: watcher RX FIFO -> target SM INSTR
        for w, tgt in ((self.csa, self.rx), (self.csb, self.tx)):
            if w.rx:
                self.dma.append((self.cyc + self.dma_lat, tgt, w.rx.popleft()))
        while self.dma and self.dma[0][0] <= self.cyc:
            _, tgt, op = self.dma.pop(0); tgt.exec_op = op
        # ---- core 1
        if self.cyc >= self.core_busy_until:
            if self.core_queue and self.rx.rx:
                # a new transaction started while words of the last burst were still waiting (cut short)
                self.core_queue.clear(); self.stats["aborts"] += 1
            if self.core_queue:
                if len(self.tx.tx) < self.tx.tx_depth:
                    self.tx.tx.append(self.core_queue.popleft()); self.core_busy_until = self.cyc + self.word_cost
            elif self.rx.rx:
                w = self.rx.rx.popleft()
                cost = self.core_lat
                if self.w0 is None:                     # command word: decode it and read the CS pins now
                    self.w0 = w
                    cs = (~(pins >> self.CS0)) & 7
                    self.dev = {1: 0, 2: 1, 4: 2}.get(cs)
                    if self.dev is None: self.stats["cs_bad"] += 1
                else:
                    c = self.w0 & 0x11111111; self.w0 = None
                    cmd = 0xEB if c == 0x11101011 else 0x38 if c == 0x00111000 else None
                    addr = w >> 8
                    if cmd is not None and len(self.trace) < 8: self.trace.append(cmd << 24 | addr)
                    dev = self.dev
                    if cmd == 0xEB and dev is not None:
                        if self.tx.tx: self.tx.tx.clear(); self.stats["stale"] += 1; cost += self.flush_cost
                        blen = 4 << min(self.burst, 5 if dev == 0 else 3); bcnt = blen - (addr & (blen - 1))
                        m, o = self._mem(dev, addr)
                        if m is None: self.stats["out_of_range"] += 1
                        data = bytes(m[o:o + bcnt]) if m is not None else bytes(bcnt)
                        data += bytes((-len(data)) % 4)
                        for i in range(0, len(data), 4): self.core_queue.append(int.from_bytes(data[i:i + 4], "big"))
                        self.stats["reads"] += 1; self.stats[f"rd_cs{dev}"] += 1
                    elif cmd == 0x38 and dev is not None:
                        if dev != 2: self.stats["wr_not_cs2"] += 1
                        m, o = self._mem(dev, addr)
                        if m is not None: m[o] = w & 0xFF
                        else: self.stats["out_of_range"] += 1
                        self.stats["writes"] += 1
                    else:
                        self.stats["bad_cmd"] += 1
                self.core_busy_until = self.cyc + cost
        if self.tx.tx_stall: self.stats["late"] += 1; self.tx.tx_stall = False
        oe = sum(self.pio0.dirs[self.SD0 + i] << i for i in range(4))
        val = sum(self.pio0.outs[self.SD0 + i] << i for i in range(4))
        return val, oe
