/* kws_fpga_host - Pico 2 W companion for the tiny_kws FPGA emulation on an iCESugar v1.5.
 *
 *   core 1 + PIO0/PIO1 + DMA : Tiny Tapeout QSPI Pmod emulation on the chip's uio pins (psram_emu.c):
 *                              flash on CS0, PSRAM A on CS1, PSRAM B on CS2 (r15)
 *   core 0                   : board-B host - SPI register access, golden-vector tests, live mode, USB console
 *
 * The model blob (model/deploy/model.bin) is placed at 0x000000 of the memory selected with 'c'
 * (MEMCFG.cs: 0 flash - the default, as on a real board - 1 PSRAM A, 2 PSRAM B); the other two model
 * windows are blanked so a stream from the wrong chip select fails.  The chip always writes and reads
 * back its h' scratch at PSRAM B 0x7FFF00 (CS2).
 *
 * Pico 2 W pins                         iCESugar (FPGA pin)        chip pin
 *   (iCESugar header numbering: PMOD2 pins 1-4 top row, 12-9 bottom row with 12 opposite 1)
 *   GP7         CS0 (flash)             PMOD2 pin 1 (46)           uio[0]
 *   GP8         CS1 (PSRAM A)           PMOD2 pin 10 (38)          uio[6]
 *   GP9         CS2 (PSRAM B)           PMOD2 pin 9 (36)           uio[7]
 *   GP10, GP11  SD0, SD1                PMOD2 pin 2 (44), 3 (42)   uio[1], uio[2]
 *   GP12, GP13  SD2, SD3                PMOD2 pin 12 (45), 11 (43) uio[4], uio[5]
 *   GP14        QSPI SCK                PMOD2 pin 4 (37)           uio[3]
 *   GP15        not used (was CS2 up to r14: move that wire to GP9)
 *   GP16..GP19  SPI0 MISO CSn SCK MOSI  PMOD3 26,23,27,25          uo[1], ui[3], ui[1], ui[2]
 *   GP20 DETECT, GP21 FRAME             PMOD3 28,32                uo[2], uo[3]
 *   GP22        chip reset (active low) PMOD4 21                   rst_n
 *   GND         common ground
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/structs/sio.h"

#include "kws_chip.h"
#include "psram_emu.h"
#include "kws_nn.h"
#include "gen/kws_model_blob.h"
#include "gen/kws_golden_m1.h"
#include "gen/kws_golden_m2.h"

#define FW_TAG "kws_fpga_host 2026-11-25 r15"

#ifndef SYS_KHZ
#define SYS_KHZ 150000               /* raise (e.g. 200000) for more PSRAM timing margin */
#endif

/* ---------------------------------------------------------------- chip configuration */
typedef struct { int div, rd; const char *name; } memcfg_preset_t;
static const memcfg_preset_t PRESETS[] = {
    {2, 1, "SCK = clk/4 (4.08 MHz), rd_dly 1"},
    {1, 0, "SCK = clk/2 (8.16 MHz), rd_dly 0"},
};
static int preset = 0;
static int cfg_burst = 3;            /* 32-byte bursts */
static int cfg_thr = 12, cfg_n = 4, cfg_nfill = 2, cfg_pdmdiv = 7, cfg_edge = 0;
static bool wifi_ok, live, verbose;
static uint32_t last_nn_us;
static bool holdoff_on = true;       /* 'g': SCK hold-off (r13 fix) on/off; off = the r12 PIO behaviour */
static int model_cs = MEM_FLASH;     /* 'c': memory holding the model (MEMCFG.cs) */

/* the model at 'base' in the model_cs memory only; the other model windows blanked (flash erased = FF) */
static void model_install(uint32_t base) {
    for (int d = 0; d < MEM_N; d++) mem_fill(d, d == MEM_FLASH ? 0xFF : 0x00);
    mem_load(model_cs, base, KWS_MODEL_BLOB, KWS_MODEL_BYTES);
}

static void chip_config(int div, int rd) {
    psram_set_holdoff(holdoff_on ? psram_holdoff_for_div(div, SYS_KHZ) : 0);   /* chip is in reset: bus idle */
    psram_rearm();
    chip_wr1(R_MEMCFG, (uint8_t)memcfg_of(rd, div, model_cs));  /* model in model_cs, h' scratch in PSRAM B */
    chip_wr1(R_BURST, (uint8_t)cfg_burst);
    psram_burst = (uint8_t)cfg_burst;
    uint8_t base[3] = {0, 0, 0};
    chip_wr(R_BASE0, base, 3);
    chip_wr1(R_THR, (uint8_t)cfg_thr);
    chip_wr1(R_DETCFG, (uint8_t)((cfg_nfill << 4) | cfg_n));
    chip_wr1(R_PDMDIV, (uint8_t)cfg_pdmdiv);
}

static void led(bool on) { if (wifi_ok) cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on); }

static void print_status(void) {
    uint8_t st = chip_rd1(R_STATUS), fc[2];
    chip_rd(R_FCNT0, fc, 2);
    printf("chip: STATUS 0x%02x%s%s%s  frames %u | Pmod: model in %s, %lu reads (CS0 %lu, CS1 %lu, CS2 %lu; %lu B), %lu writes"
           " (%lu not on CS2), %lu stale flushes, %lu cut bursts, %lu bad cmd, %lu bad CS, %lu out of range | last NN %lu us"
           " | pins: FRAME ok %lu missed %lu, DETECT mismatch %lu\n",
           st, st & S_OVERRUN ? " OVERRUN" : "", st & S_MODEL_ERR ? " MODEL_ERR" : "", st & S_DET ? " det" : "",
           fc[0] | fc[1] << 8, mem_name[model_cs], (unsigned long)psram_stats.reads, (unsigned long)psram_stats.rd_dev[0],
           (unsigned long)psram_stats.rd_dev[1], (unsigned long)psram_stats.rd_dev[2], (unsigned long)psram_stats.read_bytes,
           (unsigned long)psram_stats.writes, (unsigned long)psram_stats.wr_not_ramb, (unsigned long)psram_stats.stale_flush,
           (unsigned long)(psram_stats.burst_abort + psram_stats.hdr_abort),
           (unsigned long)psram_stats.bad_cmd, (unsigned long)psram_stats.cs_bad, (unsigned long)psram_stats.out_of_range,
           (unsigned long)last_nn_us,
           (unsigned long)chip_frame_pin_ok, (unsigned long)chip_frame_pin_missed, (unsigned long)chip_det_pin_mismatch);
}

/* ---------------------------------------------------------------- F1: registers */
static int test_regs(void) {
    chip_reset();
    int bad = 0;
    uint8_t id = chip_rd1(R_ID), h = chip_rd1(R_HREG), nch = chip_rd1(R_NCH), mc = chip_rd1(R_MEMCFG);
    bad += id != 0x4B; bad += h != 48; bad += nch != 16; bad += mc != 0x49;
    uint8_t w[2] = {0x5A, 0x37}, r[2];
    chip_wr(R_THR, w, 2); chip_rd(R_THR, r, 2);                     /* auto-increment THR, DETCFG */
    bad += r[0] != 0x5A || r[1] != 0x37;
    printf("F1 registers: ID 0x%02x H %d NCH %d MEMCFG 0x%02x THR/DETCFG %02x %02x -> %s\n", id, h, nch, mc,
           r[0], r[1], bad ? "FAIL" : "PASS");
    return bad;
}

/* ---------------------------------------------------------------- F3: host features -> NN (model via PSRAM) */
static int test_feat(int nclips, int div, int rd, bool quiet) {
    model_install(0);                                         /* model at 0 (the 'd' diagnostic moves it) */
    chip_reset();
    chip_config(div, rd);
    chip_wr1(R_THR, M1_THR);
    chip_wr1(R_DETCFG, (M1_NFILL << 4) | M1_N);
    psram_stats_clear(); psram_tx_stalled(); chip_pin_stats_clear();
    chip_wr1(R_CTRL, C_RUN | C_FEAT_HOST);
    int bad = 0, frames = 0;
    uint32_t nn_max = 0, nn_sum = 0;
    for (int c = 0; c < nclips; c++) {
        chip_wr1(R_CTRL, C_RUN | C_FEAT_HOST | C_CLEAR);           /* fresh hidden state per clip */
        sleep_us(20);
        for (int t = 0; t < M1_FRAMES; t++) {
            const int f = c * M1_FRAMES + t;
            uint32_t t0 = time_us_32();
            chip_wr(R_FEAT, &M1_FEAT[f * 16], 16);
            if (!chip_wait_frame(200000)) {
                if (!quiet) printf("  clip %d frame %d: FRAME timeout\n", c, t);
                bad++; goto done;
            }
            uint32_t dt = time_us_32() - t0;
            nn_sum += dt; if (dt > nn_max) nn_max = dt;
            uint8_t r[3];
            chip_rd(R_RESULT, r, 3);
            int top1 = r[0] >> 4, v1 = (int8_t)r[1], v2 = (int8_t)r[2], det = chip_frame_det();
            if (top1 != M1_TOP[f * 3] || v1 != M1_TOP[f * 3 + 1] || v2 != M1_TOP[f * 3 + 2] || det != M1_DET[f]) {
                if (!quiet && bad < 5)
                    printf("  clip %d frame %d: got %d %d/%d det %d, expected %d %d/%d det %d\n", c, t, top1, v1, v2,
                           det, M1_TOP[f * 3], M1_TOP[f * 3 + 1], M1_TOP[f * 3 + 2], M1_DET[f]);
                bad++;
            }
            frames++;
        }
    }
done:;
    uint8_t st = chip_rd1(R_STATUS);
    if (st & (S_OVERRUN | S_MODEL_ERR)) bad++;
    chip_wr1(R_CTRL, 0);
    last_nn_us = frames ? nn_sum / frames : 0;
    /* the model stream must come from model_cs only; PSRAM B also serves the h' read-back */
    int stray = 0;
    for (int d = 0; d < MEM_N; d++) if (d != model_cs && d != MEM_RAMB) stray += (int)psram_stats.rd_dev[d];
    int csbad = stray || psram_stats.cs_bad || psram_stats.wr_not_ramb || !psram_stats.rd_dev[model_cs];
    if (csbad) bad++;
    if (!quiet)
        printf("F3 host features -> NN, model in %s (MEMCFG 0x%02x, SCK hold-off %u): %d frames, %d mismatches, STATUS 0x%02x,"
               " frame latency %lu us avg / %lu max, reads CS0/CS1/CS2 %lu/%lu/%lu, %lu writes, %lu stale, %lu cut,"
               " %lu bad-cmd, %lu bad-CS, %lu out-of-range -> %s\n",
               mem_name[model_cs], memcfg_of(rd, div, model_cs), psram_holdoff(), frames, bad, st,
               (unsigned long)last_nn_us, (unsigned long)nn_max,
               (unsigned long)psram_stats.rd_dev[0], (unsigned long)psram_stats.rd_dev[1], (unsigned long)psram_stats.rd_dev[2],
               (unsigned long)psram_stats.writes, (unsigned long)psram_stats.stale_flush,
               (unsigned long)(psram_stats.burst_abort + psram_stats.hdr_abort), (unsigned long)psram_stats.bad_cmd,
               (unsigned long)psram_stats.cs_bad, (unsigned long)psram_stats.out_of_range, bad ? "FAIL" : "PASS (bit-exact)");
    if (!quiet && csbad)
        printf("   -> chip-select routing wrong: %d reads from a memory that does not hold the model, %lu with no/several CS"
               " low, %lu writes not on CS2%s\n", stray, (unsigned long)psram_stats.cs_bad,
               (unsigned long)psram_stats.wr_not_ramb, psram_stats.rd_dev[model_cs] ? "" : ", NO reads on the model CS (check that CS wire)");
    if (!quiet) {
        printf("   pins: FRAME seen on %lu of %lu frames%s, DETECT pin disagreed with STATUS.det on %lu frames%s\n",
               (unsigned long)chip_frame_pin_ok, (unsigned long)(chip_frame_pin_ok + chip_frame_pin_missed),
               chip_frame_pin_missed ? " (check FRAME wire: PMOD3 pin 12 -> GP21)" : "",
               (unsigned long)chip_det_pin_mismatch, chip_det_pin_mismatch ? " (check DETECT wire: PMOD3 pin 11 -> GP20)" : "");
    }
    return bad;
}

/* F3 with the model in each of the three memories in turn (flash, PSRAM A, PSRAM B) */
static int test_all_cs(int nclips) {
    int saved = model_cs, bad = 0;
    for (int d = 0; d < MEM_N; d++) {
        model_cs = d;
        bad += test_feat(nclips, PRESETS[preset].div, PRESETS[preset].rd, false) != 0;
    }
    model_cs = saved;
    model_install(0);
    printf("F3 on all three chip selects: %s\n", bad ? "FAIL" : "PASS (flash, PSRAM A and PSRAM B all bit-exact)");
    return bad;
}

/* ---------------------------------------------------------------- F4: host PCM -> front end + NN */
static int test_pcm(int nclips) {
    const memcfg_preset_t *p = &PRESETS[preset];
    model_install(0);
    chip_reset();
    chip_config(p->div, p->rd);
    psram_stats_clear();
    chip_wr1(R_CTRL, C_RUN | C_SRC_PCM | C_HOP512);
    int bad = 0, nnbad = 0, frames = 0;
    for (int c = 0; c < nclips; c++) {
        chip_wr1(R_CTRL, C_RUN | C_SRC_PCM | C_HOP512 | C_CLEAR);   /* clears filters, hop counter, h */
        sleep_us(20);
        uint32_t tlast = time_us_32();
        int t = 0;
        for (int i = 0; i < M2_SAMPLES; i++) {
            while (time_us_32() - tlast < 63) tight_loop_contents();  /* <= 16 kHz: FE needs 897 clk/sample */
            tlast = time_us_32();
            uint16_t v = (uint16_t)M2_PCM[c * M2_SAMPLES + i];
            uint8_t b[2] = {(uint8_t)v, (uint8_t)(v >> 8)};
            chip_wr(R_PCM, b, 2);
            if ((i + 1) % 512) continue;
            if (!chip_wait_frame(200000)) { printf("  clip %d frame %d: FRAME timeout\n", c, t); bad++; goto done; }
            uint8_t f[16], r[3];
            chip_rd(R_FEATRD, f, 16);
            chip_rd(R_RESULT, r, 3);
            const uint8_t *e = &M2_FEAT[(c * M2_FRAMES + t) * 16];
            if (memcmp(f, e, 16)) {
                if (bad < 5) {
                    printf("  clip %d frame %d features:", c, t);
                    for (int k = 0; k < 16; k++) printf(" %d/%d", f[k], e[k]);
                    printf(" (got/expected)\n");
                }
                bad++;
            }
            const int g = c * M1_FRAMES + t;
            if ((r[0] >> 4) != M1_TOP[g * 3] || (int8_t)r[1] != M1_TOP[g * 3 + 1] || (int8_t)r[2] != M1_TOP[g * 3 + 2])
                nnbad++;
            t++; frames++;
        }
    }
done:;
    uint8_t st = chip_rd1(R_STATUS);
    if (st & (S_OVERRUN | S_MODEL_ERR)) bad++;
    chip_wr1(R_CTRL, 0);
    printf("F4 host PCM -> front end -> NN (model in %s): %d frames, features %s, NN %s, STATUS 0x%02x -> %s\n",
           mem_name[model_cs], frames,
           bad ? "MISMATCH" : "bit-exact", nnbad ? "MISMATCH" : "bit-exact", st, (bad || nnbad) ? "FAIL" : "PASS");
    return bad + nnbad;
}

/* ---------------------------------------------------------------- QSPI read-window sweep */
static void sweep(void) {
    static const int divs[] = {7, 4, 2, 1};
    int okdiv[4] = {0, 0, 0, 0};
    printf("QSPI read window, model in %s (F3 on 1 clip for each setting; ok = 31 frames bit-exact):\n"
           "                          rd_dly:", mem_name[model_cs]);
    for (int rd = 0; rd < 8; rd++) printf("  %d   ", rd);
    printf("\n");
    for (unsigned i = 0; i < sizeof divs / sizeof divs[0]; i++) {
        int div = divs[i];
        printf("  SCK div %d (%5.2f MHz, half %3d ns):", div, 16.3125 / (2.0 * div), (int)(div * 61.3 + 0.5));
        int ok = 0;
        for (int rd = 0; rd < 8; rd++) { int f = test_feat(1, div, rd, true); ok += !f; printf(" %-5s", f ? "FAIL" : "ok"); }
        printf("\n");
        okdiv[i] = ok;
    }
    if (!okdiv[0] && !okdiv[1] && !okdiv[2] && !okdiv[3])
        printf("  -> FAIL everywhere: wiring - run 'd'\n");
    else if (!okdiv[2] || !okdiv[3])
        printf("  -> passes only at the slower speeds: signal integrity (shorter wires, a GND wire next to SCK, more GND wires)\n");
    else
        printf("  -> expected window (co-sim): div 2 rd_dly 0-3, div 1 rd_dly 0-1; presets: div 2 rd 1, div 1 rd 0\n");
}

/* ---------------------------------------------------------------- whole-frame bus walk (diagnostic) */
/* Walks the raw sniffer samples (one per SCK rising edge the Pico saw, no hold-off) transaction by
 * transaction, in the order core 1 logged the headers.  A transaction with MORE SCK edges than the chip
 * sends (read: 20 + 2*bytes - 1 samples, write: 16) means the Pico saw spikes on SCK (glitches).
 * Read data is compared with the Pico's memory only in transactions without extra edges. */
typedef struct { uint32_t ntx, nwr, nbytes, nbad, extra, gtx, lost, first_extra, scratch_skipped; } walk_t;
static void bus_walk(const uint8_t *sm, int ns, int show, walk_t *o) {
    memset(o, 0, sizeof *o);
    #define SD(i) SNIFF_SD(sm[i])
    uint32_t nt = psram_trace_n < PSRAM_TRACE_N ? psram_trace_n : PSRAM_TRACE_N;
    int32_t ppos = -1; uint32_t pw = 0, pos = 0; int pdev = 0;
    int shown_x = 0, shown_b = 0;
    for (uint32_t t = 0; t <= nt; t++) {
        int32_t q = -1; uint32_t w = 0;
        if (t < nt) {
            w = psram_trace[t];
            for (uint32_t s = pos; s + 14 <= (uint32_t)ns && s < pos + 2048; s++) {
                uint32_t c = 0, a = 0;
                for (int i = 0; i < 8; i++) c = (c << 1) | (SD(s + i) & 1);
                if (c != (w >> 24)) continue;
                for (int i = 0; i < 6; i++) a = (a << 4) | SD(s + 8 + i);
                if (a == (w & 0xFFFFFF)) { q = (int32_t)s; break; }
            }
            if (q < 0 && pos + 40 < (uint32_t)ns) {
                o->lost = 1;
                if (show) printf("  header of transaction %lu (logged %08lx) not found after sample %lu: walk stopped\n",
                                 (unsigned long)o->ntx, (unsigned long)w, (unsigned long)pos);
            }
        }
        if (ppos >= 0) {                                               /* finish the previous transaction */
            bool last = q < 0;
            uint32_t end = last ? (uint32_t)ns : (uint32_t)q, n = end - (uint32_t)ppos;
            uint32_t c = pw >> 24, a = pw & 0xFFFFFF, exp;
            o->ntx++;
            if (c == 0x38) { o->nwr++; exp = 16; }
            else {
                uint32_t bmax = pdev == MEM_FLASH ? 5u : 3u;
                uint32_t b = psram_burst & 7u; if (b > bmax) b = bmax;
                uint32_t blen = 4u << b, bcnt = blen - (a & (blen - 1u));
                exp = 20 + 2 * bcnt - 1;
            }
            if (!last && n > exp) {
                if (o->ntx == 1) o->first_extra = n - exp;
                o->extra += n - exp; o->gtx++;
                if (show && shown_x++ < 6)
                    printf("  %s 0x%06lx: %lu SCK rising edges, the chip sends %lu -> %lu EXTRA (sample %ld)\n",
                           c == 0x38 ? "write" : "read ", (unsigned long)a, (unsigned long)n, (unsigned long)exp,
                           (unsigned long)(n - exp), (long)ppos);
            } else if (c == 0x38) {
                if (n >= 16) {
                    uint8_t v = (uint8_t)((SD(ppos + 14) << 4) | SD(ppos + 15));
                    if (mem_peek(MEM_RAMB, a) != v) { o->nbad++; if (show && shown_b++ < 6) printf("  write 0x%06lx: bus %02x, memory %02x\n", (unsigned long)a, v, mem_peek(MEM_RAMB, a)); }
                }
            } else if (a >= 0x7FFF00u) {
                o->scratch_skipped++;      /* h' read-back: the chip rewrites these bytes later in the frame */
            } else {
                for (uint32_t s = (uint32_t)ppos + 20, k = 0; s + 1 < end; s += 2, k++) {
                    uint8_t v = (uint8_t)((SD(s) << 4) | SD(s + 1)), e = mem_peek(pdev, a + k);
                    o->nbytes++;
                    if (v != e) { o->nbad++; if (show && shown_b++ < 6) printf("  read %s 0x%06lx (+%lu): bus %02x, memory %02x\n", mem_name[pdev], (unsigned long)(a + k), (unsigned long)k, v, e); }
                }
            }
        }
        if (q < 0) break;
        ppos = q; pw = w; pos = (uint32_t)q + 16; pdev = psram_trace_dev[t] < MEM_N ? psram_trace_dev[t] : 0;
    }
    #undef SD
}

/* h' the chip wrote to the scratch vs the golden GRU (first frame of clip 0) */
static int hprime_diff(bool show) {
    static kws_model_t mdl; static kws_gru_t g;
    if (kws_model_parse(&mdl, KWS_MODEL_BLOB, KWS_MODEL_BYTES) != 0) return -1;
    kws_gru_reset(&g, &mdl);
    kws_gru_step(&g, &M1_FEAT[0]);
    int hb = 0;
    for (int j = 0; j < mdl.H; j++) hb += (int8_t)mem_peek(MEM_RAMB, MEM_SCRATCH + j) != g.h[j];
    if (show) {
        printf("h' in scratch (chip) vs golden, first 16 of %d:\n  chip  :", mdl.H);
        for (int j = 0; j < 16; j++) printf(" %4d", (int8_t)mem_peek(MEM_RAMB, MEM_SCRATCH + j));
        printf("\n  golden:");
        for (int j = 0; j < 16; j++) printf(" %4d", (int)g.h[j]);
        printf("\n  %d of %d h' bytes differ; golden logits:", hb, mdl.H);
        for (int c = 0; c < mdl.C; c++) printf(" %d", (int)g.logit[c]);
        printf("\n  -> %s\n", hb ? "the chip's GRU output h' is wrong (compute or weight stream)"
                                   : "h' is right: the error is in the read-back of h' or the classifier");
    }
    return hb;
}

/* one diagnostic frame (clip 0 frame 0) with the model at 'base', div 7 / rd_dly 3; returns 1 if bit-exact */
static int diag_frame(uint32_t base, uint8_t r[3], uint8_t *st) {
    model_install(base);
    for (int j = 0; j < 48; j++) { uint8_t z = 0; mem_load(MEM_RAMB, MEM_SCRATCH + j, &z, 1); }
    chip_reset();
    chip_config(7, 3);
    uint8_t b[3] = {(uint8_t)base, (uint8_t)(base >> 8), (uint8_t)(base >> 16)};
    chip_wr(R_BASE0, b, 3);
    chip_wr1(R_THR, M1_THR);
    chip_wr1(R_DETCFG, (M1_NFILL << 4) | M1_N);
    psram_stats_clear();
    psram_trace_arm();
    chip_wr1(R_CTRL, C_RUN | C_FEAT_HOST);
    chip_wr1(R_CTRL, C_RUN | C_FEAT_HOST | C_CLEAR);
    sleep_us(20);
    psram_sniff_arm();
    chip_wr(R_FEAT, &M1_FEAT[0], 16);
    bool fr = chip_wait_frame(300000);
    r[0] = r[1] = r[2] = 0;
    if (fr) chip_rd(R_RESULT, r, 3);
    *st = chip_rd1(R_STATUS);
    chip_wr1(R_CTRL, 0);
    return fr && (r[0] >> 4) == M1_TOP[0] && (int8_t)r[1] == M1_TOP[1] && (int8_t)r[2] == M1_TOP[2];
}

/* ---------------------------------------------------------------- QSPI wiring diagnostic */
/* The chip sends the model base address 0x001248 on SD3..SD0: one nibble per line in turn
 * (0001, 0010, 0100, 1000), so the received address shows exactly which FPGA SD line reaches
 * which Pico pin. The same wires carry the read data back. */
static int diag(void) {
    static const char *const sdname[4] = {"SD0 (uio1, FPGA 44, PMOD2 pin 2)", "SD1 (uio2, FPGA 42, PMOD2 pin 3)",
                                          "SD2 (uio4, FPGA 45, PMOD2 pin 12)", "SD3 (uio5, FPGA 43, PMOD2 pin 11)"};
    const uint32_t base = 0x001248;
    int bad = 0;
    printf("---- QSPI wiring diagnostic [%s] ----\n", FW_TAG);
    chip_reset();                                                  /* RUN = 0: the chip releases uio */
    sleep_ms(2);
    int cs0 = gpio_get(PSRAM_PIN_CS0), cs1 = gpio_get(PSRAM_PIN_CS1), cs2 = gpio_get(PSRAM_PIN_CS2), sck = gpio_get(PSRAM_PIN_SCK);
    printf("bus released: CS0 (GP7) = %d, CS1 (GP8) = %d, CS2 (GP9) = %d (all expect 1, pull-ups), SCK (GP14) = %d (expect 0)\n",
           cs0, cs1, cs2, sck);
    if (!cs0 || !cs1 || !cs2) { printf("  -> a CS line is held low: check that wire and its PMOD2 pin\n"); bad++; }
    /* drive test: with the chip in reset the FPGA releases uio, so the Pico must be able to drive SD0..3 */
    {
        int dbad = 0;
        printf("Pico drive test (chip in reset):");
        for (int k = 0; k < 4; k++) {
            uint p = PSRAM_PIN_SD0 + k;
            gpio_set_function(p, GPIO_FUNC_SIO); gpio_set_dir(p, GPIO_OUT);
            gpio_put(p, 1); busy_wait_us_32(5); int h = gpio_get(p);
            gpio_put(p, 0); busy_wait_us_32(5); int l = gpio_get(p);
            gpio_set_dir(p, GPIO_IN); gpio_set_function(p, GPIO_FUNC_PIO0);
            printf(" GP%u %s", p, (h == 1 && l == 0) ? "ok" : (h == l ? (h ? "STUCK-1" : "STUCK-0") : "??"));
            dbad += !(h == 1 && l == 0);
        }
        printf("%s\n", dbad ? "  -> a line is held by something else (FPGA still driving? short?)" : "");
        bad += dbad;
    }
    model_install(base);
    chip_config(7, 3);                                             /* slowest SCK: 1.17 MHz */
    uint8_t b[3] = {(uint8_t)base, (uint8_t)(base >> 8), (uint8_t)(base >> 16)};
    chip_wr(R_BASE0, b, 3);
    chip_wr1(R_THR, M1_THR);
    chip_wr1(R_DETCFG, (M1_NFILL << 4) | M1_N);
    psram_stats_clear();
    psram_trace_arm();
    chip_wr1(R_CTRL, C_RUN | C_FEAT_HOST);
    chip_wr1(R_CTRL, C_RUN | C_FEAT_HOST | C_CLEAR);
    sleep_us(20);
    psram_sniff_arm();
    psram_csraw_clear();
    chip_pin_stats_clear();
    chip_wr(R_FEAT, &M1_FEAT[0], 16);
    /* live trace of both PSRAM state machines' program counters while the chip reads (div 7: ~20 us) */
    static uint16_t tr[600];
    int ntr = 0;
    {
        uint32_t last = 0xFFFF, t0 = time_us_32();
        while (time_us_32() - t0 < 3000 && ntr < (int)(sizeof tr / sizeof tr[0])) {
            uint32_t g = sio_hw->gpio_in;
            uint32_t v = (pio0->sm[0].addr & 31) | ((pio0->sm[1].addr & 31) << 5) |
                         (((g >> PSRAM_PIN_SCK) & 1) << 10) | ((uint32_t)(((g >> PSRAM_PIN_CS0) & 7) == 7) << 11);
            if (v != last) { tr[ntr++] = (uint16_t)v; last = v; }
        }
    }
    bool fr = chip_wait_frame(300000);
    uint8_t r[3] = {0, 0, 0}, st;
    if (fr) chip_rd(R_RESULT, r, 3);
    st = chip_rd1(R_STATUS);
    chip_wr1(R_CTRL, 0);
    uint32_t n = psram_trace_n;
    printf("first words received by the Pico (%lu):", (unsigned long)n);
    for (uint32_t i = 0; i < n && i < 8; i++) printf(" %08lx", (unsigned long)psram_trace[i]);
    printf("\n");
    if (n == 0) {
        printf("  -> no transaction decoded: check the model CS (CS0 PMOD2 pin 1 -> GP7, CS1 pin 10 -> GP8, CS2 pin 9 -> GP9),"
               " SCK (pin 4 -> GP14), SD0 (pin 2 -> GP10)\n");
        return bad + 1;
    }
    uint32_t w = psram_trace[0], cmd = w >> 24, addr = w & 0xFFFFFF;
    int dev0 = psram_trace_dev[0];
    printf("first: cmd 0x%02lx (expect 0xeb), addr 0x%06lx (expect 0x%06lx), on %s (expect %s)\n", (unsigned long)cmd,
           (unsigned long)addr, (unsigned long)base, dev0 < MEM_N ? mem_name[dev0] : "no / several CS low", mem_name[model_cs]);
    if (cmd != 0xEB) { printf("  -> command wrong: SD0 or SCK wiring / noise\n"); bad++; }
    if (dev0 != model_cs) { printf("  -> chip select wrong: check the CS wires (CS0 PMOD2 pin 1 -> GP7, CS1 pin 10 -> GP8, CS2 pin 9 -> GP9)\n"); bad++; }
    if (addr >> 16) { printf("  -> address bits 23:16 not zero: an SD line is floating or picking up noise\n"); bad++; }
    for (int k = 0; k < 4; k++) {                                  /* nibble k carries 1 << k on the FPGA side */
        int got = (addr >> (12 - 4 * k)) & 0xF;
        printf("  FPGA %-34s -> ", sdname[k]);
        if (got == (1 << k)) printf("GP%d  ok\n", PSRAM_PIN_SD0 + k);
        else if (got == 0) { printf("no Pico pin  (open wire)\n"); bad++; }
        else {
            printf("Pico");
            for (int j = 0; j < 4; j++) if (got & (1 << j)) printf(" GP%d", PSRAM_PIN_SD0 + j);
            printf("  WRONG (expected GP%d)%s\n", PSRAM_PIN_SD0 + k, (got & (got - 1)) ? " - lines shorted?" : "");
            bad++;
        }
    }
    /* what was on the bus during the first transaction */
    {
        const uint8_t *sm;
        int ns = psram_sniff_read(&sm);
        uint rxpc, txpc, txl, rxl; uint32_t fd;
        psram_sm_state(&rxpc, &txpc, &txl, &rxl, &fd);
        unsigned raw = psram_csraw_count();
        printf("Pico SMs after the frame: rx pc %u, tx pc %u (start %u), tx FIFO %u words, rx FIFO %u, FDEBUG 0x%08lx\n",
               rxpc, txpc, psram_tx_start_pc(), txl, rxl, (unsigned long)fd);
        {   /* program-counter trace, relative to each program (see qspi_psram.pio.h for the listing):
             * qspi_tx: 0 set pindirs 0 | 1 set x,3 | 2 wait IRQ 5 | 3-5 edges 17-20 | 6 pull | 7 set y |
             *          8 wait SCK low | 9 out | 10 set pindirs 15 | 11 wait SCK high | 12 jmp y--.
             * qspi_rx: 2 wait IRQ 4 (CS fall) | 3 jmp first | 4-7 16-nibble loop | 8 irq 5 | 9 parked */
            unsigned orx = psram_off_rx(), otx = psram_off_tx();
            for (int i = 0; i < ntr; i++)
                tr[i] = (uint16_t)((((tr[i] & 31) - orx) & 31) | (((((tr[i] >> 5) & 31) - otx) & 31) << 5) | (tr[i] & 0xC00));
            printf("SM trace (%d samples; PIO0 offsets rx %u, tx %u): first rx/tx/SCK/CS =", ntr, orx, otx);
            for (int i = 0; i < ntr && i < 6; i++)
                printf(" %u/%u/%u/%u", tr[i] & 31, (tr[i] >> 5) & 31, (tr[i] >> 10) & 1, (tr[i] >> 11) & 1);
            printf("\n  tx pc sequence:");
            int last = -1, n = 0;
            for (int i = 0; i < ntr && n < 90; i++) {
                int t = (tr[i] >> 5) & 31;
                if (t != last) { printf(" %d", t); last = t; n++; }
            }
            printf("\n  rx pc sequence:");
            last = -1; n = 0;
            for (int i = 0; i < ntr && n < 90; i++) {
                int r = tr[i] & 31;
                if (r != last) { printf(" %d", r); last = r; n++; }
            }
            int sck_rise = 0, prev = (tr[0] >> 10) & 1;
            for (int i = 1; i < ntr; i++) { int c = (tr[i] >> 10) & 1; if (c && !prev) sck_rise++; prev = c; }
            printf("\n  SCK rising edges seen by the trace: %d\n", sck_rise);
        }
        printf("CS rises (all three high again, raw, glitches included): %u%s for %lu transactions%s\n", raw, raw >= 8 ? "+" : "",
               (unsigned long)(psram_stats.reads + psram_stats.writes),
               raw > psram_stats.reads + psram_stats.writes ? "  -> CS GLITCHES (filtered by the watchers)" : "");
        printf("bus at each SCK rising edge (%d edges captured; SD3..0 hex, '-' = all CS high):\n  ", ns);
        for (int i = 0; i < ns && i < 96; i++) {
            printf("%c", SNIFF_CS(sm[i]) == 7 ? '-' : "0123456789abcdef"[SNIFF_SD(sm[i])]);
            if (i == 7 || i == 13 || i == 15 || i == 19) printf(" | ");
        }
        printf("\n  (edges 1-8 command on SD0, 9-14 address, 15-16 mode, 17-20 dummy, 21+ data)\n");
        printf("  data on the bus:");
        int nb = 0, dok = 0;
        for (int i = 20; i + 1 < ns && nb < 16; i += 2, nb++) {     /* (first burst only, full check below) */
            uint8_t v = (uint8_t)((SNIFF_SD(sm[i]) << 4) | SNIFF_SD(sm[i + 1]));
            uint8_t e = KWS_MODEL_BLOB[nb];
            printf(" %02x", v);
            dok += v == e;
        }
        printf("\n  expected:       ");
        for (int i = 0; i < nb; i++) printf(" %02x", KWS_MODEL_BLOB[i]);
        printf("\n");
        walk_t w0; bus_walk(sm, ns, 0, &w0);
        if (nb && dok == nb) printf("  -> the Pico puts the right model bytes on the bus\n");
        else if (nb && w0.first_extra)
            printf("  -> this burst had %lu extra SCK spike(s): the raw sniffer records them too, so this view is shifted\n"
                   "     (the data SM ignores them with the hold-off on - see the A/B lines at the end)\n", (unsigned long)w0.first_extra);
        else if (nb) printf("  -> wrong data on the bus (%d of %d bytes right): Pico output path\n", dok, nb);
        else printf("  -> the chip stopped before the data phase\n");
    }
    printf("FRAME pin: %s\n", chip_frame_pin_missed ? "NOT seen - frame end taken from the frame counter (check PMOD3 pin 12 -> GP21)"
                                                    : (fr ? "ok" : "no frame"));
    {   /* whole-frame bus check (raw sniffer: every SCK rising edge the Pico saw) */
        const uint8_t *sm; int ns = psram_sniff_read(&sm);
        walk_t wk;
        bus_walk(sm, ns, 1, &wk);
        printf("full bus check: %d edges sniffed, %lu transactions walked (%lu writes, %lu h' read-backs not compared),"
               " %lu model bytes compared in spike-free reads, %lu mismatches%s\n",
               ns, (unsigned long)wk.ntx, (unsigned long)wk.nwr, (unsigned long)wk.scratch_skipped, (unsigned long)wk.nbytes,
               (unsigned long)wk.nbad,
               (ns >= SNIFF_N) ? " (sniffer buffer full: frame partly covered)" : "");
        printf("SCK glitches: the Pico saw %lu extra SCK rising edges in %lu of %lu transactions\n",
               (unsigned long)wk.extra, (unsigned long)wk.gtx, (unsigned long)wk.ntx);
        if (wk.extra)
            printf("  -> spikes on the SCK wire (crosstalk from the SD lines / ground bounce). With the hold-off off the\n"
                   "     data SM takes each spike for a clock edge, runs ahead and the chip captures shifted weights.\n");
        else if (wk.nbad)
            printf("  -> the Pico put WRONG data on the bus at the places listed\n");
        else
            printf("  -> every nibble on the bus was right\n");
    }
    bool res_ok = fr && (r[0] >> 4) == M1_TOP[0] && (int8_t)r[1] == M1_TOP[1] && (int8_t)r[2] == M1_TOP[2];
    {   /* did any chip write land in the model area (corrupting weights)? */
        int mb = 0, first = -1;
        for (int i = 0; i < KWS_MODEL_BYTES; i++)
            if (mem_peek(model_cs, base + i) != KWS_MODEL_BLOB[i]) { if (first < 0) first = i; mb++; }
        printf("model in Pico memory after the frame: %d of %d bytes changed%s", mb, KWS_MODEL_BYTES, mb ? "" : " (intact)");
        if (mb) printf(", first at model offset %d (0x%06x)", first, (unsigned)(base + first));
        printf(" | writes: %lu total, %lu into the model area (last at 0x%06lx)\n",
               (unsigned long)psram_stats.writes, (unsigned long)psram_stats.writes_model_area,
               (unsigned long)psram_stats.last_bad_waddr);
        printf("scratch 0x7FFF00..2F (all 48 h' bytes the chip wrote):\n ");
        for (int j = 0; j < 48; j++) { printf(" %4d", (int8_t)mem_peek(MEM_RAMB, MEM_SCRATCH + j)); if (j % 16 == 15) printf("\n "); }
        printf("\n");
    }
    printf("RESULT: top1 %d (%s) %d/%d, expected top1 %d (%s) %d/%d\n", r[0] >> 4, KWS_CLASS_NAMES[r[0] >> 4],
           (int8_t)r[1], (int8_t)r[2], M1_TOP[0], KWS_CLASS_NAMES[M1_TOP[0]], M1_TOP[1], M1_TOP[2]);
    {   /* features as the chip holds them (FEATRD 0x30..0x3F) vs what was written over SPI */
        uint8_t fr16[16]; int fb = 0;
        chip_rd(R_FEATRD, fr16, 16);
        printf("features in the chip (FEATRD) vs written:\n  chip   :");
        for (int k = 0; k < 16; k++) { printf(" %3d", fr16[k]); fb += fr16[k] != M1_FEAT[k]; }
        printf("\n  written:");
        for (int k = 0; k < 16; k++) printf(" %3d", M1_FEAT[k]);
        printf("\n  -> %s\n", fb ? "the feature bytes did not arrive intact (SPI FEAT path)" : "features arrived intact");
    }
    hprime_diff(true);
    /* the copy at 0x001248 overlapped the model at 0: restore it for the other tests */
    model_install(0);
    printf("frame at SCK 1.17 MHz: %s, result %s, STATUS 0x%02x%s | PSRAM %lu reads %lu writes, out of range %lu\n",
           fr ? "done" : "TIMEOUT", res_ok ? "bit-exact" : "WRONG", st, st & S_MODEL_ERR ? " MODEL_ERR" : "",
           (unsigned long)psram_stats.reads, (unsigned long)psram_stats.writes, (unsigned long)psram_stats.out_of_range);
    {   /* A/B: the same frame with the SCK hold-off off (r12 PIO behaviour) and on */
        bool saved = holdoff_on;
        printf("SCK hold-off A/B (same frame, SCK 1.17 MHz):\n");
        for (int pass = 0; pass < 2; pass++) {
            holdoff_on = pass == 1;
            uint8_t rr[3], st2;
            int ok = diag_frame(base, rr, &st2);
            const uint8_t *sm; int ns = psram_sniff_read(&sm);
            walk_t wk; bus_walk(sm, ns, 0, &wk);
            int hb = hprime_diff(false);
            printf("  hold-off %-3s (%2u cycles): result %s (%d %d/%d), h' %d of 48 bytes wrong, %lu extra SCK edges seen"
                   " in %lu of %lu transactions\n", holdoff_on ? "on" : "off", psram_holdoff(), ok ? "bit-exact" : "WRONG",
                   rr[0] >> 4, (int8_t)rr[1], (int8_t)rr[2], hb, (unsigned long)wk.extra, (unsigned long)wk.gtx,
                   (unsigned long)wk.ntx);
            if (pass == 1) res_ok = ok;                    /* the verdict below is for the hold-off on (the fix) */
        }
        holdoff_on = saved;
    }
    {   /* chip-select wiring: one frame with the model in each memory - its CS must carry the stream */
        int saved = model_cs;
        printf("chip selects (one frame each, model at 0x%06lx):\n", (unsigned long)base);
        static const char *const csname[MEM_N] = {"CS0 (uio0, FPGA 46, PMOD2 pin 1)  -> GP7",
                                                 "CS1 (uio6, FPGA 38, PMOD2 pin 10) -> GP8",
                                                 "CS2 (uio7, FPGA 36, PMOD2 pin 9)  -> GP9"};
        for (int d = 0; d < MEM_N; d++) {
            model_cs = d;
            uint8_t rr[3], st2;
            int ok = diag_frame(base, rr, &st2);
            int stray = 0;
            for (int e = 0; e < MEM_N; e++) if (e != d && e != MEM_RAMB) stray += (int)psram_stats.rd_dev[e];
            bool line_ok = ok && psram_stats.rd_dev[d] && !stray && !psram_stats.cs_bad && !psram_stats.wr_not_ramb;
            printf("  %s: %s, reads CS0/CS1/CS2 %lu/%lu/%lu, writes %lu, frame %s\n", csname[d], line_ok ? "ok" : "FAIL",
                   (unsigned long)psram_stats.rd_dev[0], (unsigned long)psram_stats.rd_dev[1], (unsigned long)psram_stats.rd_dev[2],
                   (unsigned long)psram_stats.writes, ok ? "bit-exact" : "WRONG");
            if (!line_ok) bad++;
        }
        model_cs = saved;
        model_install(0);
    }
    if (!bad && !res_ok)
        printf("  -> wiring and bus data are right but the result differs: run '3' (shows the first mismatching frames)\n");
    printf("---- wiring diagnostic %s ----\n", (bad || !res_ok) ? "FAILED" : "PASSED");
    return bad + !res_ok;
}

/* ---------------------------------------------------------------- F5: live microphone */
static void live_start(void) {
    const memcfg_preset_t *p = &PRESETS[preset];
    model_install(0);
    chip_reset();
    chip_config(p->div, p->rd);
    psram_stats_clear();
    chip_wr1(R_CTRL, C_RUN | C_HOP512 | C_CLEAR | (cfg_edge ? C_PDM_EDGE : 0));
    live = true;
    printf("live: PDM_DIV %d (PDM_CLK %.4f MHz, fs %.1f Hz), %s, model in %s, THR %d N %d - speak to the mic on the FPGA\n",
           cfg_pdmdiv, 16.3125 / (2.0 * (cfg_pdmdiv + 1)), 16.3125e6 / (128.0 * (cfg_pdmdiv + 1)), p->name, mem_name[model_cs],
           cfg_thr, cfg_n);
}

static void live_poll(void) {
    static absolute_time_t led_off;
    if (chip_wait_frame(0)) {
        uint8_t r[3];
        chip_rd(R_RESULT, r, 3);
        int top1 = r[0] >> 4, dcls = r[0] & 15, v1 = (int8_t)r[1], v2 = (int8_t)r[2];
        bool det = chip_frame_det();
        if (verbose) printf("%-9s %4d %4d margin %4d%s\n", KWS_CLASS_NAMES[top1], v1, v2, v1 - v2, det ? "  <== DETECT" : "");
        if (det) {
            printf(">>> DETECT  %-6s (class %d, margin %d)\n", KWS_CLASS_NAMES[dcls], dcls, v1 - v2);
            led(true); led_off = make_timeout_time_ms(400);
        }
    }
    if (time_reached(led_off)) led(false);
}

/* ---------------------------------------------------------------- console */
static void help(void) {
    printf("\ncommands:\n"
           "  1  F1 registers (ID, H, defaults, auto-increment)\n"
           "  3  F3 host features -> NN, all %d golden clips, model streamed from the emulated Pmod\n"
           "  a  F3 with the model in flash, PSRAM A and PSRAM B in turn\n"
           "  c  model memory (now %s)\n"
           "  4  F4 host PCM -> front end -> NN, %d golden clips at 16 kHz (~%d s)\n"
           "  w  QSPI read-window sweep (SCK div 7/4/2/1, rd_dly 0..7)\n"
           "  d  QSPI wiring diagnostic (which FPGA SD line reaches which Pico pin)\n"
           "  l  live microphone on/off        v  per-frame output on/off\n"
           "  m  QSPI preset (now: %s)\n"
           "  b  burst 8/16/32/64/128 B (now %d B; PSRAM max 32 B)    e  PDM edge (now %s)\n"
           "  + / -  THR (%d)    ] / [  N (%d)    s  status + PSRAM counters    z  zero counters\n"
           "  g  SCK hold-off on/off (now %s)  r  reset chip      h  help\n",
           M1_CLIPS, mem_name[model_cs], M2_CLIPS, M2_CLIPS + 1, PRESETS[preset].name, 4 << cfg_burst, cfg_edge ? "falling" : "rising", cfg_thr,
           cfg_n, holdoff_on ? "on" : "off - r12 behaviour");
}

static void command(int ch) {
    bool was_live = live;
    live = false;
    switch (ch) {
    case '1': test_regs(); break;
    case '3': test_feat(M1_CLIPS, PRESETS[preset].div, PRESETS[preset].rd, false); break;
    case 'a': test_all_cs(M1_CLIPS); break;
    case 'c': model_cs = (model_cs + 1) % MEM_N; model_install(0); printf("model in %s (MEMCFG.cs = %d)\n", mem_name[model_cs], model_cs); break;
    case '4': test_pcm(M2_CLIPS); break;
    case 'w': sweep(); break;
    case 'd': diag(); break;
    case 'l': if (!was_live) live_start(); else { chip_wr1(R_CTRL, 0); printf("live off\n"); } return;
    case 'v': verbose = !verbose; break;
    case 'm': preset = (preset + 1) % 2; printf("QSPI: %s\n", PRESETS[preset].name); break;
    case 'b': cfg_burst = cfg_burst == 5 ? 1 : cfg_burst + 1;
              printf("burst %d B%s\n", 4 << cfg_burst, cfg_burst > 3 ? " (flash only - the chip caps PSRAM bursts at 32 B)" : ""); break;
    case 'e': cfg_edge = !cfg_edge; printf("PDM edge %s\n", cfg_edge ? "falling" : "rising"); break;
    case '+': if (cfg_thr < 255) cfg_thr++; printf("THR %d\n", cfg_thr); break;
    case '-': if (cfg_thr > 0) cfg_thr--; printf("THR %d\n", cfg_thr); break;
    case ']': if (cfg_n < 15) cfg_n++; printf("N %d\n", cfg_n); break;
    case '[': if (cfg_n > 1) cfg_n--; printf("N %d\n", cfg_n); break;
    case 's': print_status(); break;
    case 'z': psram_stats_clear(); psram_tx_stalled(); break;
    case 'g': holdoff_on = !holdoff_on;
              printf("SCK hold-off %s (applies from the next test: %u cycles at the current preset)\n", holdoff_on ? "ON" : "OFF",
                     holdoff_on ? psram_holdoff_for_div(PRESETS[preset].div, SYS_KHZ) : 0u); break;
    case 'r': chip_reset(); printf("chip reset\n"); break;
    case 'h': case '?': printf("build %s\n", FW_TAG); help(); break;
    default: break;
    }
    if (was_live && ch != 'l') {
        if (ch == '+' || ch == '-' || ch == ']' || ch == '[') {      /* detector settings apply live */
            chip_wr1(R_THR, (uint8_t)cfg_thr); chip_wr1(R_DETCFG, (uint8_t)((cfg_nfill << 4) | cfg_n)); live = true;
        } else if (ch == 'v' || ch == 's' || ch == 'z' || ch == 'h' || ch == '?') {
            live = true;
        } else {
            live_start();                                            /* tests reconfigured the chip */
        }
    }
}

int main(void) {
    if (SYS_KHZ != 150000) set_sys_clock_khz(SYS_KHZ, true);
    stdio_init_all();
    wifi_ok = cyw43_arch_init() == 0;
    model_install(0);
    psram_start();
    chip_init();
    for (int i = 0; i < 40 && !stdio_usb_connected(); i++) { led((i / 5) % 2 == 0 && i < 30); sleep_ms(100); }
    led(false);
    printf("\n=== kws_fpga_host: tiny_kws rev B on iCESugar + Pico 2 W PSRAM (sys %lu MHz) ===\n=== build %s ===\n",
           (unsigned long)(clock_get_hz(clk_sys) / 1000000), FW_TAG);
    printf("TT QSPI Pmod emulated: flash CS0 GP7, PSRAM A CS1 GP8, PSRAM B CS2 GP9, SD0-3 GP10-13, SCK GP14"
           " (model %d B at 0x000000 of the selected memory, md5 %s; h' scratch PSRAM B 0x7FFF00); host SPI0 %d kHz\n",
           KWS_MODEL_BYTES, KWS_MODEL_MD5, SPI_HZ / 1000);
    {   /* the FPGA may still be loading its bitstream (both boards powered together): wait for the chip ID */
        int i = 0;
        for (; i < 50; i++) { chip_reset(); if (chip_rd1(R_ID) == 0x4B) break; if (!i) printf("waiting for the FPGA (chip ID 0x4B)...\n"); sleep_ms(100); }
        if (i == 50) printf("  no chip ID after 5 s: FPGA not programmed / not powered, or host SPI wiring (PMOD3)\n");
    }
    int e = test_regs();
    if (!e) e += test_all_cs(M1_CLIPS);
    if (e) { printf("F3 failed - running the QSPI wiring diagnostic:\n"); diag(); }
    help();
    while (true) {
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) command(ch);
        if (live) live_poll();
    }
}
