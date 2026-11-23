/* Self-tests M1-M3: the Pico's code against the golden vectors generated from the repo's
 * reference model (tools/gen_headers.py). Same code runs on the PC (host/) and on the Pico. */
#include <stdio.h>
#include <string.h>
#include "kws_selftest.h"
#include "kws_fe_port.h"
#include "gen/kws_model_blob.h"
#include "gen/kws_golden_m1.h"
#include "gen/kws_golden_m2.h"
#include "gen/kws_golden_m3.h"

#define MAXREPORT 5

int kws_test_m1(const kws_model_t *m) {
    kws_gru_t g; kws_det_t d;
    int bad = 0, dets = 0, correct = 0;
    uint32_t tmax = 0, ttot = 0;
    for (int c = 0; c < M1_CLIPS; c++) {
        kws_gru_reset(&g, m);
        kws_det_init(&d, M1_THR, M1_N, M1_NFILL);
        for (int t = 0; t < M1_FRAMES; t++) {
            const int f = c * M1_FRAMES + t;
            uint32_t t0 = kws_time_us();
            kws_gru_step(&g, &M1_FEAT[f * KWS_F]);
            int det = kws_det_step(&d, g.top1, g.v1, g.v2);
            uint32_t dt = kws_time_us() - t0;
            ttot += dt; if (dt > tmax) tmax = dt;
            int ok = g.top1 == M1_TOP[f * 3] && g.v1 == M1_TOP[f * 3 + 1] && g.v2 == M1_TOP[f * 3 + 2]
                     && det == M1_DET[f];
            for (int k = 0; k < m->C; k++) ok &= g.logit[k] == M1_LOGIT[f * m->C + k];
            if (!ok && bad++ < MAXREPORT)
                printf("  M1 clip %d frame %d: got top1=%d %d/%d det=%d, expected %d %d/%d det=%d\n", c, t,
                       g.top1, g.v1, g.v2, det, M1_TOP[f * 3], M1_TOP[f * 3 + 1], M1_TOP[f * 3 + 2], M1_DET[f]);
            dets += det;
        }
        correct += g.top1 == M1_LABEL[c];
    }
    printf("M1 GRU+FC+detector : %d clips x %d frames, %d detections, last-frame top1 = label in %d/%d clips\n",
           M1_CLIPS, M1_FRAMES, dets, correct, M1_CLIPS);
    printf("                     %d us/frame avg, %d us max (frame period 32000 us)  -> %s\n",
           (int)(ttot / (M1_CLIPS * M1_FRAMES)), (int)tmax, bad ? "FAIL" : "PASS (bit-exact)");
    if (bad) printf("                     %d mismatching frames\n", bad);
    return bad;
}

int kws_test_m2(const kws_model_t *m) {
    uint8_t feat[KWS_F];
    kws_gru_t g;
    int bad = 0, nnbad = 0, frames = 0;
    uint32_t ttot = 0;
    for (int c = 0; c < M2_CLIPS; c++) {
        kws_fe_reset();
        kws_gru_reset(&g, m);
        int t = 0;
        for (int i = 0; i < M2_SAMPLES; i++) {
            if (!kws_fe_sample(M2_PCM[c * M2_SAMPLES + i], feat)) continue;
            const uint8_t *exp = &M2_FEAT[(c * M2_FRAMES + t) * KWS_F];
            if (memcmp(feat, exp, KWS_F) && bad++ < MAXREPORT) {
                printf("  M2 clip %d frame %d: features", c, t);
                for (int k = 0; k < KWS_F; k++) printf(" %d/%d", feat[k], exp[k]);
                printf("  (got/expected)\n");
            }
            kws_gru_step(&g, feat);                       /* end to end: PCM -> NN */
            const int f = c * M1_FRAMES + t;
            if (g.top1 != M1_TOP[f * 3] || g.v1 != M1_TOP[f * 3 + 1] || g.v2 != M1_TOP[f * 3 + 2]) nnbad++;
            t++; frames++;
        }
        if (t != M2_FRAMES) { printf("  M2 clip %d: %d frames, expected %d\n", c, t, M2_FRAMES); bad++; }
    }
    /* timing: front end alone over the same clips (no compares) */
    uint32_t t0 = kws_time_us();
    for (int c = 0; c < M2_CLIPS; c++) {
        kws_fe_reset();
        for (int i = 0; i < M2_SAMPLES; i++) kws_fe_sample(M2_PCM[c * M2_SAMPLES + i], feat);
    }
    ttot = kws_time_us() - t0;
    printf("M2 front end (PCM) : %d clips, %d frames, features %s, PCM->NN top1/top2 %s\n", M2_CLIPS, frames,
           bad ? "MISMATCH" : "bit-exact", nnbad ? "MISMATCH" : "bit-exact");
    printf("                     %d us per 1 s of audio (%.1f %% of one core)  -> %s\n",
           (int)(ttot / M2_CLIPS), ttot / (double)M2_CLIPS / 1e4, (bad || nnbad) ? "FAIL" : "PASS");
    return bad + nnbad;
}

static int16_t s_cic[M3_SAMPLES];

int kws_test_m3(void) {
    int bad = 0;
    /* (a) word-wise CIC (the live path) on the packed PIO-format words */
    kws_cic_t c; kws_cic_reset(&c);
    uint32_t t0 = kws_time_us();
    int n = 0;
    for (int k = 0; k < M3_WORDS; k += 64)                 /* in 64-word blocks, as the DMA ring delivers */
        n += kws_cic_words(&c, &M3_PDM[k], (M3_WORDS - k) < 64 ? (M3_WORDS - k) : 64, s_cic + n);
    uint32_t tcic = kws_time_us() - t0;
    int cbad = (n != M3_SAMPLES);
    for (int i = 0; i < n && i < M3_SAMPLES; i++)
        if (s_cic[i] != M3_CIC[i] && cbad++ < MAXREPORT)
            printf("  M3 CIC sample %d: got %d expected %d\n", i, s_cic[i], M3_CIC[i]);
    /* (b) the repo's byte-per-bit fe_cic_pdm on the same words (checks the bit packing) */
    int rbad = 0, rn = 0;
    {
        uint8_t bits[64]; int16_t o[2];
        for (int k = 0; k < M3_WORDS; k++) {
            for (int b = 0; b < 32; b++) bits[b] = (M3_PDM[k] >> (31 - b)) & 1u;
            int got = kws_cic_ref(bits, 32, o, k == 0);
            for (int j = 0; j < got; j++, rn++) if (rn >= M3_SAMPLES || o[j] != M3_CIC[rn]) rbad++;
        }
        if (rn != M3_SAMPLES) rbad++;
    }
    /* (c) features from the CIC samples */
    uint8_t feat[KWS_F];
    int fbad = 0, t = 0;
    kws_fe_reset();
    for (int i = 0; i < n; i++)
        if (kws_fe_sample(s_cic[i], feat)) {
            if (t >= M3_FRAMES || memcmp(feat, &M3_FEAT[t * KWS_F], KWS_F)) fbad++;
            t++;
        }
    if (t != M3_FRAMES) fbad++;
    bad = cbad + rbad + fbad;
    printf("M3 PDM path        : %d bits -> %d samples; CIC (word-wise) %s, CIC (repo fe_cic_pdm) %s, features %s\n",
           M3_WORDS * 32, n, cbad ? "MISMATCH" : "bit-exact", rbad ? "MISMATCH" : "bit-exact",
           fbad ? "MISMATCH" : "bit-exact");
    printf("                     CIC %d us per 1 s of audio (%.1f %% of one core)  -> %s\n",
           (int)tcic, tcic / 1e4, bad ? "FAIL" : "PASS");
    return bad;
}

int kws_test_all(const kws_model_t *m) {
    printf("---- self-test vs golden model (model.bin md5 %s) ----\n", KWS_MODEL_MD5);
    int e = kws_test_m1(m);
    e += kws_test_m2(m);
    e += kws_test_m3();
    printf("---- self-test %s ----\n", e ? "FAILED" : "PASSED: Pico == golden model == RTL, bit for bit");
    return e;
}
