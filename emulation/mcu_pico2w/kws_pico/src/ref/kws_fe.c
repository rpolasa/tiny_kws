/* tiny_kws bit-exact front-end golden model.
 *
 * PCM path : int16 samples -> 16ch IIR band-pass -> |y| accumulate per hop -> log2 (u8)
 * PDM path : 1-bit PDM @ 64*fs -> CIC4 (R=64) -> int16 -> same as PCM path
 *
 * Every arithmetic step here is mirrored 1:1 in src/kws_frontend.v.
 * Build: gcc -O2 -shared -fPIC -o libkwsfe.so kws_fe.c
 */
#include <stdint.h>
#include <string.h>
#include "fbank_coeffs.h"

/* ---- frozen fixed-point parameters (see docs/ARCHITECTURE.md) ---- */
#ifndef FE_YF
#define FE_YF   1        /* fractional bits kept in filter state y            */
#endif
#ifndef FE_WY
#define FE_WY   17       /* signed width of y state                            */
#endif
#ifndef FE_ESH
#define FE_ESH  4        /* |y| >> ESH before accumulation                     */
#endif
#ifndef FE_WE
#define FE_WE   20       /* unsigned width of energy accumulator (saturating)  */
#endif

typedef struct {
    int32_t x1, x2;                 /* shared input delay line */
    int32_t y1[FB_NCH], y2[FB_NCH]; /* per channel state        */
    uint32_t e[FB_NCH];             /* energy accumulators      */
    int32_t  cnt;                   /* samples in current hop   */
    /* CIC3 state (19-bit modular) */
    uint32_t ci[3], cc[3];
    int32_t  pdm_cnt;
} fe_state_t;

static inline int64_t sat_s(int64_t v, int w) {
    int64_t mx = ((int64_t)1 << (w - 1)) - 1, mn = -((int64_t)1 << (w - 1));
    return v > mx ? mx : (v < mn ? mn : v);
}

/* log2 compressor: msb position p (0..31) and 3 mantissa bits -> p*8+m.  e==0 -> 0 */
uint8_t fe_log2u8(uint32_t e) {
    if (e == 0) return 0;
    int p = 31; while (!((e >> p) & 1)) p--;
    uint32_t m = (p >= 3) ? ((e >> (p - 3)) & 7) : ((e << (3 - p)) & 7);
    int v = p * 8 + (int)m;
    return (uint8_t)(v > 255 ? 255 : v);
}

void fe_reset(fe_state_t *s) { memset(s, 0, sizeof(*s)); }
int  fe_state_size(void) { return (int)sizeof(fe_state_t); }

/* process one int16 sample; returns 1 and writes NCH features when a hop completes */
int fe_sample(fe_state_t *s, int32_t x0, int hop, uint8_t *feat) {
    int32_t d = x0 - s->x2;                        /* 17-bit signed */
    for (int k = 0; k < FB_NCH; k++) {
        int64_t acc = ((int64_t)FB_A[k] * d) * (1 << FE_YF);
        acc += (int64_t)FB_C1[k] * s->y1[k];
        acc -= (int64_t)FB_C2[k] * s->y2[k];
        acc += (int64_t)1 << (FB_CF - 1);          /* round half up */
        int32_t y = (int32_t)sat_s(acc >> FB_CF, FE_WY);
        s->y2[k] = s->y1[k];
        s->y1[k] = y;
        uint32_t a = (uint32_t)(y < 0 ? -y : y) >> FE_ESH;
        uint64_t e = (uint64_t)s->e[k] + a;
        uint64_t emax = ((uint64_t)1 << FE_WE) - 1;
        s->e[k] = (uint32_t)(e > emax ? emax : e);
    }
    s->x2 = s->x1; s->x1 = x0;
    if (++s->cnt == hop) {
        for (int k = 0; k < FB_NCH; k++) { feat[k] = fe_log2u8(s->e[k]); s->e[k] = 0; }
        s->cnt = 0;
        return 1;
    }
    return 0;
}

/* run a whole PCM buffer. returns number of frames written to feats (nfr*NCH) */
int fe_run_pcm(fe_state_t *s, const int16_t *x, int n, int hop, uint8_t *feats, int maxfr) {
    int nf = 0;
    for (int i = 0; i < n; i++)
        if (fe_sample(s, x[i], hop, feats + nf * FB_NCH)) { if (++nf >= maxfr) break; }
    return nf;
}

/* batch helper for training: N clips of length L each, fresh state per clip */
void fe_batch_pcm(const int16_t *x, int N, int L, int hop, uint8_t *feats) {
    fe_state_t s; int nfr = L / hop;
    for (int i = 0; i < N; i++) {
        fe_reset(&s);
        fe_run_pcm(&s, x + (int64_t)i * L, L, hop, feats + (int64_t)i * nfr * FB_NCH, nfr);
    }
}

/* CIC3, R=64, 19-bit modular arithmetic, pipelined integrators (register semantics).
 * bits: one PDM bit per byte (0/1). returns number of int16 samples produced. */
#define CIC_W 19
#define CIC_M ((1u << CIC_W) - 1)
int fe_cic_pdm(fe_state_t *s, const uint8_t *bits, int nbits, int16_t *out) {
    int no = 0;
    for (int i = 0; i < nbits; i++) {
        uint32_t in = bits[i] ? 1u : (CIC_M);       /* +1 / -1 */
        uint32_t i0 = s->ci[0], i1 = s->ci[1];
        s->ci[0] = (s->ci[0] + in) & CIC_M;
        s->ci[1] = (s->ci[1] + i0) & CIC_M;
        s->ci[2] = (s->ci[2] + i1) & CIC_M;
        if (++s->pdm_cnt == 64) {
            s->pdm_cnt = 0;
            uint32_t v = s->ci[2], t;
            t = (v - s->cc[0]) & CIC_M; s->cc[0] = v; v = t;
            t = (v - s->cc[1]) & CIC_M; s->cc[1] = v; v = t;
            t = (v - s->cc[2]) & CIC_M; s->cc[2] = v; v = t;
            int32_t sv = (int32_t)(v << (32 - CIC_W)) >> (32 - CIC_W);   /* sign extend */
            out[no++] = (int16_t)(sv >> 3);                               /* 2^18 full scale -> 2^15 */
        }
    }
    return no;
}
