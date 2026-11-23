/* tiny_kws integer GRU + classifier + detector, bit-exact with model/gru_int.py.
 * Right shifts of negative int32 are arithmetic (floor) on GCC/ARM, as numpy >> and Verilog >>>. */
#include "kws_nn.h"

static inline int32_t sat(int64_t v, int w) {
    const int64_t hi = ((int64_t)1 << (w - 1)) - 1, lo = -((int64_t)1 << (w - 1));
    return (int32_t)(v > hi ? hi : (v < lo ? lo : v));
}
static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

int kws_model_parse(kws_model_t *m, const uint8_t *b, int len) {
    if (len < 8 || b[0] != 0xA5) return -1;                       /* bad magic */
    m->C = b[1]; m->XOFF = b[2]; m->Sr = b[3]; m->Sz = b[4]; m->Sn = b[5]; m->Sfc = b[6]; m->H = b[7];
    if (m->H < 1 || m->H > KWS_HMAX || m->C < 1 || m->C > KWS_CMAX) return -2;
    m->D = KWS_F + m->H;
    int need = 8 + m->H * (3 * m->D + 12) + m->C * (m->H + 3);
    if (len != need) return -3;                                    /* size mismatch */
    int o = 8;
#define RD24() (o += 3, (int32_t)(((uint32_t)b[o - 3] | (uint32_t)b[o - 2] << 8 | (uint32_t)b[o - 1] << 16) << 8) >> 8)
    for (int j = 0; j < m->H; j++) {
        m->Br[j]  = RD24(); m->Wr[j] = (const int8_t *)(b + o); o += m->D;
        m->Bz[j]  = RD24(); m->Wz[j] = (const int8_t *)(b + o); o += m->D;
        m->Bnx[j] = RD24(); m->Bnh[j] = RD24(); m->Wn[j] = (const int8_t *)(b + o); o += m->D;
    }
    for (int c = 0; c < m->C; c++) { m->Bfc[c] = RD24(); m->Wfc[c] = (const int8_t *)(b + o); o += m->H; }
#undef RD24
    return o == len ? 0 : -3;
}

void kws_gru_reset(kws_gru_t *g, const kws_model_t *m) {
    g->m = m;
    for (int i = 0; i < KWS_HMAX; i++) g->h[i] = 0;
    for (int i = 0; i < KWS_CMAX; i++) g->logit[i] = 0;
    g->top1 = 0; g->v1 = 0; g->v2 = 0;
}

static inline int32_t dot(const int8_t *w, const int32_t *v, int n) {
    int32_t a = 0;                                   /* |sum| < 2^21: no overflow before saturation */
    for (int i = 0; i < n; i++) a += (int32_t)w[i] * v[i];
    return a;
}

void kws_gru_step(kws_gru_t *g, const uint8_t *feat) {
    const kws_model_t *m = g->m;
    const int F = KWS_F, H = m->H, D = m->D;
    int32_t v[KWS_F + KWS_HMAX], hn[KWS_HMAX];
    for (int i = 0; i < F; i++) v[i] = clampi((int32_t)feat[i] - m->XOFF, -128, 127);
    for (int i = 0; i < H; i++) v[F + i] = g->h[i];
    for (int j = 0; j < H; j++) {
        int32_t ar = sat((int64_t)m->Br[j] + dot(m->Wr[j], v, D), KWS_ACC_W);
        int32_t az = sat((int64_t)m->Bz[j] + dot(m->Wz[j], v, D), KWS_ACC_W);
        int32_t ax = sat((int64_t)m->Bnx[j] + dot(m->Wn[j], v, F), KWS_ACC_W);
        int32_t ah = sat((int64_t)m->Bnh[j] + dot(m->Wn[j] + F, v + F, H), KWS_ACC_W);
        int32_t r7 = clampi(sat(ar >> m->Sr, 8) + 64, 0, 128);
        int32_t z7 = clampi(sat(az >> m->Sz, 8) + 64, 0, 128);
        int32_t t  = (ax >> m->Sn) + ((r7 * sat(ah >> m->Sn, 8)) >> 7);
        int32_t n8 = clampi(t, -128, 127);
        hn[j] = n8 + ((z7 * (g->h[j] - n8)) >> 7);
    }
    for (int j = 0; j < H; j++) g->h[j] = hn[j];
    for (int c = 0; c < m->C; c++)
        g->logit[c] = sat(sat((int64_t)m->Bfc[c] + dot(m->Wfc[c], hn, H), KWS_ACC_W) >> m->Sfc, 8);
    int i1 = 0;                                       /* lowest index wins ties */
    for (int c = 1; c < m->C; c++) if (g->logit[c] > g->logit[i1]) i1 = c;
    int v2 = -128;
    for (int c = 0; c < m->C; c++) if (c != i1 && g->logit[c] > v2) v2 = g->logit[c];
    g->top1 = i1; g->v1 = g->logit[i1]; g->v2 = v2;
}

void kws_det_init(kws_det_t *d, int thr, int n, int nfill) {
    d->thr = thr; d->n = n; d->nfill = nfill; d->det_class = 0;
    kws_det_reset(d);
}

void kws_det_reset(kws_det_t *d) { d->last = 0; d->run = 0; d->det = 0; }

int kws_det_step(kws_det_t *d, int i, int t1, int t2) {
    int kw   = i >= d->nfill && (t1 - t2) >= d->thr;
    int inc  = (i == d->last) ? (d->run + 1 > 15 ? 15 : d->run + 1) : 1;
    int req  = d->n ? d->n : 1;
    int fire = kw && inc == req && !(i == d->last && d->run == 15);
    d->run  = kw ? inc : 0;
    d->last = i;
    d->det  = fire;
    if (fire) d->det_class = i;
    return fire;
}
