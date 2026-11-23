/* Builds the repo's golden front end (src/ref/kws_fe.c, an unchanged copy of model/kws_fe.c)
 * into the firmware, so the Pico runs exactly the code the RTL was verified against. */
#include "kws_fe_port.h"
#include "ref/kws_fe.c"

#if defined(PICO_ON_DEVICE) && PICO_ON_DEVICE
#include "pico.h"
#define HOT(f) __time_critical_func(f)          /* run from SRAM, not XIP flash */
#else
#define HOT(f) f
#endif

static fe_state_t s_fe;        /* filter bank (CIC fields unused here) */
static fe_state_t s_ref;       /* reference CIC state for the self-test */

void kws_fe_reset(void) { fe_reset(&s_fe); }

int kws_fe_sample(int32_t x, uint8_t *feat) { return fe_sample(&s_fe, x, KWS_HOP, feat); }

int kws_cic_ref(const uint8_t *bits, int nbits, int16_t *out, int reset) {
    if (reset) fe_reset(&s_ref);
    return fe_cic_pdm(&s_ref, bits, nbits, out);
}

void kws_cic_reset(kws_cic_t *c) { c->i0 = c->i1 = c->i2 = c->c0 = c->c1 = c->c2 = 0; c->phase = 0; }

/* Same recurrences as fe_cic_pdm: i0' = i0 + in, i1' = i1 + i0, i2' = i2 + i1 (old values),
 * comb every 64 bits.  Kept in 32-bit wrap-around arithmetic and masked to 19 bits only at the
 * output, which is congruent mod 2^19 with the RTL's 19-bit registers. */
int HOT(kws_cic_words)(kws_cic_t *c, const uint32_t *w, int nwords, int16_t *out) {
    uint32_t i0 = c->i0, i1 = c->i1, i2 = c->i2;
    int ph = c->phase, no = 0;
    for (int k = 0; k < nwords; k++) {
        uint32_t word = w[k];
        for (int b = 31; b >= 0; b--) {
            uint32_t in = ((word >> b) & 1u) ? 1u : 0xFFFFFFFFu;      /* +1 / -1 */
            uint32_t o0 = i0, o1 = i1;
            i0 += in; i1 += o0; i2 += o1;
            if (++ph == 64) {
                ph = 0;
                uint32_t v = i2, t;
                t = v - c->c0; c->c0 = v; v = t;
                t = v - c->c1; c->c1 = v; v = t;
                t = v - c->c2; c->c2 = v; v = t;
                int32_t sv = (int32_t)(v << 13) >> 13;                  /* 19-bit sign extend */
                out[no++] = (int16_t)(sv >> 3);
            }
        }
    }
    c->i0 = i0; c->i1 = i1; c->i2 = i2; c->phase = ph;
    return no;
}
