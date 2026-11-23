/* Front end for the Pico: the repo's bit-exact model/kws_fe.c (compiled unchanged, see
 * kws_fe_port.c) plus a word-wise CIC3 that is bit-exact with fe_cic_pdm(). */
#ifndef KWS_FE_PORT_H
#define KWS_FE_PORT_H
#include <stdint.h>

#define KWS_HOP 512            /* CTRL.HOP512: 512 samples = 32 ms at 16 kHz */

typedef struct {               /* CIC3, R = 64, 19-bit modular, pipelined integrators */
    uint32_t i0, i1, i2;       /* integrators (mod 2^32; congruent mod 2^19 with the RTL) */
    uint32_t c0, c1, c2;       /* comb delays */
    int phase;                 /* bits since the last output (0..63) */
} kws_cic_t;

void kws_fe_reset(void);                                   /* filter bank + hop counter */
/* one int16 sample; returns 1 and fills feat[16] when a 512-sample hop completes */
int  kws_fe_sample(int32_t x, uint8_t *feat);

void kws_cic_reset(kws_cic_t *c);
/* PDM words, 32 bits each, first bit in the MSB (PIO 'in' with shift-left + autopush 32).
 * Writes one int16 sample per 64 bits; returns the number of samples written. */
int  kws_cic_words(kws_cic_t *c, const uint32_t *w, int nwords, int16_t *out);

/* reference path (byte per bit, the repo's fe_cic_pdm) - used by the self-test */
int  kws_cic_ref(const uint8_t *bits, int nbits, int16_t *out, int reset);

#endif
