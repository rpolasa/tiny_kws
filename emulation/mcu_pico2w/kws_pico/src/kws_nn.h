/* tiny_kws integer GRU + classifier + detector - C port of model/gru_int.py and
 * test/golden.py::detect_seq (== src/kws_nn.v). Bit-exact; no floating point. */
#ifndef KWS_NN_H
#define KWS_NN_H
#include <stdint.h>

#define KWS_F      16      /* features per frame (front-end channels) */
#define KWS_HMAX   64
#define KWS_CMAX   16
#define KWS_ACC_W  26      /* accumulator width (signed, saturating) */

typedef struct {
    int C, H, D, XOFF, Sr, Sz, Sn, Sfc;
    /* weights point into the blob (int8, row-major, D or H per row) */
    const int8_t *Wr[KWS_HMAX], *Wz[KWS_HMAX], *Wn[KWS_HMAX], *Wfc[KWS_CMAX];
    int32_t Br[KWS_HMAX], Bz[KWS_HMAX], Bnx[KWS_HMAX], Bnh[KWS_HMAX], Bfc[KWS_CMAX];
} kws_model_t;

typedef struct {
    const kws_model_t *m;
    int32_t h[KWS_HMAX];          /* hidden state, int8 range (h_real = h/128) */
    int32_t logit[KWS_CMAX];      /* last frame's logits (int8 range) */
    int top1, v1, v2;             /* top-1 index, top-1 and top-2 logit values */
} kws_gru_t;

typedef struct {
    int thr, n, nfill;            /* THR register, DETCFG[3:0] = N, DETCFG[7:4] = NFILL */
    int last, run;                /* RTL last_idx / run_cnt */
    int det, det_class;           /* DETECT pin (this frame), DET_CLASS pins (last detection) */
} kws_det_t;

/* parse a model blob (format of gru_int.pack_blob). returns 0 or a negative error code */
int  kws_model_parse(kws_model_t *m, const uint8_t *blob, int len);
void kws_gru_reset(kws_gru_t *g, const kws_model_t *m);
/* one 32 ms frame: feat[16] -> new hidden state, logits, top1/top2 */
void kws_gru_step(kws_gru_t *g, const uint8_t *feat);

void kws_det_init(kws_det_t *d, int thr, int n, int nfill);
void kws_det_reset(kws_det_t *d);          /* CTRL.CLEAR: last=0, run=0 */
/* returns 1 when DETECT fires for this frame */
int  kws_det_step(kws_det_t *d, int top1, int v1, int v2);

#endif
