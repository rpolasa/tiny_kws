#ifndef KWS_SELFTEST_H
#define KWS_SELFTEST_H
#include <stdint.h>
#include "kws_nn.h"

uint32_t kws_time_us(void);                 /* provided by the platform (Pico timer or host clock) */

/* each returns the number of mismatches (0 = bit-exact with the golden model) */
int kws_test_m1(const kws_model_t *m);      /* golden features -> logits, top1/top2, DETECT */
int kws_test_m2(const kws_model_t *m);      /* golden PCM -> features (-> NN, end to end) */
int kws_test_m3(void);                      /* golden PDM bits -> CIC3 -> PCM -> features */
int kws_test_all(const kws_model_t *m);

#endif
