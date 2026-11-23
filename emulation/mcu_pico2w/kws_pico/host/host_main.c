/* PC build of the same self-tests, to check the C port before flashing:
 *   gcc -O2 -I../src -o kws_host host_main.c ../src/kws_nn.c ../src/kws_fe_port.c ../src/kws_selftest.c
 *   ./kws_host
 */
#include <stdio.h>
#include <time.h>
#include "kws_selftest.h"
#include "gen/kws_model_blob.h"

uint32_t kws_time_us(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000000u + ts.tv_nsec / 1000);
}

int main(void) {
    static kws_model_t m;
    int e = kws_model_parse(&m, KWS_MODEL_BLOB, KWS_MODEL_BYTES);
    if (e) { printf("model parse error %d\n", e); return 2; }
    printf("model: H=%d C=%d XOFF=%d shifts r%d z%d n%d fc%d, %d bytes\n", m.H, m.C, m.XOFF, m.Sr, m.Sz, m.Sn, m.Sfc,
           KWS_MODEL_BYTES);
    return kws_test_all(&m) ? 1 : 0;
}
