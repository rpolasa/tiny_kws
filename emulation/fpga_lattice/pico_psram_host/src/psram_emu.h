/* QSPI PSRAM emulator (core 1 + PIO0 + 2 DMA channels): answers the tiny_kws chip's
 * 0xEB quad reads and 0x38 quad writes on CS2.  Memory map (as APS6404L PSRAM B):
 *   0x000000-0x00FFFF  model area (64 KB, the model blob is loaded at 0 at start-up)
 *   0x7FFF00-0x7FFFFF  h' scratch (the chip writes and reads back its 48-byte hidden state)
 *   anything else      reads 0, writes ignored (counted as out_of_range)
 */
#ifndef PSRAM_EMU_H
#define PSRAM_EMU_H
#include <stdint.h>

#define PSRAM_PIN_SD0  10      /* SD0..SD3 = GP10..GP13 */
#define PSRAM_PIN_SCK  14
#define PSRAM_PIN_CS   15      /* chip uio[7] = CS2 (RAM B) */

typedef struct {
    volatile uint32_t reads, writes, read_bytes, stale_flush, bad_cmd, out_of_range, tx_late;
    volatile uint32_t writes_model_area, last_bad_waddr;   /* writes that landed below 0x7FFF00 */
} psram_stats_t;

extern psram_stats_t psram_stats;
extern volatile uint8_t psram_burst;     /* copy of the chip's BURST register (sets the burst length) */

void psram_load(uint32_t addr, const uint8_t *src, uint32_t n);
uint8_t psram_peek(uint32_t addr);
void psram_start(void);                  /* PIO + DMA set-up, then launches core 1 */
void psram_stats_clear(void);
int  psram_tx_stalled(void);             /* 1 if the data SM ran dry since the last call (late data) */

/* SCK hold-off (r13): after each SCK edge it acts on (and after switching SD itself) the PIO ignores SCK
 * for h sys cycles, so crosstalk spikes on the SCK wire are not taken for clock edges.  0 = off (r12).
 * Must stay below the SCK half period minus ~5 cycles: psram_holdoff_for_div() gives the setting used. */
void     psram_set_holdoff(unsigned h);  /* 0..31 sys cycles; call while the chip is not running */
unsigned psram_holdoff(void);
static inline unsigned psram_holdoff_for_div(int div, unsigned sys_khz) {
    int h = (7 * (div < 1 ? 1 : div) - 3) * (int)(sys_khz / 1000) / 150;   /* validated at 150 MHz: div 1 -> 4, 2 -> 11 */
    return h < 0 ? 0 : h > 31 ? 31 : (unsigned)h;
}

/* diagnostic trace: the first PSRAM_TRACE_N words received (cmd << 24 | addr, or the 2-nibble word) */
#define PSRAM_TRACE_N 1024
extern volatile uint32_t psram_trace[PSRAM_TRACE_N];
extern volatile uint32_t psram_trace_n;
static inline void psram_trace_arm(void) { psram_trace_n = 0; }

/* bus sniffer on PIO1: records SD0..3/SCK/CS at each SCK rising edge after the next CS2 fall */
#define SNIFF_N 32768
void psram_sniff_arm(void);
int  psram_sniff_read(const uint8_t **samples);   /* number of samples captured (buffer owned here) */
/* data / command state-machine state, for the diagnostic */
void psram_sm_state(unsigned *rx_pc, unsigned *tx_pc, unsigned *tx_level, unsigned *rx_level, uint32_t *fdebug);
unsigned psram_tx_start_pc(void);
unsigned psram_off_rx(void);
unsigned psram_off_tx(void);
/* raw CS2 rising edges (glitches included) since psram_csraw_clear(), saturating at 8 */
void psram_csraw_clear(void);
unsigned psram_csraw_count(void);

#endif
