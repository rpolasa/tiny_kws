/* Tiny Tapeout QSPI Pmod emulator (core 1 + PIO0 + PIO1 + DMA), r15: answers the tiny_kws chip's
 * 0xEB quad reads on all three chip selects and its 0x38 quad writes, like the TT QSPI Pmod:
 *   CS0  W25Q128 flash   (model; bursts up to 128 B)
 *   CS1  APS6404L PSRAM A (model; bursts up to 32 B)
 *   CS2  APS6404L PSRAM B (model and/or the h' scratch the chip writes and reads back; bursts up to 32 B)
 * Memory kept for each: a 64 KB model window at 0x000000; PSRAM B also 0x7FFF00-0x7FFFFF (h' scratch).
 * Reads elsewhere return 0 (counted as out_of_range), writes elsewhere are ignored.
 */
#ifndef PSRAM_EMU_H
#define PSRAM_EMU_H
#include <stdint.h>

#define PSRAM_PIN_CS0  7       /* chip uio[0] CS0 (flash)   - CS0..CS2 must be consecutive GPIOs */
#define PSRAM_PIN_CS1  8       /* chip uio[6] CS1 (PSRAM A) */
#define PSRAM_PIN_CS2  9       /* chip uio[7] CS2 (PSRAM B) */
#define PSRAM_PIN_SD0  10      /* SD0..SD3 = GP10..GP13 */
#define PSRAM_PIN_SCK  14

enum { MEM_FLASH = 0, MEM_RAMA = 1, MEM_RAMB = 2, MEM_N = 3 };
#define MEM_WIN        0x10000u    /* model window of each memory */
#define MEM_SCRATCH    0x7FFF00u   /* PSRAM B h' scratch (256 B) */
extern const char *const mem_name[MEM_N];

typedef struct {
    volatile uint32_t reads, writes, read_bytes, stale_flush, bad_cmd, out_of_range, tx_late;
    volatile uint32_t writes_model_area, last_bad_waddr;   /* PSRAM B writes that landed below 0x7FFF00 */
    volatile uint32_t rd_dev[MEM_N];                       /* reads served per chip select */
    volatile uint32_t wr_not_ramb;                         /* writes on CS0/CS1 (the chip only writes CS2) */
    volatile uint32_t cs_bad;                              /* command word with no / several CS low */
    volatile uint32_t hdr_abort, burst_abort;              /* transactions cut short in the header / data */
} psram_stats_t;

extern psram_stats_t psram_stats;
extern volatile uint8_t psram_burst;     /* copy of the chip's BURST register (sets the burst length) */

void    mem_load(int dev, uint32_t addr, const uint8_t *src, uint32_t n);
void    mem_fill(int dev, uint8_t v);    /* the whole 64 KB model window */
uint8_t mem_peek(int dev, uint32_t addr);
void psram_start(void);                  /* PIO + DMA set-up, then launches core 1 */
void psram_rearm(void);                  /* reset the bus state machines (call while the chip is in reset) */
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

/* diagnostic trace: the first PSRAM_TRACE_N transactions (cmd << 24 | addr) and the memory (CS) of each */
#define PSRAM_TRACE_N 1024
extern volatile uint32_t psram_trace[PSRAM_TRACE_N];
extern volatile uint8_t  psram_trace_dev[PSRAM_TRACE_N];
extern volatile uint32_t psram_trace_n;
static inline void psram_trace_arm(void) { psram_trace_n = 0; }

/* bus sniffer on PIO1: one sample of GP7..GP14 per SCK rising edge from the moment it is armed:
 *   bit 0..2 CS0..CS2, bit 3..6 SD0..SD3, bit 7 SCK */
#define SNIFF_N 32768
#define SNIFF_SD(s)  (((s) >> 3) & 15u)
#define SNIFF_CS(s)  ((s) & 7u)
void psram_sniff_arm(void);
int  psram_sniff_read(const uint8_t **samples);   /* number of samples captured (buffer owned here) */
/* data / command state-machine state, for the diagnostic */
void psram_sm_state(unsigned *rx_pc, unsigned *tx_pc, unsigned *tx_level, unsigned *rx_level, uint32_t *fdebug);
unsigned psram_tx_start_pc(void);
unsigned psram_off_rx(void);
unsigned psram_off_tx(void);
/* raw end-of-transaction count (all three CS high again, glitches included) since psram_csraw_clear(),
 * saturating at 8 */
void psram_csraw_clear(void);
unsigned psram_csraw_count(void);

#endif
