/* QSPI PSRAM emulator - see psram_emu.h and qspi_psram.pio.
 *
 * PIO0: SM0 qspi_rx (command/address), SM1 qspi_tx (read data).
 * PIO1: two qspi_cs watchers (CS rising edge held ~100 ns -> DMA writes 'jmp start' into PIO0
 * SM0 / SM1 INSTR), plus the diagnostic sniffer and raw CS counter.  Core 1 runs psram_server() from RAM:
 * for each read it queues exactly the burst's data words (<= 8 words = 32 B, the joined 8-deep
 * TX FIFO holds a whole burst), for each write it stores the byte.
 */
#include "psram_emu.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "qspi_psram.pio.h"

#define LO_SIZE   0x10000u
#define HI_BASE   0x7FFF00u

static uint8_t mem_lo[LO_SIZE] __attribute__((aligned(4)));
static uint8_t mem_hi[256] __attribute__((aligned(4)));

psram_stats_t psram_stats;
volatile uint32_t psram_trace[PSRAM_TRACE_N];
volatile uint32_t psram_trace_n = PSRAM_TRACE_N;
volatile uint8_t psram_burst = 3;

static PIO pio = pio0;
static const uint SM_RX = 0, SM_TX = 1;
static uint off_rx_g, off_tx_g;
static PIO pio_s = pio1;                       /* CS watchers + diagnostics */
static uint sm_s, off_s, sm_raw, off_raw;
static int dma_s = -1;
static uint8_t sniff_buf[SNIFF_N];

static inline uint8_t *mem_ptr(uint32_t a) {
    if (a < LO_SIZE) return &mem_lo[a];
    if (a >= HI_BASE && a < HI_BASE + sizeof mem_hi) return &mem_hi[a - HI_BASE];
    return 0;
}

void psram_load(uint32_t addr, const uint8_t *src, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) { uint8_t *p = mem_ptr(addr + i); if (p) *p = src[i]; }
}

uint8_t psram_peek(uint32_t addr) { uint8_t *p = mem_ptr(addr); return p ? *p : 0; }

void psram_stats_clear(void) {
    psram_stats.reads = psram_stats.writes = psram_stats.read_bytes = psram_stats.stale_flush = 0;
    psram_stats.bad_cmd = psram_stats.out_of_range = psram_stats.tx_late = 0;
    psram_stats.writes_model_area = psram_stats.last_bad_waddr = 0;
}

int psram_tx_stalled(void) {
    uint32_t m = 1u << (PIO_FDEBUG_TXSTALL_LSB + SM_TX);
    int s = (pio->fdebug & m) != 0;
    pio->fdebug = m;                                   /* write 1 to clear */
    return s;
}

/* ---------------------------------------------------------------- core 1 */
static void __not_in_flash_func(psram_server)(void) {
    io_ro_32 *rxf = &pio->rxf[SM_RX];
    io_wo_32 *txf = &pio->txf[SM_TX];
    uint32_t wr_addr = 0;
    bool wr_pending = false;
    for (;;) {
        if (pio->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + SM_RX))) continue;
        uint32_t w = *rxf;
        if (psram_trace_n < PSRAM_TRACE_N) psram_trace[psram_trace_n++] = w;
        uint32_t cmd = w >> 24, addr = w & 0xFFFFFFu;
        if (cmd == 0xEB) {
            /* words left over from a burst the chip cut short: flush before queuing this one */
            if (!(pio->fstat & (1u << (PIO_FSTAT_TXEMPTY_LSB + SM_TX)))) {
                pio_sm_clear_fifos(pio, SM_TX);
                psram_stats.stale_flush++;
            }
            uint32_t b = psram_burst & 7u; if (b > 3) b = 3;            /* PSRAM: <= 32 B (kws_qspi bmax) */
            uint32_t blen = 4u << b, bcnt = blen - (addr & (blen - 1u)); /* bytes to the aligned boundary */
            const uint8_t *p = mem_ptr(addr);
            if (!p) { psram_stats.out_of_range++; }
            for (uint32_t i = 0; i < bcnt; i += 4) {
                uint32_t v = 0;
                for (uint32_t k = 0; k < 4; k++) v = (v << 8) | ((p && i + k < bcnt) ? p[i + k] : 0u);
                *txf = v;                                               /* high nibble of byte 0 first */
            }
            psram_stats.reads++;
            psram_stats.read_bytes += bcnt;
        } else if (cmd == 0x38) {
            wr_addr = addr; wr_pending = true;
        } else if (cmd == 0x00) {                    /* second word: write data, or a read's mode bits */
            if (wr_pending) {
                uint8_t *p = mem_ptr(wr_addr);
                if (p) *p = (uint8_t)w; else psram_stats.out_of_range++;
                if (wr_addr < HI_BASE) { psram_stats.writes_model_area++; psram_stats.last_bad_waddr = wr_addr; }
                psram_stats.writes++;
                wr_pending = false;
            }
        } else {
            psram_stats.bad_cmd++;
            wr_pending = false;
        }
    }
}

/* ---------------------------------------------------------------- set-up */
static uint cs_watch_init(uint off, uint target_sm, uint32_t opcode) {
    uint sm = pio_claim_unused_sm(pio_s, true);
    pio_sm_config c = qspi_cs_program_get_default_config(off);
    sm_config_set_in_pins(&c, PSRAM_PIN_SD0);
    sm_config_set_jmp_pin(&c, PSRAM_PIN_CS);
    pio_sm_init(pio_s, sm, off, &c);
    pio_sm_put_blocking(pio_s, sm, opcode);            /* X = opcode */
    pio_sm_exec(pio_s, sm, pio_encode_pull(false, true));
    pio_sm_exec(pio_s, sm, pio_encode_mov(pio_x, pio_osr));
    int ch = dma_claim_unused_channel(true);
    dma_channel_config d = dma_channel_get_default_config(ch);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
    channel_config_set_read_increment(&d, false);
    channel_config_set_write_increment(&d, false);
    channel_config_set_dreq(&d, pio_get_dreq(pio_s, sm, false));
    channel_config_set_high_priority(&d, true);
    dma_channel_configure(ch, &d, &pio->sm[target_sm].instr, &pio_s->rxf[sm], dma_encode_endless_transfer_count(), true);
    return sm;
}

void psram_start(void) {
    for (uint p = PSRAM_PIN_SD0; p <= PSRAM_PIN_CS; p++) pio_gpio_init(pio, p);
    gpio_pull_up(PSRAM_PIN_CS);                         /* the chip releases CS while CTRL.RUN = 0 */
    gpio_pull_down(PSRAM_PIN_SCK);
    pio_sm_set_consecutive_pindirs(pio, SM_RX, PSRAM_PIN_SD0, 6, false);

    pio_sm_claim(pio, SM_RX);
    pio_sm_claim(pio, SM_TX);
    uint off_rx = pio_add_program(pio, &qspi_rx_program);
    uint off_tx = pio_add_program(pio, &qspi_tx_program);
    off_rx_g = off_rx; off_tx_g = off_tx;
    uint off_cs = pio_add_program(pio_s, &qspi_cs_program);

    pio_sm_config c = qspi_rx_program_get_default_config(off_rx);
    sm_config_set_in_pins(&c, PSRAM_PIN_SD0);
    sm_config_set_in_shift(&c, false, true, 32);       /* shift left, autopush 32 */
    pio_sm_init(pio, SM_RX, off_rx + qspi_rx_offset_start, &c);

    c = qspi_tx_program_get_default_config(off_tx);
    sm_config_set_in_pins(&c, PSRAM_PIN_SD0);
    sm_config_set_out_pins(&c, PSRAM_PIN_SD0, 4);
    sm_config_set_set_pins(&c, PSRAM_PIN_SD0, 4);      /* 'set pindirs' drives / releases SD0..3 */
    sm_config_set_out_shift(&c, false, false, 32);     /* shift left, NO autopull (explicit pull per word) */
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);     /* 8-word TX FIFO = one 32-byte burst */
    pio_sm_init(pio, SM_TX, off_tx + qspi_tx_offset_start, &c);
    pio_sm_set_pins_with_mask(pio, SM_TX, 0, 0xFu << PSRAM_PIN_SD0);

    uint sm_csa = cs_watch_init(off_cs, SM_RX, pio_encode_jmp(off_rx + qspi_rx_offset_start));
    uint sm_csb = cs_watch_init(off_cs, SM_TX, pio_encode_jmp(off_tx + qspi_tx_offset_start));

    /* raw CS counter (diagnostic) */
    sm_raw = pio_claim_unused_sm(pio_s, true);
    off_raw = pio_add_program(pio_s, &qspi_csraw_program);
    pio_sm_config r = qspi_csraw_program_get_default_config(off_raw);
    sm_config_set_in_pins(&r, PSRAM_PIN_SD0);
    sm_config_set_fifo_join(&r, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio_s, sm_raw, off_raw, &r);

    psram_tx_stalled();
    multicore_launch_core1(psram_server);
    pio_set_sm_mask_enabled(pio_s, (1u << sm_csa) | (1u << sm_csb) | (1u << sm_raw), true);
    pio_set_sm_mask_enabled(pio, (1u << SM_RX) | (1u << SM_TX), true);
}

/* ---------------------------------------------------------------- SCK hold-off */
/* rewrite the [delay] of the hold-off instructions (public h_* labels in qspi_psram.pio); these are
 * wait / in / set instructions (no address field), so the assembled opcode is position independent */
static unsigned holdoff_g;
void psram_set_holdoff(unsigned h) {
    if (h > 31) h = 31;
    static const uint8_t rx_at[] = {qspi_rx_offset_h_r0, qspi_rx_offset_h_r1, qspi_rx_offset_h_r2, qspi_rx_offset_h_r3};
    static const uint8_t tx_at[] = {qspi_tx_offset_h_t0, qspi_tx_offset_h_t1, qspi_tx_offset_h_t2, qspi_tx_offset_h_t3};
    for (unsigned i = 0; i < 4; i++) {
        pio->instr_mem[off_rx_g + rx_at[i]] = (qspi_rx_program_instructions[rx_at[i]] & ~0x1F00u) | (h << 8);
        pio->instr_mem[off_tx_g + tx_at[i]] = (qspi_tx_program_instructions[tx_at[i]] & ~0x1F00u) | (h << 8);
    }
    holdoff_g = h;
}
unsigned psram_holdoff(void) { return holdoff_g; }

/* ---------------------------------------------------------------- diagnostics */
void psram_sm_state(unsigned *rx_pc, unsigned *tx_pc, unsigned *tx_level, unsigned *rx_level, uint32_t *fdebug) {
    *rx_pc = pio_sm_get_pc(pio, SM_RX) - off_rx_g;
    *tx_pc = pio_sm_get_pc(pio, SM_TX) - off_tx_g;
    *tx_level = pio_sm_get_tx_fifo_level(pio, SM_TX);
    *rx_level = pio_sm_get_rx_fifo_level(pio, SM_RX);
    *fdebug = pio->fdebug;
}

unsigned psram_tx_start_pc(void) { return qspi_tx_offset_start; }
unsigned psram_off_rx(void) { return off_rx_g; }
unsigned psram_off_tx(void) { return off_tx_g; }

void psram_csraw_clear(void) { pio_sm_clear_fifos(pio_s, sm_raw); }
unsigned psram_csraw_count(void) { return pio_sm_get_rx_fifo_level(pio_s, sm_raw); }   /* saturates at 8 */

void psram_sniff_arm(void) {
    if (dma_s < 0) {
        sm_s = pio_claim_unused_sm(pio_s, true);      /* PIO1: 2 watchers + raw counter + sniffer = 4 SMs */
        off_s = pio_add_program(pio_s, &qspi_sniff_program);
        dma_s = dma_claim_unused_channel(true);
    }
    pio_sm_set_enabled(pio_s, sm_s, false);
    dma_channel_abort(dma_s);
    pio_sm_config c = qspi_sniff_program_get_default_config(off_s);
    sm_config_set_in_pins(&c, PSRAM_PIN_SD0);
    sm_config_set_in_shift(&c, false, true, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio_s, sm_s, off_s, &c);
    for (int i = 0; i < SNIFF_N; i++) sniff_buf[i] = 0;
    dma_channel_config d = dma_channel_get_default_config(dma_s);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_8);       /* low byte of each RX FIFO word */
    channel_config_set_read_increment(&d, false);
    channel_config_set_write_increment(&d, true);
    channel_config_set_dreq(&d, pio_get_dreq(pio_s, sm_s, false));
    dma_channel_configure(dma_s, &d, sniff_buf, &pio_s->rxf[sm_s], SNIFF_N, true);
    pio_sm_set_enabled(pio_s, sm_s, true);
}

int psram_sniff_read(const uint8_t **samples) {
    uint32_t left = dma_channel_hw_addr(dma_s)->transfer_count;
    int n = SNIFF_N - (int)left;
    *samples = sniff_buf;
    return n;                                   /* keeps running; psram_sniff_arm() restarts it */
}
