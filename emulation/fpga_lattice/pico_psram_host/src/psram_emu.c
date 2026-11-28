/* Tiny Tapeout QSPI Pmod emulator - see psram_emu.h and qspi_psram.pio (r15).
 *
 * PIO0: SM0 qspi_rx (command/address), SM1 qspi_tx (read data), SM2 qspi_csdet (CS fall -> IRQ 4).
 * PIO1: two qspi_cs watchers (all three CS high for ~95 ns -> DMA writes 'jmp start' into PIO0 SM0 /
 * SM1 INSTR), plus the diagnostic raw CS counter and sniffer.
 * Core 1 runs psram_server() from RAM: it reads the CS pins when the command word arrives (the
 * transaction is still running then) to pick the memory, then for each read queues the burst's data
 * words (flash bursts up to 32 words wait for FIFO space), for each write stores the byte.
 */
#include "psram_emu.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/sio.h"
#include "qspi_psram.pio.h"

#define HI_BASE   MEM_SCRATCH
/* command words as qspi_rx captures them: 8 nibbles, bit 0 = SD0 = command bit, bits 3:2 = SD3:SD2 = 11 */
#define CMD_SIG_MASK  0xCCCCCCCCu
#define CMD_BIT_MASK  0x11111111u
#define CMD_EB        0x11101011u     /* 0xEB = 1110 1011 */
#define CMD_38        0x00111000u     /* 0x38 = 0011 1000 */

static uint8_t mem[MEM_N][MEM_WIN] __attribute__((aligned(4)));
static uint8_t mem_hi[256] __attribute__((aligned(4)));
const char *const mem_name[MEM_N] = {"flash (CS0)", "PSRAM A (CS1)", "PSRAM B (CS2)"};

psram_stats_t psram_stats;
volatile uint32_t psram_trace[PSRAM_TRACE_N];
volatile uint8_t psram_trace_dev[PSRAM_TRACE_N];
volatile uint32_t psram_trace_n = PSRAM_TRACE_N;
volatile uint8_t psram_burst = 3;

static PIO pio = pio0;
static const uint SM_RX = 0, SM_TX = 1, SM_DET = 2;
static uint off_rx_g, off_tx_g, off_det_g;
static PIO pio_s = pio1;                       /* end-of-transaction watchers + diagnostics */
static uint sm_s, off_s, sm_raw, off_raw;
static int dma_s = -1;
static uint8_t sniff_buf[SNIFF_N];

static inline uint8_t *mem_ptr(int dev, uint32_t a) {
    if (dev < 0 || dev >= MEM_N) return 0;
    if (a < MEM_WIN) return &mem[dev][a];
    if (dev == MEM_RAMB && a >= HI_BASE && a < HI_BASE + sizeof mem_hi) return &mem_hi[a - HI_BASE];
    return 0;
}

void mem_load(int dev, uint32_t addr, const uint8_t *src, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) { uint8_t *p = mem_ptr(dev, addr + i); if (p) *p = src[i]; }
}

void mem_fill(int dev, uint8_t v) {
    if (dev < 0 || dev >= MEM_N) return;
    for (uint32_t i = 0; i < MEM_WIN; i++) mem[dev][i] = v;
}

uint8_t mem_peek(int dev, uint32_t addr) { uint8_t *p = mem_ptr(dev, addr); return p ? *p : 0; }

void psram_stats_clear(void) {
    psram_stats.reads = psram_stats.writes = psram_stats.read_bytes = psram_stats.stale_flush = 0;
    psram_stats.bad_cmd = psram_stats.out_of_range = psram_stats.tx_late = 0;
    psram_stats.writes_model_area = psram_stats.last_bad_waddr = 0;
    for (int d = 0; d < MEM_N; d++) psram_stats.rd_dev[d] = 0;
    psram_stats.wr_not_ramb = psram_stats.cs_bad = psram_stats.hdr_abort = psram_stats.burst_abort = 0;
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
    const uint32_t rx_empty = 1u << (PIO_FSTAT_RXEMPTY_LSB + SM_RX);
    const uint32_t tx_full  = 1u << (PIO_FSTAT_TXFULL_LSB + SM_TX);
    const uint32_t tx_empty = 1u << (PIO_FSTAT_TXEMPTY_LSB + SM_TX);
    bool have_cmd = false;
    uint32_t cmdw = 0;
    int dev = -1;
    for (;;) {
        while (pio->fstat & rx_empty) tight_loop_contents();
        uint32_t w = *rxf;
        if ((w & CMD_SIG_MASK) == CMD_SIG_MASK) {
            /* command word (edge 8): the transaction runs for >= 8 more edges, so its CS is still low.
             * (A data word can never look like this: the 2 nibbles after the address are 0 for reads,
             * and writes only go to 0x7FFFxx.) */
            uint32_t cs = (~sio_hw->gpio_in >> PSRAM_PIN_CS0) & 7u;
            if (have_cmd) psram_stats.hdr_abort++;     /* the previous one was cut short in its header */
            dev = cs == 1u ? MEM_FLASH : cs == 2u ? MEM_RAMA : cs == 4u ? MEM_RAMB : -1;
            if (dev < 0) psram_stats.cs_bad++;
            cmdw = w & CMD_BIT_MASK;
            have_cmd = true;
            continue;
        }
        if (!have_cmd) { psram_stats.bad_cmd++; continue; }
        have_cmd = false;
        uint32_t addr = w >> 8;                         /* word 2 (edge 16): addr[23:0] << 8 | 2 nibbles */
        if (cmdw == CMD_EB) {
            /* words left over from a burst the chip cut short: flush before queuing this one */
            if (!(pio->fstat & tx_empty)) {
                pio_sm_clear_fifos(pio, SM_TX);
                psram_stats.stale_flush++;
            }
            uint32_t bmax = dev == MEM_FLASH ? 5u : 3u;                 /* kws_qspi: flash 128 B, PSRAM 32 B */
            uint32_t b = psram_burst & 7u; if (b > bmax) b = bmax;
            uint32_t blen = 4u << b, bcnt = blen - (addr & (blen - 1u)); /* bytes to the aligned boundary */
            const uint8_t *p = mem_ptr(dev, addr);
            bool whole = p && mem_ptr(dev, addr + bcnt - 1u) == p + bcnt - 1u;
            if (!whole && !p) psram_stats.out_of_range++;
            if (psram_trace_n < PSRAM_TRACE_N) {
                psram_trace[psram_trace_n] = 0xEBu << 24 | addr;
                psram_trace_dev[psram_trace_n++] = (uint8_t)(dev < 0 ? 3 : dev);
            }
            for (uint32_t i = 0; i < bcnt; i += 4) {
                uint32_t v = 0;
                for (uint32_t k = 0; k < 4; k++) {
                    uint32_t byte = 0;
                    if (i + k < bcnt) {
                        if (whole) byte = p[i + k];
                        else { const uint8_t *q = mem_ptr(dev, addr + i + k); byte = q ? *q : 0u; }
                    }
                    v = (v << 8) | byte;                                /* high nibble of byte 0 first */
                }
                while (pio->fstat & tx_full) {
                    if (!(pio->fstat & rx_empty)) { psram_stats.burst_abort++; goto next; }  /* a new transaction began */
                }
                *txf = v;
            }
            psram_stats.reads++;
            if (dev >= 0) psram_stats.rd_dev[dev]++;
            psram_stats.read_bytes += bcnt;
        next:;
        } else if (cmdw == CMD_38) {
            uint8_t *q = mem_ptr(dev, addr);
            if (q) *q = (uint8_t)w; else psram_stats.out_of_range++;
            if (dev != MEM_RAMB) psram_stats.wr_not_ramb++;
            else if (addr < HI_BASE) { psram_stats.writes_model_area++; psram_stats.last_bad_waddr = addr; }
            psram_stats.writes++;
            if (psram_trace_n < PSRAM_TRACE_N) {
                psram_trace[psram_trace_n] = 0x38u << 24 | addr;
                psram_trace_dev[psram_trace_n++] = (uint8_t)(dev < 0 ? 3 : dev);
            }
        } else {
            psram_stats.bad_cmd++;
        }
    }
}

/* ---------------------------------------------------------------- set-up */
static void cs3_config(pio_sm_config *c) {           /* in_base CS0, 3 pins visible (RP2350 IN_COUNT) */
    sm_config_set_in_pins(c, PSRAM_PIN_CS0);
    sm_config_set_in_pin_count(c, 3);
}

static uint cs_watch_init(uint off, uint target_sm, uint32_t opcode) {
    uint sm = pio_claim_unused_sm(pio_s, true);
    pio_sm_config c = qspi_cs_program_get_default_config(off);
    cs3_config(&c);
    pio_sm_init(pio_s, sm, off, &c);
    pio_sm_put_blocking(pio_s, sm, opcode);            /* OSR = opcode */
    pio_sm_exec(pio_s, sm, pio_encode_pull(false, true));
    pio_sm_exec(pio_s, sm, pio_encode_set(pio_y, 7));  /* Y = all three CS high */
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

void psram_rearm(void) {
    pio_set_sm_mask_enabled(pio, (1u << SM_RX) | (1u << SM_TX), false);
    pio_sm_clear_fifos(pio, SM_RX);
    pio_sm_clear_fifos(pio, SM_TX);
    pio_sm_restart(pio, SM_RX);                        /* shift counters, delay, stalled instruction */
    pio_sm_restart(pio, SM_TX);
    pio_sm_exec(pio, SM_RX, pio_encode_jmp(off_rx_g + qspi_rx_offset_start));
    pio_sm_exec(pio, SM_TX, pio_encode_jmp(off_tx_g + qspi_tx_offset_start));
    pio->irq = (1u << 4) | (1u << 5) | (1u << 7);      /* write 1 to clear */
    pio_set_sm_mask_enabled(pio, (1u << SM_RX) | (1u << SM_TX), true);
}

void psram_start(void) {
    for (uint p = PSRAM_PIN_CS0; p <= PSRAM_PIN_CS2; p++) {
        gpio_init(p); gpio_set_dir(p, GPIO_IN); gpio_pull_up(p);   /* the chip releases CS while CTRL.RUN = 0 */
    }
    for (uint p = PSRAM_PIN_SD0; p < PSRAM_PIN_SD0 + 4; p++) pio_gpio_init(pio, p);
    gpio_init(PSRAM_PIN_SCK); gpio_set_dir(PSRAM_PIN_SCK, GPIO_IN); gpio_pull_down(PSRAM_PIN_SCK);
    pio_sm_set_consecutive_pindirs(pio, SM_TX, PSRAM_PIN_SD0, 4, false);

    pio_sm_claim(pio, SM_RX);
    pio_sm_claim(pio, SM_TX);
    pio_sm_claim(pio, SM_DET);
    off_det_g = pio_add_program(pio, &qspi_csdet_program);
    off_rx_g = pio_add_program(pio, &qspi_rx_program);
    off_tx_g = pio_add_program(pio, &qspi_tx_program);
    uint off_cs = pio_add_program(pio_s, &qspi_cs_program);

    pio_sm_config c = qspi_csdet_program_get_default_config(off_det_g);
    cs3_config(&c);
    pio_sm_init(pio, SM_DET, off_det_g + qspi_csdet_offset_start, &c);
    pio_sm_exec(pio, SM_DET, pio_encode_set(pio_y, 7));

    c = qspi_rx_program_get_default_config(off_rx_g);
    sm_config_set_in_pins(&c, PSRAM_PIN_SD0);
    sm_config_set_in_shift(&c, false, true, 32);       /* shift left, autopush 32 */
    pio_sm_init(pio, SM_RX, off_rx_g + qspi_rx_offset_start, &c);

    c = qspi_tx_program_get_default_config(off_tx_g);
    sm_config_set_in_pins(&c, PSRAM_PIN_SD0);
    sm_config_set_out_pins(&c, PSRAM_PIN_SD0, 4);
    sm_config_set_set_pins(&c, PSRAM_PIN_SD0, 4);      /* 'set pindirs' drives / releases SD0..3 */
    sm_config_set_out_shift(&c, false, false, 32);     /* shift left, NO autopull (explicit pull per word) */
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);     /* 8-word TX FIFO = one 32-byte PSRAM burst */
    pio_sm_init(pio, SM_TX, off_tx_g + qspi_tx_offset_start, &c);
    pio_sm_set_pins_with_mask(pio, SM_TX, 0, 0xFu << PSRAM_PIN_SD0);

    uint sm_csa = cs_watch_init(off_cs, SM_RX, pio_encode_jmp(off_rx_g + qspi_rx_offset_start));
    uint sm_csb = cs_watch_init(off_cs, SM_TX, pio_encode_jmp(off_tx_g + qspi_tx_offset_start));

    /* raw end-of-transaction counter (diagnostic) */
    sm_raw = pio_claim_unused_sm(pio_s, true);
    off_raw = pio_add_program(pio_s, &qspi_csraw_program);
    pio_sm_config r = qspi_csraw_program_get_default_config(off_raw);
    cs3_config(&r);
    sm_config_set_fifo_join(&r, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio_s, sm_raw, off_raw, &r);
    pio_sm_exec(pio_s, sm_raw, pio_encode_set(pio_y, 7));

    pio->irq = (1u << 4) | (1u << 5) | (1u << 7);
    psram_tx_stalled();
    multicore_launch_core1(psram_server);
    pio_set_sm_mask_enabled(pio_s, (1u << sm_csa) | (1u << sm_csb) | (1u << sm_raw), true);
    pio_set_sm_mask_enabled(pio, (1u << SM_RX) | (1u << SM_TX) | (1u << SM_DET), true);
}

/* ---------------------------------------------------------------- SCK hold-off */
/* rewrite the [delay] of the hold-off instructions (public h_* labels in qspi_psram.pio); these are
 * wait / in / set instructions (no address field), so the assembled opcode is position independent */
static unsigned holdoff_g;
void psram_set_holdoff(unsigned h) {
    if (h > 31) h = 31;
    static const uint8_t rx_at[] = {qspi_rx_offset_h_r0, qspi_rx_offset_h_r1};
    static const uint8_t tx_at[] = {qspi_tx_offset_h_t0, qspi_tx_offset_h_t1, qspi_tx_offset_h_t2, qspi_tx_offset_h_t3};
    for (unsigned i = 0; i < sizeof rx_at; i++)
        pio->instr_mem[off_rx_g + rx_at[i]] = (qspi_rx_program_instructions[rx_at[i]] & ~0x1F00u) | (h << 8);
    for (unsigned i = 0; i < sizeof tx_at; i++)
        pio->instr_mem[off_tx_g + tx_at[i]] = (qspi_tx_program_instructions[tx_at[i]] & ~0x1F00u) | (h << 8);
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
    sm_config_set_in_pins(&c, PSRAM_PIN_CS0);          /* GP7..GP14: CS0 CS1 CS2 SD0..3 SCK */
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
