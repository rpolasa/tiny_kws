/* Host side of the tiny_kws chip (board-B role): SPI register access on SPI0 + pins.
 * SPI protocol (kws_spi_slave.v): mode 0, SCK <= f_clk/8, first byte {R/W#, addr[6:0]},
 * auto-increment except at PCM (0x10) and FEAT (0x11). */
#ifndef KWS_CHIP_H
#define KWS_CHIP_H
#include <stdint.h>
#include <stdbool.h>

#define PIN_SPI_MISO   16      /* chip uo[1] */
#define PIN_SPI_CSN    17      /* chip ui[3] */
#define PIN_SPI_SCK    18      /* chip ui[1] */
#define PIN_SPI_MOSI   19      /* chip ui[2] */
#define PIN_DETECT     20      /* chip uo[2] */
#define PIN_FRAME      21      /* chip uo[3] */
#define PIN_CHIP_RSTN  22      /* FPGA pin 21 (host_rst_n) */
#define PIN_CS1_MON    8       /* chip uio[6] CS1 (RAM A): should stay high */
#define SPI_HZ         1000000 /* f_clk/8 = 2.04 MHz at 16.3125 MHz: keep well below */

enum { R_CTRL = 0x00, R_PDMDIV, R_MEMCFG, R_BURST, R_BASE0, R_BASE1, R_BASE2, R_THR, R_DETCFG,
       R_PCM = 0x10, R_FEAT = 0x11,
       R_STATUS = 0x20, R_RESULT, R_TOP1, R_TOP2, R_FCNT0, R_FCNT1, R_ID, R_HREG, R_NCH,
       R_FEATRD = 0x30 };
enum { C_RUN = 1, C_SRC_PCM = 2, C_FEAT_HOST = 4, C_HOP512 = 8, C_CLEAR = 16, C_PDM_EDGE = 32 };
enum { S_NN_BUSY = 1, S_FE_BUSY = 2, S_OVERRUN = 4, S_MODEL_ERR = 8, S_DET = 16, S_Q_BUSY = 32 };

void chip_init(void);
void chip_reset(void);                                  /* pulse rst_n, wait for the chip */
void chip_wr(uint8_t addr, const uint8_t *d, int n);
void chip_rd(uint8_t addr, uint8_t *d, int n);
void chip_wr1(uint8_t addr, uint8_t v);
uint8_t chip_rd1(uint8_t addr);
/* wait for the end of a frame (FRAME pin toggle, or frame-counter register advance); false on timeout */
bool chip_wait_frame(uint32_t timeout_us);
bool chip_frame_det(void);                 /* DETECT of the last frame from STATUS.det (pin cross-checked) */
uint16_t chip_fcnt(void);
void chip_pin_stats_clear(void);
extern uint32_t chip_frame_pin_ok, chip_frame_pin_missed, chip_det_pin_mismatch;
static inline int memcfg_of(int rd_dly, int div, int cs) { return (rd_dly << 5) | (div << 2) | cs; }

#endif
