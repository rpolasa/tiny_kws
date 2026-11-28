#include "kws_chip.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"

static bool frame_last;
static uint16_t fcnt_last;
static absolute_time_t next_poll;
uint32_t chip_frame_pin_ok, chip_frame_pin_missed, chip_det_pin_mismatch;

uint16_t chip_fcnt(void) { uint8_t b[2]; chip_rd(R_FCNT0, b, 2); return (uint16_t)(b[0] | b[1] << 8); }

void chip_init(void) {
    spi_init(spi0, SPI_HZ);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_SPI_CSN);  gpio_put(PIN_SPI_CSN, 1);  gpio_set_dir(PIN_SPI_CSN, GPIO_OUT);
    gpio_init(PIN_CHIP_RSTN); gpio_put(PIN_CHIP_RSTN, 1); gpio_set_dir(PIN_CHIP_RSTN, GPIO_OUT);
    gpio_init(PIN_DETECT);   gpio_set_dir(PIN_DETECT, GPIO_IN);  gpio_pull_down(PIN_DETECT);
    gpio_init(PIN_FRAME);    gpio_set_dir(PIN_FRAME, GPIO_IN);   gpio_pull_down(PIN_FRAME);
}

void chip_reset(void) {
    gpio_put(PIN_CHIP_RSTN, 0);
    sleep_us(20);
    gpio_put(PIN_CHIP_RSTN, 1);
    sleep_us(50);                       /* reset synchronisers + internal clear sequences (~80 clk) */
    frame_last = gpio_get(PIN_FRAME);
    fcnt_last = chip_fcnt();
    next_poll = get_absolute_time();
}

static inline void cs_low(void)  { gpio_put(PIN_SPI_CSN, 0); busy_wait_us_32(1); }
static inline void cs_high(void) { busy_wait_us_32(1); gpio_put(PIN_SPI_CSN, 1); busy_wait_us_32(2); }

void chip_wr(uint8_t addr, const uint8_t *d, int n) {
    uint8_t a = addr & 0x7F;
    cs_low();
    spi_write_blocking(spi0, &a, 1);
    if (n) spi_write_blocking(spi0, d, (size_t)n);
    cs_high();
}

void chip_rd(uint8_t addr, uint8_t *d, int n) {
    uint8_t a = 0x80 | addr;
    cs_low();
    spi_write_blocking(spi0, &a, 1);
    spi_read_blocking(spi0, 0x00, d, (size_t)n);
    cs_high();
}

void chip_wr1(uint8_t addr, uint8_t v) { chip_wr(addr, &v, 1); }
uint8_t chip_rd1(uint8_t addr) { uint8_t v; chip_rd(addr, &v, 1); return v; }

/* A frame is done when the FRAME pin toggles or, if that wire is not seen, when the frame counter
 * register (0x24/0x25) advances (polled every 250 us). The register is the reference; pin health is
 * counted in chip_frame_pin_ok / chip_frame_pin_missed. */
bool chip_wait_frame(uint32_t timeout_us) {
    absolute_time_t t = make_timeout_time_us(timeout_us);
    do {
        bool f = gpio_get(PIN_FRAME);
        if (f != frame_last) {
            frame_last = f; busy_wait_us_32(2);
            fcnt_last = chip_fcnt(); chip_frame_pin_ok++;
            return true;
        }
        if (time_reached(next_poll)) {
            next_poll = make_timeout_time_us(250);
            uint16_t c = chip_fcnt();
            if (c != fcnt_last) {
                fcnt_last = c; chip_frame_pin_missed++;
                frame_last = gpio_get(PIN_FRAME);
                return true;
            }
        }
    } while (!time_reached(t));
    return false;
}

/* DETECT for the frame just finished: STATUS.det (bit 4, latched by the chip, cleared here). The DETECT
 * pin is compared with it and mismatches counted (the pin is high for the detecting frame only). */
bool chip_frame_det(void) {
    uint8_t st = chip_rd1(R_STATUS);
    bool det = (st & S_DET) != 0;
    if (det) chip_wr1(R_STATUS, S_DET);
    if ((bool)gpio_get(PIN_DETECT) != det) chip_det_pin_mismatch++;
    return det;
}

void chip_pin_stats_clear(void) { chip_frame_pin_ok = chip_frame_pin_missed = chip_det_pin_mismatch = 0; }
