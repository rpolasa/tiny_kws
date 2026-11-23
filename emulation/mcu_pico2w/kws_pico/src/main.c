/* kws_pico - tiny_kws (TTSKY26e rev B) emulated on a Raspberry Pi Pico 2 W.
 *
 * Signal chain, bit-exact with the chip (src/kws_frontend.v, src/kws_nn.v):
 *   PDM mic --PIO (1.024 MHz)--> DMA ring --> CIC3 /64 --> 16-ch IIR filter bank, 512-sample hop
 *   --> log2 features --> integer GRU (H=48) + classifier (12 classes) --> detector --> pins / USB / LED
 * The chip's external QSPI memory is replaced by the model blob in the Pico's flash.
 *
 * Pins (chip pin it emulates):
 *   GP2  PDM_CLK  (uo[0])   -> mic CLK          GP6  FRAME     (uo[3], toggles every frame)
 *   GP3  PDM_DAT  (ui[0])   <- mic DAT          GP7  DETECT    (uo[2], high for the detecting frame)
 *                                               GP8-11 DET_CLASS[0..3] (uo[4..7])
 *   On-board LED: lights for 0.4 s on each detection.   USB serial: results + commands ('h').
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

#include "pdm_mic.pio.h"
#include "kws_nn.h"
#include "kws_fe_port.h"
#include "kws_selftest.h"
#include "gen/kws_model_blob.h"

/* ---------------------------------------------------------------- configuration */
#define PIN_PDM_CLK    2
#define PIN_PDM_DAT    3
#define PIN_FRAME      6
#define PIN_DETECT     7
#define PIN_CLASS0     8          /* GP8..GP11 = DET_CLASS[0..3] */
#define F_PDM          1024000.0f /* 64 x 16 kHz */
#define DEF_THR        12         /* THR register    (chip default set-up, app chipSetup) */
#define DEF_N          4          /* DETCFG[3:0]     consecutive frames */
#define DEF_NFILL      2          /* DETCFG[7:4]     classes 0,1 = silence, unknown */
#define LED_MS         400

#define RING_BITS      14                     /* 16 KB ring = 4096 words = 128 ms of PDM */
#define RING_WORDS     (1u << (RING_BITS - 2))
#define RING_MASK      (RING_WORDS - 1)
static uint32_t ring[RING_WORDS] __attribute__((aligned(1u << RING_BITS)));

/* ---------------------------------------------------------------- state */
static PIO  pio = pio0;
static uint sm, off_rise, off_fall;
static int  dma_ch;
static uint32_t rd;                           /* ring read index (words) */
static float f_pdm_actual;

static kws_model_t model;
static kws_gru_t   gru;
static kws_det_t   det;
static kws_cic_t   cic;

static bool live = true, verbose = false, featdump = false, meter = false;
static int  edge = 0;                         /* 0: pdm_rise (SEL=GND), 1: pdm_fall (SEL=3V) */
static uint32_t frames, detections, overruns;
static absolute_time_t led_off;
static bool led_on;

/* statistics since the last 's' */
static uint32_t st_t0, st_busy, st_nn_max, st_nn_sum, st_nn_n, st_lag_max;
/* level meter */
static int32_t lv_min = 32767, lv_max = -32768; static int64_t lv_sum, lv_sq; static uint32_t lv_n;

uint32_t kws_time_us(void) { return time_us_32(); }

/* ---------------------------------------------------------------- PDM capture */
static void pdm_start(int e) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    uint off = e ? off_fall : off_rise;
    pio_sm_config cfg = e ? pdm_fall_program_get_default_config(off) : pdm_rise_program_get_default_config(off);
    f_pdm_actual = pdm_mic_program_init(pio, sm, off, cfg, PIN_PDM_CLK, PIN_PDM_DAT, F_PDM);
    pio_sm_set_enabled(pio, sm, true);
}

static void dma_start(void) {
    dma_ch = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, RING_BITS);          /* wrap the write address */
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
    dma_channel_configure(dma_ch, &c, ring, &pio->rxf[sm], dma_encode_endless_transfer_count(), true);
}

static inline uint32_t ring_widx(void) {
    return ((uintptr_t)dma_channel_hw_addr(dma_ch)->write_addr - (uintptr_t)ring) / 4u & RING_MASK;
}

/* ---------------------------------------------------------------- chip state (CTRL.CLEAR) */
static void kws_clear(void) {
    kws_cic_reset(&cic);
    kws_fe_reset();
    kws_gru_reset(&gru, &model);
    kws_det_reset(&det);
    rd = ring_widx();                                      /* drop audio captured before the clear */
}

/* ---------------------------------------------------------------- outputs */
static void led(bool on) {
    if (on != led_on) { cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on); led_on = on; }
}

static void on_frame(const uint8_t *feat) {
    uint32_t t0 = time_us_32();
    kws_gru_step(&gru, feat);
    int fire = kws_det_step(&det, gru.top1, gru.v1, gru.v2);
    uint32_t dt = time_us_32() - t0;
    st_nn_sum += dt; st_nn_n++; if (dt > st_nn_max) st_nn_max = dt;
    frames++;

    gpio_xor_mask(1u << PIN_FRAME);
    gpio_put(PIN_DETECT, fire);
    if (fire) gpio_put_masked(0xFu << PIN_CLASS0, (uint32_t)det.det_class << PIN_CLASS0);

    if (featdump) {
        printf("F %6lu", (unsigned long)frames);
        for (int k = 0; k < KWS_F; k++) printf(" %3d", feat[k]);
        printf("\n");
    }
    if (verbose)
        printf("%6lu  %-9s %4d %4d  margin %4d%s\n", (unsigned long)frames, KWS_CLASS_NAMES[gru.top1], gru.v1, gru.v2,
               gru.v1 - gru.v2, fire ? "  <== DETECT" : "");
    if (fire) {
        detections++;
        printf(">>> DETECT  %-6s  (class %d, margin %d, frame %lu)\n", KWS_CLASS_NAMES[det.det_class], det.det_class,
               gru.v1 - gru.v2, (unsigned long)frames);
        led(true);
        led_off = make_timeout_time_ms(LED_MS);
    }
}

/* ---------------------------------------------------------------- live processing */
static void process_audio(void) {
    static int16_t pcm[32];
    static uint8_t feat[KWS_F];
    uint32_t w = ring_widx();
    uint32_t avail = (w - rd) & RING_MASK;
    if (avail > st_lag_max) st_lag_max = avail;
    if (avail > RING_WORDS - 256) { overruns++; rd = w; return; }   /* fell behind: resync */
    while (avail) {
        uint32_t n = avail;
        if (n > 64) n = 64;                                    /* 64 words = 32 samples = 2 ms */
        if (n > RING_WORDS - rd) n = RING_WORDS - rd;
        uint32_t t0 = time_us_32();
        int ns = kws_cic_words(&cic, &ring[rd], (int)n, pcm);
        for (int i = 0; i < ns; i++) {
            if (meter) {
                int32_t x = pcm[i];
                if (x < lv_min) lv_min = x;
                if (x > lv_max) lv_max = x;
                lv_sum += x; lv_sq += (int64_t)x * x; lv_n++;
            }
            if (kws_fe_sample(pcm[i], feat)) on_frame(feat);
        }
        st_busy += time_us_32() - t0;
        rd = (rd + n) & RING_MASK;
        avail -= n;
    }
}

static void meter_print(void) {
    if (!meter || lv_n < 8000) return;                         /* every 0.5 s */
    int32_t mean = (int32_t)(lv_sum / (int64_t)lv_n);
    int64_t var = lv_sq / (int64_t)lv_n - (int64_t)mean * mean;
    int32_t rms = (int32_t)sqrtf((float)var);
    printf("level: mean %6ld  rms %6ld  min %6ld  max %6ld  %s\n", (long)mean, (long)rms, (long)lv_min, (long)lv_max,
           (lv_max - lv_min) < 8 ? "<- flat: no data on this edge (press 'e') or mic not wired" : "");
    lv_min = 32767; lv_max = -32768; lv_sum = lv_sq = 0; lv_n = 0;
}

/* ---------------------------------------------------------------- console */
static void help(void) {
    printf("\ncommands:\n"
           "  t  run self-tests M1-M3 (bit-exact vs golden model)\n"
           "  l  live detection on/off            c  CLEAR (reset hidden state, detector, filters)\n"
           "  v  per-frame top1/margin on/off     f  per-frame 16 features on/off\n"
           "  m  microphone level meter on/off    e  PDM capture edge (now %s)\n"
           "  + / -  THR +1 / -1 (now %d)         ] / [  N +1 / -1 (now %d)\n"
           "  s  statistics (CPU load, NN time, overruns)     h  this help\n",
           edge ? "falling: SEL=3V" : "rising: SEL=GND", det.thr, det.n);
}

static void stats(void) {
    uint32_t el = time_us_32() - st_t0;
    printf("frames %lu, detections %lu, overruns %lu | CPU %.1f %% (CIC+FE+NN), NN %lu us avg / %lu us max per frame,"
           " ring lag max %lu words of %u | THR %d N %d NFILL %d | PDM %.1f Hz -> fs %.2f Hz\n",
           (unsigned long)frames, (unsigned long)detections, (unsigned long)overruns, el ? 100.0 * st_busy / el : 0.0,
           (unsigned long)(st_nn_n ? st_nn_sum / st_nn_n : 0), (unsigned long)st_nn_max, (unsigned long)st_lag_max,
           RING_WORDS, det.thr, det.n, det.nfill, f_pdm_actual, f_pdm_actual / 64.0f);
    st_t0 = time_us_32(); st_busy = st_nn_max = st_nn_sum = st_nn_n = st_lag_max = 0;
}

static void command(int ch) {
    switch (ch) {
    case 't': kws_test_all(&model); kws_clear(); break;      /* tests reuse the FE state */
    case 'l': live = !live; kws_clear(); printf("live %s\n", live ? "ON" : "OFF"); break;
    case 'c': kws_clear(); printf("CLEAR\n"); break;
    case 'v': verbose = !verbose; break;
    case 'f': featdump = !featdump; break;
    case 'm': meter = !meter; break;
    case 'e': edge = !edge; pdm_start(edge); kws_clear();
              printf("PDM capture: %s\n", edge ? "falling edge (SEL=3V)" : "rising edge (SEL=GND)"); break;
    case '+': if (det.thr < 255) det.thr++; printf("THR %d\n", det.thr); break;
    case '-': if (det.thr > 0) det.thr--; printf("THR %d\n", det.thr); break;
    case ']': if (det.n < 15) det.n++; printf("N %d\n", det.n); break;
    case '[': if (det.n > 1) det.n--; printf("N %d\n", det.n); break;
    case 's': stats(); break;
    case 'h': case '?': help(); break;
    default: break;
    }
}

/* ---------------------------------------------------------------- main */
int main(void) {
    stdio_init_all();
    bool wifi_ok = cyw43_arch_init() == 0;                     /* the LED is on the CYW43 chip */

    gpio_init_mask((1u << PIN_FRAME) | (1u << PIN_DETECT) | (0xFu << PIN_CLASS0));
    gpio_set_dir_out_masked((1u << PIN_FRAME) | (1u << PIN_DETECT) | (0xFu << PIN_CLASS0));
    gpio_put_masked((1u << PIN_FRAME) | (1u << PIN_DETECT) | (0xFu << PIN_CLASS0), 0);

    /* M0: blink 3x while waiting (up to 4 s) for the USB serial terminal */
    for (int i = 0; i < 40 && !stdio_usb_connected(); i++) {
        if (wifi_ok) cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, (i / 5) % 2 == 0 && i < 30);
        sleep_ms(100);
    }
    if (wifi_ok) cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    printf("\n=== kws_pico: tiny_kws rev B emulation on Pico 2 W (sys clk %lu MHz) ===\n",
           (unsigned long)(clock_get_hz(clk_sys) / 1000000));
    if (!wifi_ok) printf("warning: CYW43 init failed - no LED (detection output still on USB and GP7)\n");
    int e = kws_model_parse(&model, KWS_MODEL_BLOB, KWS_MODEL_BYTES);
    if (e) {
        printf("model blob error %d - halted\n", e);
        while (true) tight_loop_contents();
    }
    printf("model: %d bytes, md5 %s, H=%d, %d classes:", KWS_MODEL_BYTES, KWS_MODEL_MD5, model.H, model.C);
    for (int c = 0; c < model.C; c++) printf(" %s", KWS_CLASS_NAMES[c]);
    printf("\n");
    kws_det_init(&det, DEF_THR, DEF_N, DEF_NFILL);

    kws_test_all(&model);                                      /* M1-M3 at every boot */

    sm = pio_claim_unused_sm(pio, true);
    off_rise = pio_add_program(pio, &pdm_rise_program);
    off_fall = pio_add_program(pio, &pdm_fall_program);
    pdm_start(edge);
    dma_start();
    kws_clear();
    printf("live: PDM_CLK GP2 %.1f Hz (f_s %.2f Hz), PDM_DAT GP3, %s; say: yes no up down left right on off stop go\n",
           f_pdm_actual, f_pdm_actual / 64.0f, edge ? "falling edge" : "rising edge (mic SEL = GND)");
    help();
    st_t0 = time_us_32();

    while (true) {
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) command(ch);
        if (live) process_audio();
        else rd = ring_widx();
        meter_print();
        if (led_on && time_reached(led_off)) led(false);
    }
}
