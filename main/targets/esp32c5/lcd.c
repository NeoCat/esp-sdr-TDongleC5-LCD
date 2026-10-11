/* ST7735 (80x160) waterfall for the LilyGO T-Dongle-C5.
 * Rows arrive as power codes of any FFT size; each is folded to 160 columns
 * (peak per column, DC centred), normalized against a tracked noise floor and
 * stored in a ring that is redrawn newest-first by a low-load render task. */
#include "lcd.h"
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_gpio_reserve.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* T-Dongle-C5 wiring (LilyGO pin_config.h). */
#define PIN_MOSI 2
#define PIN_SCK 6
#define PIN_BL 0 /* active low */
#define PIN_RST 1
#define PIN_DC 3
#define PIN_CS 10
#define PIN_BUTTON 28 /* BOOT, active low */

#define LCD_W 160
#define LCD_H 80
#define HEADER_H 10
#define WF_H (LCD_H - HEADER_H)
#define X_GAP 1 /* panel RAM offsets for the 80x160 glass in landscape */
#define Y_GAP 26
#define CHUNK_ROWS 8
#define SPI_HZ 20000000
#define RANGE_CODES 80 /* 40 dB between noise floor and saturation */
#define FLOOR_MARGIN 6

#define FREQ_A 2442
#define FREQ_B 5500

static spi_device_handle_t spi;
static uint16_t *chunk; /* DMA-capable RGB565 staging */
static TaskHandle_t task;
static SemaphoreHandle_t lock;
static volatile unsigned button_mhz = FREQ_A;
static bool ready;

/* Heap, not .bss: the C5 RF dump bank caps static data (see sram_guard.ld). */
static struct {
    uint8_t ring[WF_H][LCD_W];
    uint16_t header_cols[LCD_W]; /* bit y set = foreground pixel */
    uint16_t palette[256];       /* RGB565, byte-swapped for the wire */
} *ui;
#define ring ui->ring
#define header_cols ui->header_cols
#define palette ui->palette
static unsigned head; /* newest row */
static unsigned shown_mhz, shown_span_mhz, floor_q4;
static lcd_source_t shown_source;
static bool have_row;

static void dc_callback(spi_transaction_t *t) { gpio_set_level(PIN_DC, (int)(intptr_t)t->user); }

static void send(const void *data, size_t bytes, int dc) {
    spi_transaction_t t = {.length = bytes * 8, .tx_buffer = data, .user = (void *)(intptr_t)dc};
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

static void command(uint8_t cmd, const uint8_t *args, unsigned count) {
    spi_transaction_t t = {.length = 8, .flags = SPI_TRANS_USE_TXDATA, .tx_data = {cmd}, .user = (void *)0};
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    if (count) {
        memcpy(chunk, args, count);
        send(chunk, count, 1);
    }
}
#define CMD(c, ...) do { const uint8_t a[] = {__VA_ARGS__}; command(c, a, sizeof(a)); } while (0)

static void window(void) {
    CMD(0x2a, 0, X_GAP, 0, X_GAP + LCD_W - 1);
    CMD(0x2b, 0, Y_GAP, 0, Y_GAP + LCD_H - 1);
    command(0x2c, NULL, 0);
}

static void panel_init(void) {
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    command(0x01, NULL, 0); /* SWRESET */
    vTaskDelay(pdMS_TO_TICKS(150));
    command(0x11, NULL, 0); /* SLPOUT */
    vTaskDelay(pdMS_TO_TICKS(120));
    CMD(0xb1, 0x01, 0x2c, 0x2d);
    CMD(0xb2, 0x01, 0x2c, 0x2d);
    CMD(0xb3, 0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d);
    CMD(0xb4, 0x07);
    CMD(0xc0, 0xa2, 0x02, 0x84);
    CMD(0xc1, 0xc5);
    CMD(0xc2, 0x0a, 0x00);
    CMD(0xc3, 0x8a, 0x2a);
    CMD(0xc4, 0x8a, 0xee);
    CMD(0xc5, 0x0e);
    command(0x21, NULL, 0); /* INVON */
    CMD(0x3a, 0x05);        /* 16 bit/pixel */
    CMD(0x36, 0xa8);        /* MY | MV | BGR: landscape, USB connector on the left */
    CMD(0xe0, 0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2b, 0x39, 0x00, 0x01, 0x03, 0x10);
    CMD(0xe1, 0x03, 0x1d, 0x07, 0x06, 0x2e, 0x2c, 0x29, 0x2d, 0x2e, 0x2e, 0x37, 0x3f, 0x00, 0x00, 0x02, 0x10);
    command(0x13, NULL, 0); /* NORON */
    vTaskDelay(pdMS_TO_TICKS(10));
    command(0x29, NULL, 0); /* DISPON */
    vTaskDelay(pdMS_TO_TICKS(100));
}

/* Black, blue, cyan, yellow, red. */
static void build_palette(void) {
    static const uint8_t stops[5][3] = {{0, 0, 0}, {0, 0, 190}, {0, 210, 210}, {255, 255, 0}, {255, 30, 0}};
    for (unsigned v = 0; v < 256; v++) {
        unsigned seg = v * 4 / 255;
        if (seg > 3) seg = 3;
        unsigned base = seg * 255 / 4, span = (seg + 1) * 255 / 4 - base;
        uint8_t rgb[3];
        for (unsigned k = 0; k < 3; k++)
            rgb[k] = (stops[seg][k] * (span - (v - base)) + stops[seg + 1][k] * (v - base)) / span;
        uint16_t c = (rgb[0] & 0xf8) << 8 | (rgb[1] & 0xfc) << 3 | rgb[2] >> 3;
        palette[v] = c >> 8 | c << 8;
    }
}

/* Classic 5x7 column-major glyphs; only what the header prints. */
static const uint8_t *glyph(char c) {
    static const uint8_t blank[5] = {0}, digits[10][5] = {
        {0x3e, 0x51, 0x49, 0x45, 0x3e}, {0x00, 0x42, 0x7f, 0x40, 0x00}, {0x42, 0x61, 0x51, 0x49, 0x46},
        {0x21, 0x41, 0x45, 0x4b, 0x31}, {0x18, 0x14, 0x12, 0x7f, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3c, 0x4a, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03}, {0x36, 0x49, 0x49, 0x49, 0x36},
        {0x06, 0x49, 0x49, 0x29, 0x1e}};
    static const uint8_t M[5] = {0x7f, 0x02, 0x0c, 0x02, 0x7f}, H[5] = {0x7f, 0x08, 0x08, 0x08, 0x7f},
                         z[5] = {0x44, 0x64, 0x54, 0x4c, 0x44}, W[5] = {0x3f, 0x40, 0x38, 0x40, 0x3f},
                         E[5] = {0x7f, 0x49, 0x49, 0x49, 0x41}, B[5] = {0x7f, 0x49, 0x49, 0x49, 0x36},
                         L[5] = {0x7f, 0x40, 0x40, 0x40, 0x40}, C[5] = {0x3e, 0x41, 0x41, 0x41, 0x22},
                         D[5] = {0x7f, 0x41, 0x41, 0x22, 0x1c};
    if (c >= '0' && c <= '9') return digits[c - '0'];
    switch (c) {
    case 'M': return M; case 'H': return H; case 'z': return z; case 'W': return W;
    case 'E': return E; case 'B': return B; case 'L': return L; case 'C': return C;
    case 'D': return D;
    default: return blank;
    }
}

static void text(unsigned x, const char *s) {
    for (; *s && x + 5 <= LCD_W; s++, x += 6) {
        const uint8_t *g = glyph(*s);
        for (unsigned col = 0; col < 5; col++) header_cols[x + col] |= (uint16_t)g[col] << 1; /* rows 1..7 */
    }
}

static void build_header(void) {
    char s[24];
    memset(header_cols, 0, sizeof(header_cols));
    snprintf(s, sizeof(s), "%uMHz", shown_mhz);
    text(2, s);
    snprintf(s, sizeof(s), "%uM", shown_span_mhz);
    text(56, s);
    text(LCD_W - 2 - 18, shown_source == LCD_SOURCE_HOST ? "WEB" : "LCD");
    /* Frequency ticks on the separator row: both edges and the centre. */
    for (unsigned x = 0; x < LCD_W; x++) header_cols[x] |= 1u << 9;
    static const unsigned ticks[] = {0, LCD_W / 4, LCD_W / 2, 3 * LCD_W / 4, LCD_W - 1};
    for (unsigned k = 0; k < 5; k++) header_cols[ticks[k]] |= 1u << 8;
}

static void render(void) {
    window();
    for (unsigned y0 = 0; y0 < LCD_H; y0 += CHUNK_ROWS) {
        xSemaphoreTake(lock, portMAX_DELAY);
        uint16_t *p = chunk;
        for (unsigned y = y0; y < y0 + CHUNK_ROWS; y++)
            for (unsigned x = 0; x < LCD_W; x++) {
                if (y < HEADER_H) {
                    *p++ = (header_cols[x] >> y & 1) ? 0xffff : 0x0000;
                } else {
                    unsigned row = (head + WF_H - (y - HEADER_H)) % WF_H;
                    *p++ = palette[ring[row][x]];
                }
            }
        xSemaphoreGive(lock);
        send(chunk, LCD_W * CHUNK_ROWS * 2, 1);
    }
}

void lcd_push(const uint8_t *codes, unsigned bins, unsigned rate_hz, unsigned mhz, lcd_source_t source) {
    if (!ready || bins < LCD_W || (bins & (bins - 1))) return;
    uint8_t row[LCD_W];
    unsigned counts[256] = {0};
    for (unsigned c = 0; c < LCD_W; c++) {
        unsigned a = c * bins / LCD_W, b = (c + 1) * bins / LCD_W, peak = 0;
        if (b == a) b = a + 1;
        for (unsigned k = a; k < b; k++) { /* fftshift: negative frequencies on the left */
            unsigned v = codes[(k + bins / 2) & (bins - 1)];
            if (v > peak) peak = v;
        }
        row[c] = peak;
        counts[peak]++;
    }
    /* Noise floor: 20th percentile of the row, smoothed over time. */
    unsigned seen = 0, low = 0;
    while (low < 255 && (seen += counts[low]) < LCD_W / 5) low++;
    unsigned span = (rate_hz + 500000) / 1000000;

    xSemaphoreTake(lock, portMAX_DELAY);
    if (!have_row || mhz != shown_mhz || span != shown_span_mhz || source != shown_source) {
        memset(ring, 0, sizeof(ring));
        floor_q4 = low * 16;
        shown_mhz = mhz; shown_span_mhz = span; shown_source = source;
        have_row = true;
        build_header();
    } else {
        floor_q4 += ((int)(low * 16) - (int)floor_q4) / 8;
    }
    int lo = (int)(floor_q4 / 16) - FLOOR_MARGIN;
    head = (head + 1) % WF_H;
    for (unsigned c = 0; c < LCD_W; c++) {
        int v = ((int)row[c] - lo) * 255 / RANGE_CODES;
        ring[head][c] = v < 0 ? 0 : v > 255 ? 255 : v;
    }
    xSemaphoreGive(lock);
    xTaskNotifyGive(task);
}

unsigned lcd_button_frequency(void) { return button_mhz; }

static void lcd_task(void *arg) {
    (void)arg;
    unsigned stable = 0;
    bool pressed = false;
    for (;;) {
        bool dirty = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) > 0;
        bool down = gpio_get_level(PIN_BUTTON) == 0;
        stable = down == pressed ? 0 : stable + 1;
        if (stable >= 2) { /* two consecutive samples (>= 20 ms) differ: debounced edge */
            pressed = down;
            stable = 0;
            if (pressed) button_mhz = button_mhz == FREQ_A ? FREQ_B : FREQ_A;
        }
        if (dirty) render();
    }
}

bool lcd_init(void) {
    const uint64_t pins = (1ull << PIN_MOSI) | (1ull << PIN_SCK) | (1ull << PIN_BL) |
                          (1ull << PIN_RST) | (1ull << PIN_DC) | (1ull << PIN_CS) | (1ull << PIN_BUTTON);
    const gpio_config_t outputs = {.pin_bit_mask = (1ull << PIN_BL) | (1ull << PIN_RST) | (1ull << PIN_DC),
                                   .mode = GPIO_MODE_OUTPUT};
    const gpio_config_t button = {.pin_bit_mask = 1ull << PIN_BUTTON, .mode = GPIO_MODE_INPUT,
                                  .pull_up_en = GPIO_PULLUP_ENABLE};
    const spi_bus_config_t bus = {.mosi_io_num = PIN_MOSI, .miso_io_num = -1, .sclk_io_num = PIN_SCK,
                                  .quadwp_io_num = -1, .quadhd_io_num = -1,
                                  .max_transfer_sz = LCD_W * CHUNK_ROWS * 2};
    const spi_device_interface_config_t dev = {.clock_speed_hz = SPI_HZ, .mode = 0, .spics_io_num = PIN_CS,
                                               .queue_size = 1, .pre_cb = dc_callback};
    chunk = heap_caps_malloc(LCD_W * CHUNK_ROWS * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ui = heap_caps_calloc(1, sizeof(*ui), MALLOC_CAP_INTERNAL);
    lock = xSemaphoreCreateMutex();
    if (!chunk || !ui || !lock) return false;
    if (gpio_config(&outputs) != ESP_OK || gpio_config(&button) != ESP_OK) return false;
    gpio_set_level(PIN_BL, 1); /* dark until the first frame */
    if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) return false;
    if (spi_bus_add_device(SPI2_HOST, &dev, &spi) != ESP_OK) return false;
    esp_gpio_reserve(pins);
    build_palette();
    panel_init();
    memset(chunk, 0, LCD_W * CHUNK_ROWS * 2);
    window();
    for (unsigned y = 0; y < LCD_H; y += CHUNK_ROWS) send(chunk, LCD_W * CHUNK_ROWS * 2, 1);
    memset(header_cols, 0, sizeof(header_cols));
    shown_mhz = FREQ_A; shown_span_mhz = 80;
    build_header();
    gpio_set_level(PIN_BL, 0);
    ready = true;
    if (xTaskCreate(lcd_task, "lcd", 3072, NULL, 2, &task) != pdPASS) { ready = false; return false; }
    xTaskNotifyGive(task); /* draw the idle header */
    return true;
}
