/* Waterfall on the LilyGO T-Dongle-C5 0.96" ST7735 LCD (160x80, landscape). */
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum { LCD_SOURCE_HOST, LCD_SOURCE_LOCAL } lcd_source_t;

/* Claim the LCD/button GPIOs and start the render task. Call before
 * burst_serial_init() so the GPIO service never offers these pins. */
bool lcd_init(void);
/* Add one spectrum row (power codes, natural FFT order, 2 codes per dB). */
void lcd_push(const uint8_t *codes, unsigned bins, unsigned rate_hz, unsigned frequency_mhz,
              lcd_source_t source);
/* Centre frequency (MHz) chosen with the BOOT button for standalone scanning. */
unsigned lcd_button_frequency(void);
