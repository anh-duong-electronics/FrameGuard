/* SPDX-License-Identifier: MIT */
#ifndef LCD_ST7567_H
#define LCD_ST7567_H

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_WIDTH   128
#define LCD_HEIGHT  64
#define LCD_PAGES   (LCD_HEIGHT / 8)

/*
 * Xuyang COG12864-14-1 (1.9", ST7567, 1/65 duty, 1/9 bias, 3.3 V, SPI).
 * CS tied to GND on the board.
 *
 * FrameGuard (STM32C031): PA1=SCL(SPI1_SCK), PA2=SDA(SPI1_MOSI) — hspi1 CubeMX,
 * A0 = PA0 (labelled "D0" in the .ioc). /RST = PA4 (former USART2_TX pin; USART2 only
 * uses RX on PA3): lcd_init() configures PA4 itself and hard-resets the LCD once SPI is stable.
 * With CS tied to GND, /RST is the only way to resynchronise — so /RST must be driven by
 * the MCU, not shared with NRST (LCD wakes before the MCU → sees floating SCK → byte slip).
 */
#define LCD_A0_GPIO_Port   D0_GPIO_Port
#define LCD_A0_Pin         D0_Pin
#define LCD_RST_GPIO_Port  GPIOA
#define LCD_RST_Pin        GPIO_PIN_4

/* ST7567 DDRAM has 132 columns; this 128x64 panel starts at 0. */
#ifndef LCD_X_OFFSET
#define LCD_X_OFFSET  0
#endif

/* Contrast EV 0..63. F030 used EV=10; on C031 EV=7 is sharper than 10, trying 3. */
#ifndef LCD_CONTRAST
#define LCD_CONTRAST  10
#endif

/* Regulator ratio 0x20..0x27 → 3.0..6.5. 0x25 = 5.5 */
#ifndef LCD_RR
#define LCD_RR        0x25
#endif

/* 0xA0 SEG0→SEG131, 0xA1 reverse. 0xC0 COM0→COM63, 0xC8 reverse. */
#ifndef LCD_SEG_REMAP
#define LCD_SEG_REMAP 0xA0
#endif
#ifndef LCD_COM_DIR
#define LCD_COM_DIR   0xC8
#endif

void lcd_init(void);
void lcd_set_contrast(uint8_t ev);
void lcd_display_on(uint8_t on);
void lcd_invert(uint8_t on);

void lcd_clear(void);
void lcd_load_fullscreen(const uint8_t data[LCD_PAGES][LCD_WIDTH]);
void lcd_update(void);
void lcd_update_pages(uint8_t page0, uint8_t page1);

void lcd_set_pixel(int x, int y, uint8_t color);
void lcd_draw_hline(int x, int y, int w, uint8_t color);
void lcd_draw_vline(int x, int y, int h, uint8_t color);
void lcd_draw_rect(int x, int y, int w, int h, uint8_t color);
void lcd_fill_rect(int x, int y, int w, int h, uint8_t color);
void lcd_fill_circle(int cx, int cy, int r, uint8_t color);

/* Bold 5x7 glyphs (2 overlapping columns), 8 px advance. ASCII 0x20..0x7E. */
void lcd_draw_char(int x, int y, char c);
void lcd_draw_string(int x, int y, const char *s);
void lcd_draw_char_inv(int x, int y, char c);
void lcd_draw_string_inv(int x, int y, const char *s);
void lcd_draw_uint(int x, int y, uint32_t v);

#ifdef __cplusplus
}
#endif

#endif
