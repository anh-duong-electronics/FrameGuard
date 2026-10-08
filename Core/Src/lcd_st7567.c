/* SPDX-License-Identifier: MIT */
#include "lcd_st7567.h"
#include <string.h>

/* ST7567 128x64, SPI, CS tied to GND.
 * FrameGuard (STM32C031): PA1 SCK, PA2 MOSI (hspi1 CubeMX), PA0 A0, PA4 /RST.
 * lcd_init() has ~750 ms of delays in total — the WWDG is running, so every delay goes through lcd_delay(). */

extern SPI_HandleTypeDef hspi1;
extern void app_wwdg_kick(void);

/* The window WWDG only accepts a refresh within ~44 ms (128–172 ms after the previous one) and
 * app_wwdg_kick() ignores early calls. Delay in 10 ms steps + kick so that one kick always
 * lands inside the window; a single 50 ms delay + one kick would only hit it by luck. */
static void lcd_delay(uint32_t ms)
{
  for (uint32_t t = 0; t < ms; t += 10) {
    HAL_Delay(10);
    app_wwdg_kick();
  }
}

#define CMD_DISPLAY_OFF     0xAE
#define CMD_DISPLAY_ON      0xAF
#define CMD_START_LINE      0x40
#define CMD_PAGE            0xB0
#define CMD_COL_MSB         0x10
#define CMD_COL_LSB         0x00
#define CMD_ADC_NORMAL      0xA0
#define CMD_DISP_NORMAL     0xA6
#define CMD_DISP_REVERSE    0xA7
#define CMD_ALLPIX_OFF      0xA4
#define CMD_ALLPIX_ON       0xA5
#define CMD_BIAS_1_9        0xA2
#define CMD_SOFT_RESET      0xE2
#define CMD_POWER           0x28
#define CMD_EV              0x81
#define CMD_BOOSTER         0xF8

static uint8_t fb[LCD_PAGES][LCD_WIDTH];

/* Classic 5x7 ASCII 0x20..0x7E, column-major, LSB = top pixel. */
static const uint8_t font5x7[][5] = {
  {0x00,0x00,0x00,0x00,0x00}, /*  */
  {0x00,0x00,0x5F,0x00,0x00}, /* ! */
  {0x00,0x07,0x00,0x07,0x00}, /* " */
  {0x14,0x7F,0x14,0x7F,0x14}, /* # */
  {0x24,0x2A,0x7F,0x2A,0x12}, /* $ */
  {0x23,0x13,0x08,0x64,0x62}, /* % */
  {0x36,0x49,0x55,0x22,0x50}, /* & */
  {0x00,0x05,0x03,0x00,0x00}, /* ' */
  {0x00,0x1C,0x22,0x41,0x00}, /* ( */
  {0x00,0x41,0x22,0x1C,0x00}, /* ) */
  {0x14,0x08,0x3E,0x08,0x14}, /* * */
  {0x08,0x08,0x3E,0x08,0x08}, /* + */
  {0x00,0x00,0xA0,0x60,0x00}, /* , */
  {0x08,0x08,0x08,0x08,0x08}, /* - */
  {0x00,0x60,0x60,0x00,0x00}, /* . */
  {0x20,0x10,0x08,0x04,0x02}, /* / */
  {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
  {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
  {0x42,0x61,0x51,0x49,0x46}, /* 2 */
  {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
  {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
  {0x27,0x45,0x45,0x45,0x39}, /* 5 */
  {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
  {0x01,0x71,0x09,0x05,0x03}, /* 7 */
  {0x36,0x49,0x49,0x49,0x36}, /* 8 */
  {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
  {0x00,0x36,0x36,0x00,0x00}, /* : */
  {0x00,0x56,0x36,0x00,0x00}, /* ; */
  {0x08,0x14,0x22,0x41,0x00}, /* < */
  {0x14,0x14,0x14,0x14,0x14}, /* = */
  {0x00,0x41,0x22,0x14,0x08}, /* > */
  {0x02,0x01,0x51,0x09,0x06}, /* ? */
  {0x32,0x49,0x79,0x41,0x3E}, /* @ */
  {0x7E,0x11,0x11,0x11,0x7E}, /* A */
  {0x7F,0x49,0x49,0x49,0x36}, /* B */
  {0x3E,0x41,0x41,0x41,0x22}, /* C */
  {0x7F,0x41,0x41,0x22,0x1C}, /* D */
  {0x7F,0x49,0x49,0x49,0x41}, /* E */
  {0x7F,0x09,0x09,0x09,0x01}, /* F */
  {0x3E,0x41,0x49,0x49,0x7A}, /* G */
  {0x7F,0x08,0x08,0x08,0x7F}, /* H */
  {0x00,0x41,0x7F,0x41,0x00}, /* I */
  {0x20,0x40,0x41,0x3F,0x01}, /* J */
  {0x7F,0x08,0x14,0x22,0x41}, /* K */
  {0x7F,0x40,0x40,0x40,0x40}, /* L */
  {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
  {0x7F,0x04,0x08,0x10,0x7F}, /* N */
  {0x3E,0x41,0x41,0x41,0x3E}, /* O */
  {0x7F,0x09,0x09,0x09,0x06}, /* P */
  {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
  {0x7F,0x09,0x19,0x29,0x46}, /* R */
  {0x46,0x49,0x49,0x49,0x31}, /* S */
  {0x01,0x01,0x7F,0x01,0x01}, /* T */
  {0x3F,0x40,0x40,0x40,0x3F}, /* U */
  {0x1F,0x20,0x40,0x20,0x1F}, /* V */
  {0x3F,0x40,0x30,0x40,0x3F}, /* W — middle peak only 2 rows: bold (2 overlapping columns) does not smear into a block */
  {0x63,0x14,0x08,0x14,0x63}, /* X */
  {0x07,0x08,0x70,0x08,0x07}, /* Y */
  {0x61,0x51,0x49,0x45,0x43}, /* Z */
  {0x00,0x7F,0x41,0x41,0x00}, /* [ */
  {0x02,0x04,0x08,0x10,0x20}, /* \ */
  {0x00,0x41,0x41,0x7F,0x00}, /* ] */
  {0x04,0x02,0x01,0x02,0x04}, /* ^ */
  {0x40,0x40,0x40,0x40,0x40}, /* _ */
  {0x00,0x01,0x02,0x04,0x00}, /* ` */
  {0x20,0x54,0x54,0x54,0x78}, /* a */
  {0x7F,0x48,0x44,0x44,0x38}, /* b */
  {0x38,0x44,0x44,0x44,0x20}, /* c */
  {0x38,0x44,0x44,0x48,0x7F}, /* d */
  {0x38,0x54,0x54,0x54,0x18}, /* e */
  {0x08,0x7E,0x09,0x01,0x02}, /* f */
  {0x18,0xA4,0xA4,0xA4,0x7C}, /* g */
  {0x7F,0x08,0x04,0x04,0x78}, /* h */
  {0x00,0x44,0x7D,0x40,0x00}, /* i */
  {0x40,0x80,0x84,0x7D,0x00}, /* j */
  {0x7F,0x10,0x28,0x44,0x00}, /* k */
  {0x00,0x41,0x7F,0x40,0x00}, /* l */
  {0x7C,0x04,0x18,0x04,0x78}, /* m */
  {0x7C,0x08,0x04,0x04,0x78}, /* n */
  {0x38,0x44,0x44,0x44,0x38}, /* o */
  {0xFC,0x24,0x24,0x24,0x18}, /* p */
  {0x18,0x24,0x24,0x18,0xFC}, /* q */
  {0x7C,0x08,0x04,0x04,0x08}, /* r */
  {0x48,0x54,0x54,0x54,0x20}, /* s */
  {0x04,0x3F,0x44,0x40,0x20}, /* t */
  {0x3C,0x40,0x40,0x20,0x7C}, /* u */
  {0x1C,0x20,0x40,0x20,0x1C}, /* v */
  {0x3C,0x40,0x30,0x40,0x3C}, /* w */
  {0x44,0x28,0x10,0x28,0x44}, /* x */
  {0x1C,0xA0,0xA0,0xA0,0x7C}, /* y */
  {0x44,0x64,0x54,0x4C,0x44}, /* z */
  {0x00,0x08,0x36,0x41,0x00}, /* { */
  {0x00,0x00,0x7F,0x00,0x00}, /* | */
  {0x00,0x41,0x36,0x08,0x00}, /* } */
  {0x08,0x04,0x08,0x10,0x08}, /* ~ */
};

static void lcd_spi_ready(void)
{
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY)) {
  }
}

static void lcd_a0(GPIO_PinState s)
{
  lcd_spi_ready();
  HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, s);
  /* CS is tied low: A0 must settle before the next SCLK. */
  for (volatile uint32_t i = 0; i < 48U; i++) {
  }
}

static void lcd_cmd(uint8_t c)
{
  lcd_a0(GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &c, 1, 10);
  lcd_spi_ready();
}

static void lcd_data_buf(uint8_t *p, uint16_t n)
{
  lcd_a0(GPIO_PIN_SET);
  HAL_SPI_Transmit(&hspi1, p, n, 50);
  lcd_spi_ready();
}

static void lcd_set_page_col(uint8_t page, uint8_t col)
{
  uint8_t x = (uint8_t)(col + LCD_X_OFFSET);
  lcd_cmd((uint8_t)(CMD_PAGE | (page & 0x07u)));
  lcd_cmd((uint8_t)(CMD_COL_MSB | (x >> 4)));
  lcd_cmd((uint8_t)(CMD_COL_LSB | (x & 0x0Fu)));
}

void lcd_init(void)
{
  /* DEBUG: slow SPI down to 750 kHz (12 MHz/16) while debugging the LCD;
   * once stable this block can be removed to return to CubeMX's 6 MHz. */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }

  /* Hard reset via PA4 after SPI is configured (SCK/MOSI at defined levels).
   * With CS tied to GND this is the only way to resynchronise the serial interface;
   * ST7567 needs /RST low ≥ 1 µs, use 5 ms to be safe. */
  GPIO_InitTypeDef rst = {
    .Pin = LCD_RST_Pin, .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &rst);
  lcd_delay(5);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  lcd_delay(10);

  lcd_cmd(CMD_SOFT_RESET);
  lcd_delay(10);
  lcd_cmd(CMD_DISPLAY_OFF);
  lcd_cmd(CMD_START_LINE | 0x00);
  lcd_cmd(LCD_SEG_REMAP);
  lcd_cmd(LCD_COM_DIR);
  lcd_cmd(CMD_BIAS_1_9);
  lcd_cmd(CMD_DISP_NORMAL);
  lcd_cmd(CMD_ALLPIX_OFF);

  /* Power-up: booster → regulator → follower */
  lcd_cmd(CMD_POWER | 0x04);
  lcd_delay(50);
  lcd_cmd(CMD_POWER | 0x06);
  lcd_delay(50);
  lcd_cmd(CMD_POWER | 0x07);
  lcd_delay(10);

  lcd_cmd(LCD_RR);
  lcd_cmd(CMD_EV);
  lcd_cmd(LCD_CONTRAST & 0x3Fu);

  lcd_cmd(CMD_BOOSTER);
  lcd_cmd(0x00);

  lcd_clear();
  lcd_update();
  lcd_cmd(CMD_DISPLAY_ON);

  /* DEBUG: turn all pixels on for ~600 ms to check LCD communication + supply
   * (independent of RAM/addressing). The screen must go fully dark, then clear again. */
  lcd_cmd(CMD_ALLPIX_ON);
  lcd_delay(600);
  lcd_cmd(CMD_ALLPIX_OFF);
}

void lcd_set_contrast(uint8_t ev)
{
  lcd_cmd(CMD_EV);
  lcd_cmd(ev & 0x3Fu);
}

void lcd_display_on(uint8_t on)
{
  lcd_cmd(on ? CMD_DISPLAY_ON : CMD_DISPLAY_OFF);
}

void lcd_invert(uint8_t on)
{
  lcd_cmd(on ? CMD_DISP_REVERSE : CMD_DISP_NORMAL);
}

void lcd_clear(void)
{
  memset(fb, 0, sizeof(fb));
}

void lcd_load_fullscreen(const uint8_t data[LCD_PAGES][LCD_WIDTH])
{
  memcpy(fb, data, sizeof(fb));
}

void lcd_update(void)
{
  lcd_update_pages(0, LCD_PAGES - 1);
}

void lcd_update_pages(uint8_t page0, uint8_t page1)
{
  uint8_t page;
  if (page1 >= LCD_PAGES) {
    page1 = LCD_PAGES - 1;
  }
  for (page = page0; page <= page1; page++) {
    lcd_set_page_col(page, 0);
    lcd_data_buf(fb[page], LCD_WIDTH);
  }
}

void lcd_set_pixel(int x, int y, uint8_t color)
{
  uint8_t mask;
  if ((unsigned)x >= LCD_WIDTH || (unsigned)y >= LCD_HEIGHT) {
    return;
  }
  mask = (uint8_t)(1u << (y & 7));
  if (color) {
    fb[y >> 3][x] |= mask;
  } else {
    fb[y >> 3][x] &= (uint8_t)~mask;
  }
}

void lcd_draw_hline(int x, int y, int w, uint8_t color)
{
  int i;
  for (i = 0; i < w; i++) {
    lcd_set_pixel(x + i, y, color);
  }
}

void lcd_draw_vline(int x, int y, int h, uint8_t color)
{
  int i;
  for (i = 0; i < h; i++) {
    lcd_set_pixel(x, y + i, color);
  }
}

void lcd_draw_rect(int x, int y, int w, int h, uint8_t color)
{
  if (w <= 0 || h <= 0) {
    return;
  }
  lcd_draw_hline(x, y, w, color);
  lcd_draw_hline(x, y + h - 1, w, color);
  lcd_draw_vline(x, y, h, color);
  lcd_draw_vline(x + w - 1, y, h, color);
}

void lcd_fill_rect(int x, int y, int w, int h, uint8_t color)
{
  int iy, ix;
  for (iy = 0; iy < h; iy++) {
    for (ix = 0; ix < w; ix++) {
      lcd_set_pixel(x + ix, y + iy, color);
    }
  }
}

void lcd_fill_circle(int cx, int cy, int r, uint8_t color)
{
  int y, x;
  int r2;

  if (r < 0) {
    return;
  }
  r2 = r * r;
  for (y = -r; y <= r; y++) {
    for (x = -r; x <= r; x++) {
      if ((x * x + y * y) <= r2) {
        lcd_set_pixel(cx + x, cy + y, color);
      }
    }
  }
}

void lcd_draw_char(int x, int y, char c)
{
  const uint8_t *g;
  int col, row;
  if (c < 0x20 || c > 0x7E) {
    c = '?';
  }
  g = font5x7[c - 0x20];
  for (col = 0; col < 5; col++) {
    uint8_t bits = g[col];
    for (row = 0; row < 8; row++) {
      if (bits & (1u << row)) {
        lcd_set_pixel(x + col, y + row, 1);
        lcd_set_pixel(x + col + 1, y + row, 1);
      }
    }
  }
}

void lcd_draw_string(int x, int y, const char *s)
{
  while (*s) {
    lcd_draw_char(x, y, *s++);
    x += 8;
  }
}


void lcd_draw_char_inv(int x, int y, char c)
{
  const uint8_t *g;
  int col, row;
  if (c < 0x20 || c > 0x7E) c = '?';
  g = font5x7[c - 0x20];
  for (col = 0; col < 5; col++) {
    uint8_t bits = g[col];
    for (row = 0; row < 8; row++) {
      if (bits & (1u << row)) {
        lcd_set_pixel(x + col, y + row, 0);
        lcd_set_pixel(x + col + 1, y + row, 0);
      }
    }
  }
}

void lcd_draw_string_inv(int x, int y, const char *s)
{
  while (*s) {
    lcd_draw_char_inv(x, y, *s++);
    x += 8;
  }
}

void lcd_draw_uint(int x, int y, uint32_t v)
{
  char buf[11];
  int i = 10;
  buf[10] = 0;
  do {
    buf[--i] = (char)('0' + (v % 10u));
    v /= 10u;
  } while (v && i > 0);
  lcd_draw_string(x, y, &buf[i]);
}
