# LCD ST7567 Reference

Reference notes for the ST7567 128x64 COG LCD module, reusable in other STM32 projects.

## Panel

- **IC**: ST7567 (controller on glass)
- **Panel**: Xuyang COG12864-14-1, 1.9", 128x64 pixels, monochrome
- **Interface**: SPI (mode 0: CPOL=0, CPHA=0), MSB first
- **Supply**: 3.3 V logic, internal booster generates V0 for the LCD
- **Duty/Bias**: 1/65 duty, 1/9 bias (`CMD_BIAS_1_9 = 0xA2`)

## Hardware connections

| LCD pin | Function | Notes |
|---------|----------|-------|
| SCK | SPI clock | SPI1_SCK |
| SDA/MOSI | SPI data | SPI1_MOSI |
| A0 | Command/data select | GPIO output: LOW = command, HIGH = data |
| /RST | Hardware reset | GPIO output: active low, 10 ms pulse |
| CS | Chip select | **Tied to GND** (always selected) |
| V0, XV0 | Booster output | Booster capacitor between the two pins |
| VG, VSS | Booster ground | Booster capacitor between the two pins |

### Booster capacitors (IMPORTANT)

V0–XV0 and VG–VSS need **X7R 0.1–1 µF** capacitors (ceramic, low ESR). The wrong capacitor causes LCD flicker:
- Wrong value or type → V0 unstable → the whole display flickers every ~200 ms
- SPI bursts interfering with the booster → V0 dips briefly → random flashes during fast updates
- Symptom: content is correct and not garbled, but blinks off and recovers by itself

**Diagnosis**: if a static test (no SPI after boot) still flickers → definitely the capacitors. If it only flickers during updates → marginal capacitors, SPI bursts make V0 dip.

## Driver configuration

```c
/* lcd_st7567.h */
#define LCD_WIDTH      128
#define LCD_HEIGHT     64
#define LCD_PAGES      (LCD_HEIGHT / 8)   /* 8 pages */
#define LCD_X_OFFSET   0                  /* DDRAM has 132 columns, panel starts at 0 */

#define LCD_CONTRAST   10       /* EV 0..63, EV=10 gives good contrast on this panel */
#define LCD_RR         0x25     /* Regulator ratio 5.5 (0x20..0x27 → 3.0..6.5) */
#define LCD_SEG_REMAP  0xA0     /* 0xA0 normal, 0xA1 mirror horizontal */
#define LCD_COM_DIR    0xC8     /* 0xC0 normal, 0xC8 mirror vertical */
```

## Initialisation sequence

```
1. Configure the /RST GPIO (output push-pull)
2. Hardware reset: /RST LOW 10 ms → HIGH 10 ms
3. Kick the WWDG (if used)
4. Soft reset (0xE2), delay 10 ms
5. Display OFF (0xAE)
6. Start line = 0 (0x40)
7. SEG remap (0xA0 or 0xA1)
8. COM direction (0xC0 or 0xC8)
9. Bias 1/9 (0xA2)
10. Display normal (0xA6, not inverted)
11. All pixels off (0xA4)
12. Power up booster → regulator → follower:
    - 0x2C, delay 50 ms, kick WWDG
    - 0x2E, delay 50 ms, kick WWDG
    - 0x2F, delay 10 ms, kick WWDG
13. Regulator ratio (LCD_RR)
14. EV set: 0x81, LCD_CONTRAST
15. Booster ratio: 0xF8, 0x00
16. Clear the framebuffer, update the display
17. Display ON (0xAF)
```

Total init time: ~140 ms (mostly the power-up sequence). With a WWDG (~131 ms window), kicks must be interleaved between delays.

## Framebuffer & rendering

- Framebuffer: `uint8_t fb[8][128]` — 8 pages × 128 columns = 1024 bytes
- Each page = 8 vertical pixels, LSB = top pixel
- Set pixel: `fb[y >> 3][x] |= (1u << (y & 7))`
- Clear pixel: `fb[y >> 3][x] &= ~(1u << (y & 7))`

### Updating the display

```c
void lcd_update_pages(uint8_t page0, uint8_t page1)
{
  for (page = page0; page <= page1; page++) {
    lcd_set_page_col(page, 0);     /* set page + column 0 */
    lcd_data_buf(fb[page], 128);   /* SPI transmit 128 bytes */
  }
}
```

- `lcd_update()` = update all 8 pages
- Partial updates (`lcd_update_pages(2, 5)`) reduce SPI traffic when only a middle region changes

### A0 settle time

CS is tied to GND, so there is no CS toggle between command and data. A0 must settle before the first SCLK edge:

```c
static void lcd_a0(GPIO_PinState s)
{
  lcd_spi_ready();                    /* wait for SPI to finish */
  HAL_GPIO_WritePin(A0_PORT, A0_PIN, s);
  for (volatile uint32_t i = 0; i < 48U; i++) {}  /* settle delay */
}
```

48 loop iterations @ 12 MHz ≈ 4 µs, enough for the ST7567.

## Font

- 5x7 font, column-major, LSB = top pixel
- ASCII 0x20..0x7E (space..tilde)
- Bold: each column is drawn as 2 adjacent pixels (col, col+1)
- Advance: 8 px per character (5 data + 1 bold + 2 spacing)

### Inverted text (white on black)

Used for inverted headers or blinking:

```c
void lcd_draw_char_inv(int x, int y, char c)
{
  /* Same as lcd_draw_char but lcd_set_pixel(..., 0) instead of 1 */
  /* Requires: the background area is already filled black */
}
```

Pattern: `lcd_fill_rect(x, y, w, h, 1)` then `lcd_draw_string_inv(...)`.

## Drawing primitives

| Function | Description |
|----------|-------------|
| `lcd_clear()` | Clear the framebuffer (memset 0) |
| `lcd_update()` | Write the whole framebuffer to the display |
| `lcd_set_pixel(x, y, color)` | Set/clear one pixel |
| `lcd_draw_hline(x, y, w, color)` | Horizontal line |
| `lcd_draw_vline(x, y, h, color)` | Vertical line |
| `lcd_draw_rect(x, y, w, h, color)` | Rectangle outline |
| `lcd_fill_rect(x, y, w, h, color)` | Filled rectangle |
| `lcd_fill_circle(cx, cy, r, color)` | Filled circle |
| `lcd_draw_char(x, y, c)` | One character (bold, 8 px advance) |
| `lcd_draw_string(x, y, s)` | String |
| `lcd_draw_char_inv(x, y, c)` | Inverted character (white on black) |
| `lcd_draw_string_inv(x, y, s)` | Inverted string |
| `lcd_draw_uint(x, y, v)` | Unsigned integer |

## SPI speed

- CubeMX default: `SPI_BAUDRATEPRESCALER_2` → 6 MHz @ 12 MHz Fsys
- Debug prescaler in `lcd_init()`: `SPI_BAUDRATEPRESCALER_16` → 750 kHz
- Once stable, remove the override in `lcd_init()` to return to 6 MHz

## Debugging LCD flicker

1. **Static test**: draw once, then loop doing nothing → rules out software
2. **Counter test**: update every 1 s vs 100 ms → if flicker rate scales, SPI is triggering V0 dips
3. **Uptime counter**: if the counter keeps running through a flicker → the MCU did not reset → electrical issue
4. **Correct content after a flicker**: V0 sag (self-recovering), not SPI corruption (which would need DISPLAY_ON to recover)
5. **All-pixels-on test** (`0xA5`): bypasses RAM, tests the booster directly
