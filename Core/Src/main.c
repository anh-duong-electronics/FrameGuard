/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include "ads1015.h"
#include "config_store.h"
#include "lcd_st7567.h"
#include "app_debug.h"
#include "wire_probe.h"
#include "modbus_rtu.h"
#include "mb_port_usart1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VAC_ZERO_V          0.05f     /* V_AC below this is treated as 0 — less decimal noise on the LCD */
#define PROBE_SETTLE_MS     3         /* wait for the switch to return to measure position before reading ADS */
#define LED_ENABLE          1         /* 1 = status LEDs on, 0 = off */
#define PRINT_INTERVAL_MS   1000UL    /* LCD + RTT log update period */
#define RS485_OFFLINE_MS    10000UL
#define CFG_MSG_MS          3000UL    /* how long a config-command result stays on the LCD ID row */
#define MB_REG_COUNT        18        /* map 1-based: addr 1..17, addr 0 reserved — docs/rs485-modbus-protocol.md */
#define LCD_PAGE_INTERVAL   5000UL
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */
static app_config_t g_cfg = {
  .device_id = 1001, .modbus_addr = 1, .ch_count = ADS_NUM_CHANNELS, .alert_mv = 5000,
  .wire_mvpp = PROBE_OPEN_MVPP_DEFAULT, .k_factor = { 1.0f, 1.0f, 1.0f, 1.0f },
};

/* ACTIVE alarm threshold (mV). Boots from g_cfg.alert_mv; FC06 overrides it
 * at runtime (RAM-only — the persistent value is managed by the master/UART2). Every threshold
 * comparison must go through is_leak() — never compare directly, to avoid mixing V and mV. */
static uint16_t g_alert_mv = 5000;
static uint16_t mb_heartbeat = 0;

static uint32_t last_rs485_poll  = 0;
static bool     lcd_page         = false;
static uint32_t last_page_switch = 0;

static float   vac_ch[ADS_NUM_CHANNELS] = { 0 };
static float   raw_ch[ADS_NUM_CHANNELS] = { 0 };   /* vrms_total — used for table calibration */
static float   vac_uncal[ADS_NUM_CHANNELS] = { 0 }; /* VAC before factor K — source for v=/kN= calibration */

/* Per-channel calibration factor; old configs (previous layout) stored 0 → treated as 1.0 */
static float kf(int i)
{
  float k = g_cfg.k_factor[i];
  return (k > 0.0f) ? k : 1.0f;
}
static probe_state_t wire_ch[ADS_NUM_CHANNELS];     /* confirmed wire-check result */
static uint16_t      wire_mvpp[ADS_NUM_CHANNELS];   /* latest wire-check amplitude, diagnostics */

/* Per-channel verdict for LED/LCD/RS485. Fixed numeric codes, used on the wire. */
typedef enum {
  CH_INIT = 0,   /* wire not yet confirmed after startup */
  CH_OK,         /* wire connected, no leakage */
  CH_LEAK,       /* leakage (LCD "NG") */
  CH_WIRE_NG,    /* sensing wire broken / open */
  CH_ERR,        /* ADS1015 not responding */
  CH_DISABLED,   /* channel beyond the configured count (qty=) — not measured, LED off, LCD blank */
} ch_status_t;

/* A machine group has 1..4 channels (qty= command): only P1..Pn are active */
static bool ch_enabled(int i) { return i < g_cfg.ch_count; }
static bool    wwdg_started = false;

#if LED_ENABLE
static GPIO_TypeDef *const LED_G_PORT[ADS_NUM_CHANNELS] =
  { L_GREEN4_GPIO_Port, L_GREEN3_GPIO_Port, L_GREEN2_GPIO_Port, L_GREEN1_GPIO_Port };
static const uint16_t LED_G_PIN[ADS_NUM_CHANNELS] =
  { L_GREEN4_Pin, L_GREEN3_Pin, L_GREEN2_Pin, L_GREEN1_Pin };
static GPIO_TypeDef *const LED_R_PORT[ADS_NUM_CHANNELS] =
  { L_RED4_GPIO_Port, L_RED3_GPIO_Port, L_RED2_GPIO_Port, L_RED1_GPIO_Port };
static const uint16_t LED_R_PIN[ADS_NUM_CHANNELS] =
  { L_RED4_Pin, L_RED3_Pin, L_RED2_Pin, L_RED1_Pin };
/* Desired LED state — written by the main loop, read by the SysTick ISR to update outputs */
static volatile bool g_green_on[ADS_NUM_CHANNELS] = { 0 };
static volatile bool g_red_on[ADS_NUM_CHANNELS]   = { 0 };
static volatile bool g_red_hb[ADS_NUM_CHANNELS]   = { 0 };   /* red heartbeat blink: wire broken */
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_WWDG_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void poll_serial(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static char     cfg_msg[21];
static uint32_t cfg_msg_until = 0;
#define CFG_PAGE_MS 5000u
static uint32_t cfg_page_until = 0;

static void cfg_show(const char *s)
{
  strncpy(cfg_msg, s, sizeof(cfg_msg) - 1);
  cfg_msg[sizeof(cfg_msg) - 1] = '\0';
  cfg_msg_until = HAL_GetTick() + CFG_MSG_MS;
}

/* Window watchdog: may only be refreshed once the counter has dropped below Window
 * (~131 ms after the previous reload); refreshing earlier also resets the MCU.
 * Call it liberally everywhere — it ignores calls made before the window opens. */
void app_wwdg_kick(void)
{
  if (!wwdg_started) return;
  if ((hwwdg.Instance->CR & WWDG_CR_T) < hwwdg.Init.Window)
    HAL_WWDG_Refresh(&hwwdg);
}

/* The ONLY threshold comparison point — LEDs/LCD/Modbus STATUS all go through here */
static bool is_leak(int i)
{
  return vac_ch[i] >= 0 && vac_ch[i] * 1000.0f > (float)g_alert_mv;
}

/* Leakage ranks above a broken wire: a measured leakage voltage means the wire is connected. */
static ch_status_t ch_status(int i)
{
  if (!ch_enabled(i))                   return CH_DISABLED;
  if (vac_ch[i] < 0)                    return CH_ERR;
  if (is_leak(i))                       return CH_LEAK;
  if (wire_ch[i] == PROBE_OPEN)         return CH_WIRE_NG;
  if (wire_ch[i] == PROBE_UNKNOWN)      return CH_INIT;
  return CH_OK;
}

/* LEDs share a common anode, active LOW: pin RESET (pulled low) = on.
 * OK: green; leakage: red; wire broken: red heartbeat blink; ERR/INIT: both off. */
static void set_machine_led(int i, ch_status_t st)
{
#if LED_ENABLE
  g_green_on[i] = (st == CH_OK);
  g_red_on[i]   = (st == CH_LEAK);
  g_red_hb[i]   = (st == CH_WIRE_NG);
#else
  (void)i; (void)st;
#endif
}

/* Called from SysTick_Handler (1 kHz): updates the status LEDs.
 * Active-low LEDs (common anode, resistor-limited): RESET = on, SET = off.
 * Linux-style heartbeat for the red blink when the wire is broken. */
void app_led_pwm_tick(void)
{
#if LED_ENABLE
  static uint16_t hb = 0;
  if (++hb >= 1000) hb = 0;
  bool hb_on = (hb < 70) || (hb >= 170 && hb < 240);
  for (int i = 0; i < ADS_NUM_CHANNELS; i++) {
    bool red = g_red_on[i] || (g_red_hb[i] && hb_on);
    HAL_GPIO_WritePin(LED_G_PORT[i], LED_G_PIN[i], g_green_on[i] ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_R_PORT[i], LED_R_PIN[i], red             ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }
#endif
}

void app_rs485_poll_received(void) { last_rs485_poll = HAL_GetTick(); }

static bool    blink_state = false;
static uint32_t last_blink  = 0;


/* Layout (no header): 4 data rows + 1 ID row
 *  y=0  ───── outer border top
 *  y=1..12   M1 row (12px content)
 *  y=13 ───── divider
 *  y=14..25  M2 row
 *  y=26 ───── divider
 *  y=27..38  M3 row
 *  y=39 ───── divider
 *  y=40..51  M4 row
 *  y=52 ───── divider (above ID)
 *  y=53..62  ID row (10px, same as spec v2)
 *  y=63 ───── outer border bottom
 */
/* Append "<label><number>" to b at position n, return the new position */
static int fmt_label_u(char *b, int n, const char *label, uint32_t v)
{
  while (*label) b[n++] = *label++;
  char t[6];
  int  c = 0;
  do { t[c++] = (char)('0' + v % 10); v /= 10; } while (v);
  while (c) b[n++] = t[--c];
  b[n] = '\0';
  return n;
}

/* Settings page ("config" command): only the configured values, 5 rows */
static void lcd_show_settings(void)
{
  char b[17];
  int  n;
  lcd_draw_rect(0, 0, 128, 64, 1);

  n = fmt_label_u(b, 0, "ID:", g_cfg.device_id);
  n = fmt_label_u(b, n, " RS:", g_cfg.modbus_addr);
  lcd_draw_string(4, 3, b);

  n = fmt_label_u(b, 0, "THR:", g_cfg.alert_mv);
  n = fmt_label_u(b, n, " Qty:", g_cfg.ch_count);
  lcd_draw_string(4, 15, b);

  fmt_label_u(b, 0, "WIRE:", g_cfg.wire_mvpp);
  lcd_draw_string(4, 27, b);

  /* K×1000 — "K1:1000 K2:1000" matches how the kN= command displays it */
  n = fmt_label_u(b, 0, "K1:", (uint32_t)(kf(0) * 1000.0f + 0.5f));
  n = fmt_label_u(b, n, " K2:", (uint32_t)(kf(1) * 1000.0f + 0.5f));
  lcd_draw_string(4, 39, b);

  n = fmt_label_u(b, 0, "K3:", (uint32_t)(kf(2) * 1000.0f + 0.5f));
  n = fmt_label_u(b, n, " K4:", (uint32_t)(kf(3) * 1000.0f + 0.5f));
  lcd_draw_string(4, 51, b);
}

static void lcd_show_channels(bool any_alert)
{
  char buf[16];
  int i;
  lcd_clear();

  uint32_t now = HAL_GetTick();
  if (now - last_blink >= 500) {
    last_blink = now;
    blink_state = !blink_state;
  }

  if ((int32_t)(cfg_page_until - now) > 0) {
    lcd_show_settings();
    lcd_update();
    return;
  }

  lcd_draw_rect(0, 0, 128, 64, 1);

  lcd_draw_hline(0, 13, 128, 1);
  lcd_draw_hline(0, 26, 128, 1);
  lcd_draw_hline(0, 39, 128, 1);
  lcd_draw_hline(0, 52, 128, 1);

  lcd_draw_vline(28, 0, 53, 1);
  lcd_draw_vline(88, 0, 53, 1);

  static const int row_y[]  = {1, 14, 27, 40};
  static const int row_ye[] = {12, 25, 38, 51};

  for (i = 0; i < ADS_NUM_CHANNELS; i++) {
    int cy = row_y[i];
    int ty = cy + 2;

    if (!ch_enabled(i)) continue;        /* disabled channel: leave the row blank */

    lcd_draw_char(7, ty, 'P');
    lcd_draw_char(15, ty, (char)('1' + i));

    ch_status_t st = ch_status(i);
    if (vac_ch[i] < 0) {
      lcd_draw_string(32, ty, "----");
      lcd_draw_string(98, ty, "ERR");
    } else if (st == CH_WIRE_NG) {
      /* Wire broken: do not print 0.00V (easily mistaken for "no leakage"); both cells show
       * evenly spaced blinking 2x2 dots: voltage cell 29..87 5 dots every 12 px, status cell 89..126 4 dots every 9 px */
      if (blink_state) {
        for (int dx = 34; dx <= 82; dx += 12) lcd_fill_rect(dx, cy + 5, 2, 2, 1);
        for (int dx = 94; dx <= 121; dx += 9) lcd_fill_rect(dx, cy + 5, 2, 2, 1);
      }
    } else {
      int32_t s = (int32_t)(vac_ch[i] * 100.0f + 0.5f);
      if (s > 9999) s = 9999;
      int32_t ip = s / 100, fp = s % 100;
      int n = 0;
      if (ip >= 10) buf[n++] = (char)('0' + ip / 10);
      buf[n++] = (char)('0' + ip % 10);
      buf[n++] = '.';
      buf[n++] = (char)('0' + fp / 10);
      buf[n++] = (char)('0' + fp % 10);
      buf[n] = '\0';
      lcd_draw_string(32, ty, buf);
      lcd_draw_char(32 + n * 8, ty, 'V');

      const char *txt = st == CH_LEAK ? "NG" : st == CH_INIT ? "..." : "OK";
      if (st == CH_LEAK && blink_state) {
        lcd_fill_rect(89, cy, 38, row_ye[i] - cy + 1, 1);
        lcd_draw_string_inv(98, ty, txt);
      } else {
        lcd_draw_string(98, ty, txt);
      }
    }
  }

  /* ID row — 2-page alternating display */
  {
    if (now - last_page_switch >= LCD_PAGE_INTERVAL) {
      lcd_page = !lcd_page;
      last_page_switch = now;
    }

    if ((int32_t)(cfg_msg_until - now) > 0) {
      /* Config-command feedback, inverted to stand out, centred */
      int n = (int)strlen(cfg_msg);
      lcd_fill_rect(1, 53, 126, 10, 1);
      if (n <= 16) {
        lcd_draw_string_inv(64 - n * 4, 55, cfg_msg);
      } else {
        /* Long strings (5-command usage = 20 chars): 5 px glyphs drawn at 6 px pitch → 120 px */
        int x = 64 - n * 3;
        for (int k = 0; k < n; k++, x += 6) lcd_draw_char_inv(x, 55, cfg_msg[k]);
      }
    } else
    if (!lcd_page) {
      /* Page 1: ID:1001-RS:001 (14 chars, 112px, x=8) */
      uint16_t id = g_cfg.device_id;
      buf[0]  = 'I'; buf[1] = 'D';
      buf[2]  = blink_state ? ':' : ' ';
      buf[3]  = (char)('0' + (id / 1000) % 10);
      buf[4]  = (char)('0' + (id / 100) % 10);
      buf[5]  = (char)('0' + (id / 10) % 10);
      buf[6]  = (char)('0' + id % 10);
      buf[7]  = '-';
      buf[8]  = 'R'; buf[9] = 'S'; buf[10] = ':';
      buf[11] = (char)('0' + (g_cfg.modbus_addr / 100) % 10);
      buf[12] = (char)('0' + (g_cfg.modbus_addr / 10) % 10);
      buf[13] = (char)('0' + g_cfg.modbus_addr % 10);
      buf[14] = '\0';
      lcd_draw_string(8, 55, buf);
    } else {
      /* Page 2: LINK OK or OFFLINE M:SS */
      uint32_t elapsed = now - last_rs485_poll;
      bool offline = (elapsed > RS485_OFFLINE_MS);

      if (offline) {
        uint32_t secs = elapsed / 1000;
        uint32_t mm = secs / 60; if (mm > 9) mm = 9;
        uint32_t ss = secs % 60;
        buf[0] = 'O'; buf[1] = 'F'; buf[2] = 'F';
        buf[3] = 'L'; buf[4] = 'I'; buf[5] = 'N'; buf[6] = 'E'; buf[7] = ' ';
        buf[8] = (char)('0' + mm);
        buf[9] = ':';
        buf[10] = (char)('0' + ss / 10);
        buf[11] = (char)('0' + ss % 10);
        buf[12] = '\0';
        /* 12 chars = 96px → x=16 */
        lcd_fill_rect(1, 53, 126, 10, 1);
        lcd_draw_string_inv(16, 55, buf);
      } else {
        /* 7 chars = 56px → x=36 */
        lcd_draw_string(36, 55, "LINK OK");
      }
    }
  }

  (void)any_alert;
  lcd_update();
}

/* ---- Modbus RTU slave: 1-based register map, addr = channel number for clarity
 * (docs/rs485-modbus-protocol.md). Each channel's state is read from CH_STATUS —
 * there is no combined bitmap (it duplicated CH_STATUS). ----
 *  0    reserved (reads 0)
 *  1–4  V_AC channels P1–P4 (unit 0.01 V — matches LCD X.XX; I2C error → 0xFFFF, valid values clamp at 0xFFFE)
 *  5    DEVICE_ID                    6  ALERT_THRESHOLD mV (FC06 ghi, RAM-only)
 *  7    FW_HEARTBEAT                 8–11  CH_STATUS channels P1–P4 (ch_status_t)
 *  12–15 WIRE_MVPP channels P1–P4 (wire-check amplitude)
 *  16   WIRE_THRESHOLD (wire-break threshold shared by 4 channels, read-only)
 *  17   CH_COUNT 1..4 (channels in the group, writable via FC06 — like qty=, saved to flash;
 *       disabled channel: VAC=0, CH_STATUS=5, WIRE_MVPP=0) */
static uint16_t mb_app_reg_read(uint16_t addr)
{
  switch (addr) {
  case 1: case 2: case 3: case 4: {
    int i = addr - 1;
    if (!ch_enabled(i)) return 0;                  /* disabled channel (qty=) — CH_STATUS = 5 */
    if (vac_ch[i] < 0) return 0xFFFF;              /* I2C error — before any conversion */
    uint32_t cv = (uint32_t)(vac_ch[i] * 100.0f + 0.5f);   /* 0.01 V/LSB */
    return (cv > 0xFFFE) ? 0xFFFE : (uint16_t)cv;
  }
  case 5: return g_cfg.device_id;
  case 6: return g_alert_mv;
  case 7: return mb_heartbeat;
  case 8: case 9: case 10: case 11:
    return (uint16_t)ch_status(addr - 8);
  case 12: case 13: case 14: case 15:
    return wire_mvpp[addr - 12];
  case 16: return g_cfg.wire_mvpp;                 /* wire-break threshold (read-only; set with wire=) */
  case 17: return g_cfg.ch_count;                  /* channels in the group (qty= command) */
  default: return 0;                               /* addr 0 reserved; the core rejects addr ≥ MB_REG_COUNT */
  }
}

static void cfg_save_show(const char *label, uint32_t v)
{
  app_wwdg_kick();
  bool saved = config_save(&g_cfg);
  app_wwdg_kick();
  char m[20];
  int  n = 0;
  while (label[n]) { m[n] = label[n]; n++; }
  char tmp[6];
  int  t = 0;
  do { tmp[t++] = (char)('0' + v % 10); v /= 10; } while (v);
  while (t) m[n++] = tmp[--t];
  m[n] = '\0';
  strcat(m, saved ? " SAVED" : " FLASH!");
  cfg_show(m);
}

/* FC06 can write 2 registers:
 *  - 6  ALERT_THRESHOLD: RAM-only — the persistent value lives in flash (thr= command);
 *       a master that wants its own threshold re-writes it on every poll
 *  - 17 CH_COUNT: like the qty= UART2 command — saved to flash, applied on the next scan */
static mb_wr_result_t mb_app_reg_write(uint16_t addr, uint16_t val)
{
  switch (addr) {
  case 6:
    if (val < 100 || val > 60000) return MB_WR_BAD_VALUE;
    g_alert_mv = val;
    return MB_WR_OK;
  case 17:
    if (val < 1 || val > ADS_NUM_CHANNELS) return MB_WR_BAD_VALUE;
    g_cfg.ch_count = (uint8_t)val;       /* next scan: channels > val are skipped, data cleared */
    cfg_save_show("QTY:", val);          /* write flash + LCD feedback, same path as qty= */
    return MB_WR_OK;
  default:
    return MB_WR_BAD_ADDR;
  }
}

static const mb_port_t MB_PORT = {
  .get_tick_ms = HAL_GetTick,
  .tx_start    = mb_port_usart1_tx_start,
  .tx_busy     = mb_port_usart1_tx_busy,
  .reg_read    = mb_app_reg_read,
  .reg_write   = mb_app_reg_write,
  .on_reply_ok = app_rs485_poll_received,
};

void app_comm_poll(void) {
  mb_poll();
}

/* ---- Config commands over USART2 RX (no TX — result shown on the LCD for 3 s):
 * id=<1..9999> | addr=<1..247> | thr=<100..60000> (mV) | wire=<100..3000> (mVpp)
 * | qty=<1..4> (channel count) | v=<500..24000> (reference mV being applied → calibrate K on all 4 channels)
 * | k1=..k4=<500..24000> (calibrate K on one channel). Saved to flash immediately.
 * | config (no argument): show the settings page for 5 s. ---- */

/* Case-insensitive prefix match; returns a pointer past the prefix or NULL */
static const char *cmd_prefix(const char *p, const char *kw)
{
  while (*kw) {
    char c = *p, k = *kw;
    if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
    if (c != k) return NULL;
    p++; kw++;
  }
  return p;
}

/* Parse a decimal number; -1 if empty or containing invalid characters */
static int32_t cmd_num(const char *d)
{
  if (*d < '0' || *d > '9') return -1;
  int32_t v = 0;
  for (; *d >= '0' && *d <= '9'; d++) {
    v = v * 10 + (*d - '0');
    if (v > 100000) return -1;
  }
  return (*d == '\0') ? v : -1;
}


/* Compute K for channel i from the reference voltage being applied (V). Rejects disabled channels,
 * readings that are too small (noise divided by ~0), or K outside 0.5..2.0 (wrong reference applied). */
static bool cal_apply(int i, float target_v)
{
  if (!ch_enabled(i) || vac_uncal[i] < 0.2f) return false;
  float k = target_v / vac_uncal[i];
  if (k < 0.5f || k > 2.0f) return false;
  g_cfg.k_factor[i] = k;
  return true;
}


static void handle_command(char *p)
{
  while (*p == ' ' || *p == '\t') p++;
  const char *d;

  if ((d = cmd_prefix(p, "id=")) != NULL) {
    int32_t v = cmd_num(d);
    if (v >= 1 && v <= 9999) {
      g_cfg.device_id = (uint16_t)v;
      cfg_save_show("ID:", (uint32_t)v);
    } else {
      cfg_show("ID 1..9999");
    }
    return;
  }

  if ((d = cmd_prefix(p, "addr=")) != NULL) {
    int32_t v = cmd_num(d);
    if (v >= 1 && v <= 247) {
      g_cfg.modbus_addr = (uint8_t)v;
      mb_set_addr((uint8_t)v);
      cfg_save_show("ADDR:", (uint32_t)v);
    } else {
      cfg_show("ADDR 1..247");
    }
    return;
  }

  if ((d = cmd_prefix(p, "thr=")) != NULL) {
    int32_t v = cmd_num(d);
    if (v >= 100 && v <= 60000) {
      g_cfg.alert_mv = (uint16_t)v;
      g_alert_mv     = (uint16_t)v;    /* RAM + flash together — thr= is the persistent value */
      cfg_save_show("THR:", (uint32_t)v);
    } else {
      cfg_show("THR 100..60000");
    }
    return;
  }

  if ((d = cmd_prefix(p, "wire=")) != NULL) {
    int32_t v = cmd_num(d);
    if (v >= 100 && v <= 3000) {
      g_cfg.wire_mvpp = (uint16_t)v;
      probe_set_open_mvpp((uint16_t)v);  /* effective from the next wire-check window */
      cfg_save_show("WIRE:", (uint32_t)v);
    } else {
      cfg_show("WIRE 100..3000");
    }
    return;
  }

  if ((d = cmd_prefix(p, "qty=")) != NULL) {
    int32_t v = cmd_num(d);
    if (v >= 1 && v <= ADS_NUM_CHANNELS) {
      g_cfg.ch_count = (uint8_t)v;       /* next scan: channels > v are skipped, data cleared */
      cfg_save_show("QTY:", (uint32_t)v);
    } else {
      cfg_show("QTY 1..4");
    }
    return;
  }

  if ((d = cmd_prefix(p, "v=")) != NULL) {
    int32_t v = cmd_num(d);
    if (v >= 500 && v <= 24000) {
      int n = 0;
      for (int i = 0; i < ADS_NUM_CHANNELS; i++)
        if (cal_apply(i, (float)v / 1000.0f)) n++;
      if (n) cfg_save_show("CAL:", (uint32_t)n);   /* "CAL:4 SAVED" = number of channels calibrated */
      else   cfg_show("CAL FAIL");
    } else {
      cfg_show("V 500..24000");
    }
    return;
  }

  if ((p[0] == 'k' || p[0] == 'K') && p[1] >= '1' && p[1] <= '4' && p[2] == '=') {
    int32_t v = cmd_num(p + 3);
    int ch = p[1] - '1';
    if (v >= 500 && v <= 24000) {
      if (cal_apply(ch, (float)v / 1000.0f)) {
        char lbl[4] = { 'K', p[1], ':', '\0' };
        /* show K×1000: "K1:1023 SAVED" reads as 1.023 */
        cfg_save_show(lbl, (uint32_t)(g_cfg.k_factor[ch] * 1000.0f + 0.5f));
      } else {
        char msg[12] = "CAL P1 FAIL";
        msg[5] = p[1];
        cfg_show(msg);
      }
    } else {
      cfg_show("K 500..24000");
    }
    return;
  }

  if ((d = cmd_prefix(p, "config")) != NULL && *d == '\0') {
    cfg_page_until = HAL_GetTick() + CFG_PAGE_MS;
    return;
  }


  cfg_show("id addr qty v kN");
}

/* USART2 has no FIFO (1-byte RDR) and the measurement loop has stretches that do not poll
 * for tens of ms → a command line sent in one burst (paste/script, 9 bytes ≈ 0.8 ms) would overrun.
 * Receive via RXNE interrupt into a small ring; poll_serial() only drains the ring and parses. */
#define U2_RING_SIZE 32u                     /* power of 2 */
static volatile uint8_t u2_ring[U2_RING_SIZE];
static volatile uint8_t u2_head, u2_tail;    /* head: written by ISR; tail: read by poll_serial */

void app_usart2_isr(void)
{
  uint32_t isr = USART2->ISR;
  USART2->ICR = isr & (USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF);
  if (isr & USART_ISR_RXNE_RXFNE) {
    uint8_t c = (uint8_t)USART2->RDR;
    uint8_t next = (uint8_t)((u2_head + 1u) & (U2_RING_SIZE - 1u));
    if (next != u2_tail && !(isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE))) {
      u2_ring[u2_head] = c;
      u2_head = next;
    }
  }
}

static void poll_serial(void)
{
  static char    sbuf[24];
  static uint8_t len = 0;

  while (u2_tail != u2_head) {
    char c = (char)u2_ring[u2_tail];
    u2_tail = (uint8_t)((u2_tail + 1u) & (U2_RING_SIZE - 1u));
    if (c == '\n' || c == '\r') {
      if (len == 0) continue;
      sbuf[len] = '\0';
      handle_command(sbuf);
      len = 0;
    } else if (len < sizeof(sbuf) - 1) {
      sbuf[len++] = c;
    } else {
      len = 0;
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_WWDG_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  APP_LOG_INIT();
#if APP_DEBUG
  /* J-Link halts the core (flashing, RTT attach) but the WWDG keeps counting → spurious WWDG reset.
   * Freeze the WWDG while the core is halted; not needed in production. */
  __HAL_RCC_DBGMCU_CLK_ENABLE();
  __HAL_DBGMCU_FREEZE_WWDG();
#endif
  wwdg_started = true;

  /* Load config from flash (if present); V conversion uses the static calibration table in ads1015.c */
  bool cfg_loaded = config_load(&g_cfg);
  g_alert_mv = g_cfg.alert_mv;
  probe_set_open_mvpp(g_cfg.wire_mvpp);

  /* Modbus RTU slave on USART1: core + bare-register port.
   * From here on, no HAL UART TX API may be used on huart1 (one UART, one owner). */
  mb_init(&MB_PORT, g_cfg.modbus_addr, MB_REG_COUNT);
  mb_port_usart1_init();

  APP_LOG("[boot] id=%u addr=%u thr=%umV cfg=%s rst=%s%s%s%s\r\n",
          g_cfg.device_id, g_cfg.modbus_addr, g_alert_mv, cfg_loaded ? "flash" : "defaults",
          __HAL_RCC_GET_FLAG(RCC_FLAG_PWRRST)  ? "POR "  : "",
          __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)  ? "PIN "  : "",
          __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)  ? "SFT "  : "",
          __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) ? "WWDG " : "");
  __HAL_RCC_CLEAR_RESET_FLAGS();

  lcd_init();
  app_wwdg_kick();
  APP_LOG("[boot] lcd_init ok\r\n");
  probe_init();
  app_wwdg_kick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    mb_poll();
    static uint32_t last_print = 0;
    bool any_alert = false;
    ads_meas_t m;

    for (int i = 0; i < ADS_NUM_CHANNELS; i++) {
      poll_serial();
      mb_poll();
      app_wwdg_kick();

      if (!ch_enabled(i)) {
        /* Disabled channel: clear old data (qty= changed at runtime), LED off, no measurement */
        vac_ch[i] = 0.0f; raw_ch[i] = 0.0f; vac_uncal[i] = 0.0f;
        wire_ch[i] = PROBE_UNKNOWN; wire_mvpp[i] = 0;
        set_machine_led(i, CH_DISABLED);
        continue;
      }

      /* Check the wire before every measurement, regardless of the previous result. */
      probe_result_t pr;
      if (probe_measure(i, &pr)) {
        wire_ch[i]   = probe_update(i, &pr);
        wire_mvpp[i] = pr.mvpp;
      } else {
        APP_LOG("[probe] P%d ADC error\r\n", i + 1);
      }
      HAL_Delay(PROBE_SETTLE_MS);
      app_wwdg_kick();

      bool ok = ads_measure_channel(&hi2c1, i, &m);
      vac_uncal[i] = ok ? m.vac : 0.0f;
      vac_ch[i] = ok ? m.vac * kf(i) : -1.0f;   /* -1 = IC not responding */
      raw_ch[i] = ok ? m.vrms_total : 0.0f;
      if (ok && vac_ch[i] < VAC_ZERO_V) vac_ch[i] = 0.0f;
      if (!ok) APP_LOG("[i2c] P%d not responding\r\n", i + 1);


      ch_status_t st = ch_status(i);
      if (st == CH_LEAK || st == CH_WIRE_NG) any_alert = true;
      set_machine_led(i, st);
      app_wwdg_kick();
    }
    mb_heartbeat++;

    /* PA0 (label D0) is the LCD A0 pin — no longer used as a global alert output */

    /* Print results + update the LCD every 1 s (a 4-channel scan takes ~820 ms) */
    uint32_t now = HAL_GetTick();
    if (now - last_print >= PRINT_INTERVAL_MS) {
      last_print = now;
      lcd_show_channels(any_alert);
      app_wwdg_kick();
      mb_poll();
      /* RTT has no %f: print mV (vac*1000), -1 = channel error */
      APP_LOG("[meas] P1=%d P2=%d P3=%d P4=%d mV | %s, %s, %s, %s (%u %u %u %u mVpp)%s\r\n",
          (int)(vac_ch[0] * 1000.0f), (int)(vac_ch[1] * 1000.0f),
          (int)(vac_ch[2] * 1000.0f), (int)(vac_ch[3] * 1000.0f),
          probe_state_str(wire_ch[0]), probe_state_str(wire_ch[1]),
          probe_state_str(wire_ch[2]), probe_state_str(wire_ch[3]),
          (unsigned)wire_mvpp[0], (unsigned)wire_mvpp[1],
          (unsigned)wire_mvpp[2], (unsigned)wire_mvpp[3],
          any_alert ? " ALERT" : "");
    }
    app_wwdg_kick();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00402D41;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 11;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 11;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  /* Standard Modbus RTU 8E1: WordLength 9B = 8 data + 1 even parity */
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_RX;   /* RX only (PA3); PA4 is reserved for LCD /RST */
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  USART2->CR1 |= USART_CR1_RXNEIE_RXFNEIE;   /* RX into the ring via interrupt — see app_usart2_isr() */
  HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);   /* lower than Modbus (USART1 = 1) */
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */

  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 80;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */

  /* USER CODE END WWDG_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN1_IC3_Pin|IN1_IC2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D0_Pin|IN1_IC4_Pin|L_GREEN4_Pin|L_RED4_Pin
                          |IN1_IC1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_GREEN3_Pin|L_RED3_Pin|L_GREEN2_Pin|L_RED2_Pin
                          |L_GREEN1_Pin|L_RED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_IC3_Pin IN1_IC2_Pin */
  GPIO_InitStruct.Pin = IN1_IC3_Pin|IN1_IC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D0_Pin IN1_IC4_Pin L_GREEN4_Pin L_RED4_Pin
                           IN1_IC1_Pin */
  GPIO_InitStruct.Pin = D0_Pin|IN1_IC4_Pin|L_GREEN4_Pin|L_RED4_Pin
                          |IN1_IC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : L_GREEN3_Pin L_RED3_Pin L_GREEN2_Pin L_RED2_Pin
                           L_GREEN1_Pin L_RED1_Pin */
  GPIO_InitStruct.Pin = L_GREEN3_Pin|L_RED3_Pin|L_GREEN2_Pin|L_RED2_Pin
                          |L_GREEN1_Pin|L_RED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Active-low LEDs (common anode): the generated code above writes RESET = all on.
   * Turn everything off right away. When regenerating with CubeMX: set the GPIO output level of
   * the L_GREENx/L_REDx pins to High in the .ioc, then remove this block. */
  HAL_GPIO_WritePin(GPIOA, L_GREEN4_Pin | L_RED4_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, L_GREEN3_Pin | L_RED3_Pin | L_GREEN2_Pin | L_RED2_Pin
                          | L_GREEN1_Pin | L_RED1_Pin, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
