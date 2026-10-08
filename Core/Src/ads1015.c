/* SPDX-License-Identifier: MIT */
/**
 * ads1015.c — differential AC voltage measurement via ADS1015:
 * continuous mode, PGA ±2.048V, DR 3300SPS, 200 ms window, 50 Hz Goertzel
 * with the coefficient computed at runtime from the actual sample rate (adapts to I2C speed).
 */
#include "ads1015.h"
#include <math.h>
#include <stdlib.h>

#define REG_CONVERT   0x00
#define REG_CONFIG    0x01

/* Base: OS=1, PGA=010 (±2.048V — with VDD at 3.3 V, ±4.096V would waste range),
 * MODE=0 (continuous), DR=111 (3300SPS), COMP_QUE=11 (disable). MUX changes per channel. */
#define CFG_DIFF_01   0x84E3   /* MUX=000: AIN0-AIN1 */
#define CFG_DIFF_23   0xB4E3   /* MUX=011: AIN2-AIN3 */

typedef struct { uint8_t addr; uint16_t cfg; } channel_cfg_t;

static const channel_cfg_t CHANNELS[ADS_NUM_CHANNELS] = {
  { 0x48, CFG_DIFF_01 },   /* M1 — IC0 (ADDR=GND), diff 0-1 */
  { 0x48, CFG_DIFF_23 },   /* M2 — IC0 (ADDR=GND), diff 2-3 */
  { 0x49, CFG_DIFF_01 },   /* M3 — IC1 (ADDR=VDD), diff 0-1 */
  { 0x49, CFG_DIFF_23 },   /* M4 — IC1 (ADDR=VDD), diff 2-3 */
};

#define PGA_FS_V        2.048f
#define ADC_FS_COUNTS   2047
#define V_PER_COUNT     (PGA_FS_V / (float)ADC_FS_COUNTS)

#define WINDOW_US       200000UL   /* 10 cycles of 50 Hz */
#define MAX_SAMPLES     512
#define SAMPLE_DELAY_US 310UL
#define MUX_SETTLE_MS   3
#define I2C_TMO_MS      10

#define SQRT2           1.41421356f
#define TARGET_HZ       50.0f

/* ==== Calibration table: raw vrms_total (V, ADC side) → actual V_AC ====
 * ILLUSTRATIVE LINEAR PLACEHOLDER. Measurement transformers are usually non-linear at
 * low voltage: apply known reference voltages one by one, read vrms_total (RTT log, DEBUG=1),
 * then replace these entries with your hardware's real measurements. raw must increase.
 * Piecewise-linear interpolation; below the first point interpolate to (0,0); above the last
 * point extrapolate along the last segment. Per-channel deviation is trimmed by factor K (v=/kN=). */
typedef struct { float raw; float vac; } cal_point_t;
static const cal_point_t CAL[] = {
  { 0.0100f,  0.5f },
  { 0.1000f,  5.0f },
  { 0.4000f, 20.0f },
};
#define CAL_N  (int)(sizeof(CAL) / sizeof(CAL[0]))

static float raw_to_vac(float raw)
{
  if (raw <= 0.0f) return 0.0f;
  /* below the first point: segment (0,0) → CAL[0] */
  if (raw < CAL[0].raw)
    return raw * (CAL[0].vac / CAL[0].raw);
  for (int i = 1; i < CAL_N; i++) {
    if (raw < CAL[i].raw) {
      float t = (raw - CAL[i - 1].raw) / (CAL[i].raw - CAL[i - 1].raw);
      return CAL[i - 1].vac + t * (CAL[i].vac - CAL[i - 1].vac);
    }
  }
  /* above the last point: extrapolate along the last segment */
  float slope = (CAL[CAL_N - 1].vac - CAL[CAL_N - 2].vac)
              / (CAL[CAL_N - 1].raw - CAL[CAL_N - 2].raw);
  return CAL[CAL_N - 1].vac + (raw - CAL[CAL_N - 1].raw) * slope;
}

static int16_t sample_buf[MAX_SAMPLES];

/* cos for small arguments (2π·50/sps < 0.7 rad) via 8th-order Taylor — error < 1e-7,
 * avoids pulling in libm cosf (~4.5 KB of argument-reduction code for large angles). */
static float cos_small(float x)
{
  float x2 = x * x;
  return 1.0f - x2 / 2.0f + x2 * x2 / 24.0f
         - x2 * x2 * x2 / 720.0f + x2 * x2 * x2 * x2 / 40320.0f;
}

/* ---- microseconds from SysTick (HAL reloads it every 1 ms) ---- */
static uint32_t micros(void)
{
  uint32_t ms, val;
  do {
    ms  = HAL_GetTick();
    val = SysTick->VAL;
  } while (ms != HAL_GetTick());          /* avoid reading across a tick */
  uint32_t us_frac = (SysTick->LOAD - val) / (SystemCoreClock / 1000000U);
  return ms * 1000U + us_frac;
}

static void delay_us(uint32_t us)
{
  uint32_t t0 = micros();
  while ((micros() - t0) < us) { }
}

/* ---- I2C helpers ---- */
static bool ads_write16(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint16_t value)
{
  uint8_t d[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
  return HAL_I2C_Master_Transmit(hi2c, addr << 1, d, 3, I2C_TMO_MS) == HAL_OK;
}

static bool ads_set_pointer(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg)
{
  return HAL_I2C_Master_Transmit(hi2c, addr << 1, &reg, 1, I2C_TMO_MS) == HAL_OK;
}

/* Read CONVERT with the pointer already at 0x00; ADS1015 returns left-aligned 12-bit */
static bool ads_read_convert(I2C_HandleTypeDef *hi2c, uint8_t addr, int16_t *out)
{
  uint8_t d[2];
  if (HAL_I2C_Master_Receive(hi2c, addr << 1, d, 2, I2C_TMO_MS) != HAL_OK)
    return false;
  int16_t raw = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
  *out = raw >> 4;
  return true;
}

bool ads_probe(I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
  if (HAL_I2C_IsDeviceReady(hi2c, addr7 << 1, 2, I2C_TMO_MS) != HAL_OK)
    return false;
  return ads_write16(hi2c, addr7, REG_CONFIG, CFG_DIFF_01);
}

bool ads_measure_channel(I2C_HandleTypeDef *hi2c, int idx, ads_meas_t *m)
{
  uint8_t  addr = CHANNELS[idx].addr;
  uint16_t cfg  = CHANNELS[idx].cfg;

  /* 1. Set MUX/PGA (continuous), point to CONVERT, wait to settle */
  if (!ads_write16(hi2c, addr, REG_CONFIG, cfg)) return false;
  if (!ads_set_pointer(hi2c, addr, REG_CONVERT)) return false;
  HAL_Delay(MUX_SETTLE_MS);

  /* 2. Sample over the 200 ms window — longer than the WWDG timeout, so kick inside the loop */
  int n = 0;
  uint32_t t0 = micros();
  while (n < MAX_SAMPLES && (micros() - t0) < WINDOW_US) {
    int16_t s;
    if (!ads_read_convert(hi2c, addr, &s)) return false;
    sample_buf[n++] = s;
    app_wwdg_kick();
    app_comm_poll();
    delay_us(SAMPLE_DELAY_US);
  }
  if (n < 2) return false;

  /* 3. DC offset */
  long sum = 0;
  for (int i = 0; i < n; i++) sum += sample_buf[i];
  float dc_counts = (float)sum / n;

  /* 4. Goertzel coefficient from the actual sample rate of this window */
  float sps_actual = (float)n * 1000000.0f / (float)WINDOW_US;
  float coeff = 2.0f * cos_small(2.0f * (float)M_PI * TARGET_HZ / sps_actual);

  /* 5. Goertzel + total RMS + peak in a single pass */
  float q1 = 0, q2 = 0, sum_sq = 0;
  int16_t peak_counts = 0;
  for (int i = 0; i < n; i++) {
    float ac = (float)sample_buf[i] - dc_counts;
    float q0 = coeff * q1 - q2 + ac;
    q2 = q1;
    q1 = q0;
    sum_sq += ac * ac;
    int16_t a = (int16_t)abs(sample_buf[i]);
    if (a > peak_counts) peak_counts = a;
  }

  /* 6. |X|² = q1² + q2² − q1·q2·coeff; RMS_50Hz = √2·|X|/N */
  float mag_sq = q1 * q1 + q2 * q2 - q1 * q2 * coeff;
  if (mag_sq < 0) mag_sq = 0;
  float mag50 = sqrtf(mag_sq);

  m->v50_rms    = (SQRT2 * mag50 / n) * V_PER_COUNT;
  m->vrms_total = sqrtf(sum_sq / n) * V_PER_COUNT;
  m->vpeak      = peak_counts * V_PER_COUNT;
  m->vdc        = dc_counts * V_PER_COUNT;
  /* Convert from full-band RMS (50 Hz + harmonics + noise) to match a true-RMS meter
   * when the signal is distorted. */
  m->vac        = raw_to_vac(m->vrms_total);
  (void)idx;
  m->n          = n;
  return true;
}
