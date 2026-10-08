/* SPDX-License-Identifier: MIT */
/**
 * ads1015.h — differential AC voltage measurement via ADS1015, 50 Hz amplitude via Goertzel.
 * 2 ICs (ADDR=GND 0x48, ADDR=VDD 0x49), 2 differential channels each (AIN0-1, AIN2-3) → 4 channels.
 */
#ifndef ADS1015_H
#define ADS1015_H

#include "stm32c0xx_hal.h"
#include <stdbool.h>

#define ADS_NUM_CHANNELS   4


/* Result of one measurement on one channel */
typedef struct {
  float v50_rms;     /* RMS of the 50 Hz component (V, ADC side) */
  float vac;         /* V_AC converted from vrms_total through the calibration table (V, mains side) */
  float vrms_total;  /* full-band RMS */
  float vpeak;       /* absolute peak */
  float vdc;         /* DC component */
  int   n;           /* number of samples captured in the window */
} ads_meas_t;


/* Provided by the app: refresh the WWDG if inside the allowed window.
 * Called inside the sampling loop (~200 ms/channel > WWDG timeout). */
extern void app_wwdg_kick(void);

/* Provided by the app: service communication (Modbus) — called every sampling iteration to keep
 * reply latency low during the 200 ms measurement window. Completing a frame costs a few
 * hundred µs (stretches one sample interval ~once per second while polled — acceptable). */
extern void app_comm_poll(void);

/* Default configuration for ICs present on the bus (call once at startup) */
bool ads_probe(I2C_HandleTypeDef *hi2c, uint8_t addr7);

/* Measure channel idx (0..3): set MUX, sample for 200 ms, Goertzel + RMS.
 * Returns false if the IC does not respond. */
bool ads_measure_channel(I2C_HandleTypeDef *hi2c, int idx, ads_meas_t *m);

#endif /* ADS1015_H */
