/* SPDX-License-Identifier: MIT */
/**
 * config_store.h — stores K factors + device id in the last flash page
 * (STM32C031 has no EEPROM; 32 KB flash, 2 KB pages → page 15 @ 0x08007800).
 */
#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint16_t device_id;       /* 1..9999 — device identifier */
  uint8_t  modbus_addr;     /* 1..247  — RS-485 slave address */
  uint8_t  ch_count;        /* 1..4    — number of channels in use (P1..Pn); the rest are disabled */
  uint16_t alert_mv;        /* 100..60000 — leakage alarm threshold, mV */
  uint16_t wire_mvpp;       /* 100..3000  — wire-break threshold (shared by all 4 channels) */
  float   k_factor[4];
} app_config_t;

/* Load config from flash; returns false if absent (bad magic) — defaults are kept */
bool config_load(app_config_t *cfg);

/* Write config to flash (erase page + program). Returns false on flash error. */
bool config_save(const app_config_t *cfg);

#endif /* CONFIG_STORE_H */
