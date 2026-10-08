/* SPDX-License-Identifier: MIT */
/**
 * config_store.c — stores configuration in the last flash page of the STM32C031 (page 15).
 * Written as double-words (8 bytes); the struct is padded to a multiple of 8.
 *
 * The magic sits in the LAST double-word and is programmed last: a power loss mid-save
 * → no magic → next boot falls back to defaults, never reads a half-written config
 * (a garbage Modbus address would cause address clashes on the bus).
 */
#include "config_store.h"
#include "stm32c0xx_hal.h"
#include <stddef.h>
#include <string.h>

#define CFG_PAGE        15U
#define CFG_ADDR        0x08007800UL
/* Layout v4 (adds wire_mvpp, ch_count) — differs from v1 0xAC05C0DE / v2 0xAC05C1D2 /
 * v3 0xAC05C1D3: older devices booting this firmware fall back to defaults and must be reconfigured. */
#define CFG_MAGIC       0xAC05C1D4UL

/* Flash layout: 32 bytes = 4 double-words; the magic occupies the last word */
typedef struct {
  uint16_t device_id;
  uint8_t  modbus_addr;
  uint8_t  ch_count;
  uint16_t alert_mv;
  uint16_t wire_mvpp;
  float    k_factor[4];
  uint32_t pad2;
  uint32_t magic;          /* last double-word — programmed last */
} flash_cfg_t;

_Static_assert(sizeof(flash_cfg_t) % 8 == 0, "flash_cfg_t must be multiple of 8 bytes");
_Static_assert(offsetof(flash_cfg_t, magic) == sizeof(flash_cfg_t) - 4,
               "magic must live in the last double-word");

bool config_load(app_config_t *cfg)
{
  const flash_cfg_t *f = (const flash_cfg_t *)CFG_ADDR;
  if (f->magic != CFG_MAGIC) return false;

  if (f->device_id >= 1 && f->device_id <= 9999) cfg->device_id = f->device_id;
  if (f->modbus_addr >= 1 && f->modbus_addr <= 247) cfg->modbus_addr = f->modbus_addr;
  if (f->ch_count >= 1 && f->ch_count <= 4) cfg->ch_count = f->ch_count;
  if (f->alert_mv >= 100 && f->alert_mv <= 60000) cfg->alert_mv = f->alert_mv;
  if (f->wire_mvpp >= 100 && f->wire_mvpp <= 3000) cfg->wire_mvpp = f->wire_mvpp;
  for (int i = 0; i < 4; i++) {
    /* Valid K is 0.5..2.0 (matches the v=/kN= command guard); garbage/0 → keep default 1.0 */
    if (f->k_factor[i] >= 0.5f && f->k_factor[i] <= 2.0f)
      cfg->k_factor[i] = f->k_factor[i];
  }
  return true;
}

bool config_save(const app_config_t *cfg)
{
  flash_cfg_t f;
  memset(&f, 0, sizeof(f));
  f.device_id   = cfg->device_id;
  f.modbus_addr = cfg->modbus_addr;
  f.ch_count    = cfg->ch_count;
  f.alert_mv    = cfg->alert_mv;
  f.wire_mvpp   = cfg->wire_mvpp;
  memcpy(f.k_factor, cfg->k_factor, sizeof(f.k_factor));
  f.magic       = CFG_MAGIC;

  if (HAL_FLASH_Unlock() != HAL_OK) return false;

  FLASH_EraseInitTypeDef erase = {
    .TypeErase = FLASH_TYPEERASE_PAGES,
    .Page      = CFG_PAGE,
    .NbPages   = 1,
  };
  uint32_t page_err = 0;
  bool ok = (HAL_FLASHEx_Erase(&erase, &page_err) == HAL_OK);

  if (ok) {
    /* Program sequentially from the start: the double-word holding the magic is written last */
    const uint64_t *src = (const uint64_t *)&f;
    for (uint32_t off = 0; off < sizeof(f); off += 8) {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                            CFG_ADDR + off, src[off / 8]) != HAL_OK) {
        ok = false;
        break;
      }
    }
  }

  HAL_FLASH_Lock();
  return ok;
}
