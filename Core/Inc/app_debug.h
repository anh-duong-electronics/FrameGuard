/* SPDX-License-Identifier: MIT */
#ifndef APP_DEBUG_H
#define APP_DEBUG_H

/*
 * Debug logging over SEGGER RTT (J-Link), switched by APP_DEBUG:
 *   - Debug build:       make DEBUG=1    → APP_DEBUG=1, RTT logging enabled
 *   - Production build:  make (DEBUG=0)  → APP_DEBUG=0, DBG() disappears
 *     completely from the binary (empty macro + gc-sections drops RTT code).
 *
 * View logs: JLinkRTTClient (while JLinkExe/GDB server is connected),
 * or JLinkRTTViewer (device STM32C031K6, SWD, 3200 kHz).
 *
 * NOTE: SEGGER_RTT_printf does NOT support %f — log floats by multiplying
 * by 1000 and printing %d (mV instead of V).
 */

#if APP_DEBUG
  #include "SEGGER_RTT.h"
  #define APP_LOG(...)    SEGGER_RTT_printf(0, __VA_ARGS__)
  /* Print to virtual terminal t (RTT Viewer: Terminal 0..15 tabs), then switch back to terminal 0 */
  #define APP_LOG_T(t, ...) do { SEGGER_RTT_SetTerminal(t); SEGGER_RTT_printf(0, __VA_ARGS__); \
                                SEGGER_RTT_SetTerminal(0); } while (0)
  #define APP_LOG_INIT()  SEGGER_RTT_Init()
#else
  #define APP_LOG(...)    ((void)0)
  #define APP_LOG_T(...)  ((void)0)
  #define APP_LOG_INIT()  ((void)0)
#endif

#endif /* APP_DEBUG_H */
