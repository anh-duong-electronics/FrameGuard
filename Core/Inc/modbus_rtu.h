/* SPDX-License-Identifier: MIT */
/**
 * modbus_rtu.h — Modbus RTU slave, protocol-only core (portable).
 *
 * Supported scope (deliberately minimal): slave-only, FC03/FC04 (read holding/input
 * registers, same data), FC06 (write single register), standard 8-byte RTU requests.
 * No FC16, no coils, no broadcast (address 0 is ignored entirely).
 *
 * Reuse in other projects: this file and modbus_rtu.c include NO STM32/HAL
 * headers — every hardware dependency goes through the mb_port_t callback struct.
 * A new project only needs a port layer (see mb_port_usart1.c as an example):
 *   - the UART ISR calls mb_rx_byte() for each received byte (with a frame-boundary flag),
 *     and mb_rx_error() on parity/framing/noise errors;
 *   - provide tx_start/tx_busy (background TX: DMA or IRQ, must NOT block);
 *   - provide get_tick_ms and the two application register read/write callbacks.
 *
 * Protocol rules:
 *   - Frames complete BY LENGTH (8 bytes from the boundary) — immune to
 *     1–16 ms inter-byte gaps from USB-RS485 adapters; gaps are only used to resync.
 *   - A frame longer than 8 bytes with no gap in between → discarded, silent.
 *   - A request received more than MB_STALE_MS ago and not yet answered → dropped silently.
 *   - Strict silence on wrong address/CRC/length; exceptions 0x01/0x02/0x03
 *     for unknown FC / addr+qty out of range / rejected write value.
 */
#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdbool.h>
#include <stdint.h>

/* Result of the register write callback (FC06) */
typedef enum {
  MB_WR_OK = 0,        /* accepted — echo the request */
  MB_WR_BAD_ADDR,      /* register not writable → exception 0x02 */
  MB_WR_BAD_VALUE,     /* value out of range    → exception 0x03 */
} mb_wr_result_t;

/* All external dependencies of the core. Every pointer is required except on_reply_ok. */
typedef struct {
  uint32_t (*get_tick_ms)(void);                       /* millisecond time source */
  bool     (*tx_start)(const uint8_t *buf, uint16_t len); /* background TX; false if busy.
                            buf points into the core's static buffer, unchanged until tx_busy()==false */
  bool     (*tx_busy)(void);                           /* TX still running? port polls completion flags here */
  uint16_t (*reg_read)(uint16_t addr);                 /* read application register, addr < reg_count */
  mb_wr_result_t (*reg_write)(uint16_t addr, uint16_t val); /* ghi register (FC06) */
  void     (*on_reply_ok)(void);                       /* optional (may be NULL): a data response was
                                                          sent successfully — used for LINK OK */
} mb_port_t;

#define MB_STALE_MS      150u   /* requests older than this → dropped silently */
#define MB_MAX_REGS      125u   /* protocol hard limit for one read request */
#define MB_TX_BUF_SIZE   (5u + 2u * 20u)   /* enough for reg_count ≤ 20; increase for larger maps */

/* port: callback struct (a reference is kept, so it must live for the whole lifetime);
 * slave_addr: 1..247; reg_count: number of readable registers (validates addr+qty ≤ reg_count,
 * requires 5 + 2*reg_count ≤ MB_TX_BUF_SIZE). */
void mb_init(const mb_port_t *port, uint8_t slave_addr, uint16_t reg_count);

/* Change the slave address at runtime (config command) — effective immediately, no reset */
void mb_set_addr(uint8_t addr);

/* Call from the UART ISR with EACH valid received byte.
 * frame_boundary = true if this byte follows a silent gap (gap ≥ T3.5,
 * practically ≥ 5 ms) — i.e. it is the first byte of a new frame. */
void mb_rx_byte(uint8_t byte, bool frame_boundary);

/* Call from the UART ISR when a received byte has a parity/framing/noise/overrun error:
 * discards the frame being assembled and waits for the next boundary to resync. */
void mb_rx_error(void);

/* Call periodically from the main loop / measurement-loop hooks (NOT from an ISR).
 * Each call: handles TX completion, takes a pending frame, parses it and sends the reply. */
void mb_poll(void);

#endif /* MODBUS_RTU_H */
