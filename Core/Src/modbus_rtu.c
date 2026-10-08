/* SPDX-License-Identifier: MIT */
/**
 * modbus_rtu.c — Modbus RTU slave, protocol-only core (portable).
 * Includes no STM32/HAL headers — see modbus_rtu.h for how to port it.
 *
 * ISR ↔ parser synchronisation model: single-producer/single-consumer.
 * The ISR assembles frames in place (mb_rx_byte): frame boundaries are decided by the port
 * (gap ≥ 5 ms); a frame completes BY LENGTH at 8 bytes → latched into the pending slot
 * with its tick. A 9th byte arriving without a gap → wrong-length frame,
 * the whole slot is discarded (silently). There is only one slot: a new request overwrites an
 * unprocessed old one — matching Modbus request/response semantics (the old one is stale for the master anyway).
 * The consumer (mb_poll) copies the slot using a sequence counter, no IRQ masking needed.
 */
#include "modbus_rtu.h"

#define MB_REQ_LEN 8u   /* every valid FC03/04/06 request is exactly 8 bytes */

/* Supported function codes */
#define FC_READ_HOLDING 0x03u
#define FC_READ_INPUT   0x04u
#define FC_WRITE_SINGLE 0x06u

/* Exception codes */
#define EXC_ILLEGAL_FC    0x01u
#define EXC_ILLEGAL_ADDR  0x02u
#define EXC_ILLEGAL_VALUE 0x03u

static const mb_port_t *s_port;
static volatile uint8_t s_addr;      /* slave address — read by ISR, written by main */
static uint16_t s_reg_count;

/* --- Frame assembly, touched only by the ISR --- */
static uint8_t s_asm[MB_REQ_LEN];
static uint8_t s_asm_len;            /* 0..8; stays 8 after latching to detect extra bytes */
static bool    s_asm_drop;           /* corrupt/too-long frame: drop bytes until the next boundary */

/* --- Pending frame slot: written by ISR, read by mb_poll --- */
static volatile uint8_t  s_pend[MB_REQ_LEN];
static volatile uint32_t s_pend_tick;   /* tick when the 8th byte arrived — for the stale rule */
static volatile bool     s_pend_ready;
static volatile uint8_t  s_pend_seq;    /* incremented on every ISR slot write — lock-free copy */

/* --- TX --- */
static uint8_t s_tx_buf[MB_TX_BUF_SIZE];
static bool    s_tx_sending;

/* CRC16-Modbus, reflected poly 0xA001, init 0xFFFF, bitwise (no table — saves flash) */
static uint16_t mb_crc16(const uint8_t *p, uint16_t len)
{
  uint16_t crc = 0xFFFFu;
  while (len--) {
    crc ^= *p++;
    for (int i = 0; i < 8; i++)
      crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
  }
  return crc;   /* sent on the wire low byte first */
}

void mb_init(const mb_port_t *port, uint8_t slave_addr, uint16_t reg_count)
{
  s_port      = port;
  s_addr      = slave_addr;
  s_reg_count = reg_count;
  s_asm_len   = 0;
  s_asm_drop  = false;
  s_pend_ready = false;
  s_tx_sending = false;
}

void mb_set_addr(uint8_t addr) { s_addr = addr; }

void mb_rx_byte(uint8_t byte, bool frame_boundary)
{
  if (frame_boundary) {              /* gap → resynchronise from this byte */
    s_asm_len  = 0;
    s_asm_drop = false;
  }
  if (s_asm_drop) return;

  if (s_asm_len < MB_REQ_LEN) {
    s_asm[s_asm_len++] = byte;
    if (s_asm_len == MB_REQ_LEN) {   /* complete by length — latch into the slot */
      s_pend_seq++;
      for (int i = 0; i < (int)MB_REQ_LEN; i++) s_pend[i] = s_asm[i];
      s_pend_tick  = s_port->get_tick_ms();
      s_pend_ready = true;
      s_pend_seq++;
      /* s_asm_len stays 8: a following byte without a gap means the frame is really longer than 8 */
    }
  } else {
    /* 9th byte of the same frame: wrong length — discard the slot just latched, silently */
    s_pend_ready = false;
    s_asm_drop   = true;
  }
}

void mb_rx_error(void)
{
  if (s_asm_len == MB_REQ_LEN)       /* error right after the 8 latched bytes → frame is corrupt */
    s_pend_ready = false;
  s_asm_drop = true;
}

/* Copy the pending slot into a private buffer; false if no stable frame */
static bool take_pending(uint8_t *dst, uint32_t *tick)
{
  for (int attempt = 0; attempt < 3; attempt++) {
    if (!s_pend_ready) return false;
    uint8_t seq1 = s_pend_seq;
    if (seq1 & 1u) continue;         /* ISR is mid-write */
    for (int i = 0; i < (int)MB_REQ_LEN; i++) dst[i] = s_pend[i];
    uint32_t t = s_pend_tick;
    if (s_pend_seq != seq1) continue;/* overwritten meanwhile — retry */
    *tick = t;
    s_pend_ready = false;
    return true;
  }
  return false;
}

static void send_frame(uint16_t len, bool is_data_reply)
{
  uint16_t crc = mb_crc16(s_tx_buf, (uint16_t)(len - 2u));
  s_tx_buf[len - 2u] = (uint8_t)(crc & 0xFFu);
  s_tx_buf[len - 1u] = (uint8_t)(crc >> 8);
  if (s_port->tx_start(s_tx_buf, len)) {
    s_tx_sending = true;
    if (is_data_reply && s_port->on_reply_ok) s_port->on_reply_ok();
  }
}

static void send_exception(uint8_t fc, uint8_t code)
{
  s_tx_buf[0] = s_addr;
  s_tx_buf[1] = (uint8_t)(fc | 0x80u);
  s_tx_buf[2] = code;
  send_frame(5u, false);
}

void mb_poll(void)
{
  if (!s_port) return;

  /* TX running: wait for the port to report completion (the port clears flags + echo guard) */
  if (s_tx_sending) {
    if (s_port->tx_busy()) return;
    s_tx_sending = false;
  }

  uint8_t  req[MB_REQ_LEN];
  uint32_t tick;
  if (!take_pending(req, &tick)) return;

  /* Stale request: a late reply would collide with the master's retry — drop silently */
  if ((uint32_t)(s_port->get_tick_ms() - tick) > MB_STALE_MS) return;

  if (req[0] == 0u) return;          /* broadcast: not supported, silent */
  if (req[0] != s_addr) return;      /* not for us */

  uint16_t crc = (uint16_t)(req[6] | ((uint16_t)req[7] << 8));
  if (mb_crc16(req, 6u) != crc) return;   /* bad CRC: silent */

  uint8_t  fc    = req[1];
  uint16_t field1 = (uint16_t)(((uint16_t)req[2] << 8) | req[3]);
  uint16_t field2 = (uint16_t)(((uint16_t)req[4] << 8) | req[5]);

  switch (fc) {
  case FC_READ_HOLDING:
  case FC_READ_INPUT: {
    uint16_t start = field1, qty = field2;
    /* validate BEFORE touching the buffer — prevents overflow/RAM disclosure */
    if (qty < 1u || qty > MB_MAX_REGS ||
        start >= s_reg_count || (uint32_t)start + qty > s_reg_count) {
      send_exception(fc, EXC_ILLEGAL_ADDR);
      return;
    }
    s_tx_buf[0] = s_addr;
    s_tx_buf[1] = fc;
    s_tx_buf[2] = (uint8_t)(qty * 2u);
    for (uint16_t i = 0; i < qty; i++) {
      uint16_t v = s_port->reg_read((uint16_t)(start + i));
      s_tx_buf[3u + 2u * i] = (uint8_t)(v >> 8);
      s_tx_buf[4u + 2u * i] = (uint8_t)(v & 0xFFu);
    }
    send_frame((uint16_t)(5u + 2u * qty), true);
    return;
  }
  case FC_WRITE_SINGLE:
    switch (s_port->reg_write(field1, field2)) {
    case MB_WR_OK:
      /* echo the request verbatim */
      for (int i = 0; i < (int)MB_REQ_LEN; i++) s_tx_buf[i] = req[i];
      if (s_port->tx_start(s_tx_buf, MB_REQ_LEN)) {
        s_tx_sending = true;
        if (s_port->on_reply_ok) s_port->on_reply_ok();
      }
      return;
    case MB_WR_BAD_ADDR:  send_exception(fc, EXC_ILLEGAL_ADDR);  return;
    default:              send_exception(fc, EXC_ILLEGAL_VALUE); return;
    }
  default:
    send_exception(fc, EXC_ILLEGAL_FC);
    return;
  }
}
