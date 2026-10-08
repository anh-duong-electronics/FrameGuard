/* SPDX-License-Identifier: MIT */
/**
 * mb_port_usart1.c — Modbus RTU port layer: STM32C031, USART1 (hardware RS-485 DE)
 * + DMA1 channel 1 for TX.
 *
 * Both RX ISR and TX DMA are bare-register and do NOT go through HAL_UART_IRQHandler /
 * HAL_UART_Transmit_DMA: the HAL completion path needs the TC interrupt handled by HAL itself,
 * which conflicts with a bare ISR (TC interrupt storm → WWDG reset / gState stuck BUSY).
 * After mb_port_usart1_init(), do NOT call any HAL UART TX API on huart1
 * — one UART, one owner.
 */
#include "mb_port_usart1.h"
#include "modbus_rtu.h"
#include "stm32c0xx_hal.h"

#define GAP_BOUNDARY_MS 5u    /* gap ≥ this → first byte of a new frame (T3.5 ≈ 4 ms @ 9600 8E1) */
#define TX_TAIL_GUARD_MS 2u   /* keep RX blocked ~1 byte-time after TC — drains the last echo byte */

static volatile bool     s_tx_active;
static volatile bool     s_guard_on;
static volatile uint32_t s_guard_until;
static uint32_t          s_last_rx_tick;

void mb_port_usart1_init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();        /* DMAMUX1 shares the DMA1 clock on C0 */

  /* DMA1 ch1 ← DMAMUX1 ch0: request 51 = USART1_TX; mem→periph, MINC, 8 bit */
  DMAMUX1_Channel0->CCR = DMA_REQUEST_USART1_TX;
  DMA1_Channel1->CCR    = DMA_CCR_MINC | DMA_CCR_DIR;
  DMA1_Channel1->CPAR   = (uint32_t)&USART1->TDR;

  USART1->CR3 |= USART_CR3_DMAT;

  /* RX interrupts: incoming byte + all receive errors (PE separately, FE/NE/ORE via EIE) */
  USART1->ICR  = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
  USART1->CR1 |= USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE;
  USART1->CR3 |= USART_CR3_EIE;

  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void mb_port_usart1_isr(void)
{
  uint32_t isr = USART1->ISR;

  if (isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) {
    USART1->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
    if (isr & USART_ISR_RXNE_RXFNE)
      (void)USART1->RDR;              /* read and discard the bad byte (including a byte stuck after ORE) */
    mb_rx_error();
    return;
  }

  if (isr & USART_ISR_RXNE_RXFNE) {
    uint8_t  byte = (uint8_t)(USART1->RDR & 0xFFu);
    uint32_t now  = HAL_GetTick();
    bool boundary = (uint32_t)(now - s_last_rx_tick) >= GAP_BOUNDARY_MS;
    s_last_rx_tick = now;
    if (s_tx_active) return;          /* transmitting: this byte is our own echo — drop it */
    mb_rx_byte(byte, boundary);
  }
}

bool mb_port_usart1_tx_start(const uint8_t *buf, uint16_t len)
{
  if (s_tx_active) return false;

  s_tx_active = true;                 /* block RX echo before the first byte hits the wire */
  s_guard_on  = false;

  USART1->ICR = USART_ICR_TCCF;       /* TC sets again when the last byte leaves the shifter */
  DMA1_Channel1->CCR &= ~DMA_CCR_EN;
  DMA1->IFCR = DMA_IFCR_CGIF1;
  DMA1_Channel1->CMAR  = (uint32_t)buf;
  DMA1_Channel1->CNDTR = len;
  DMA1_Channel1->CCR |= DMA_CCR_EN;
  return true;
}

bool mb_port_usart1_tx_busy(void)
{
  if (!s_tx_active) return false;

  if (!s_guard_on) {
    if (!(DMA1->ISR & DMA_ISR_TCIF1)) return true;   /* DMA has not pushed everything into TDR yet */
    if (!(USART1->ISR & USART_ISR_TC)) return true;  /* last byte has not left the wire (DE still held) */
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    s_guard_until = HAL_GetTick() + TX_TAIL_GUARD_MS;
    s_guard_on    = true;
    return true;
  }
  if ((int32_t)(HAL_GetTick() - s_guard_until) < 0) return true;

  s_guard_on  = false;
  s_tx_active = false;
  return false;
}
