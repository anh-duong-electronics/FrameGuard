/* SPDX-License-Identifier: MIT */
/**
 * mb_port_usart1.h — Modbus RTU port layer for STM32C031, USART1 + DMA1.
 * Reference example when porting modbus_rtu.c to another project/chip.
 */
#ifndef MB_PORT_USART1_H
#define MB_PORT_USART1_H

#include <stdbool.h>
#include <stdint.h>

/* Call once after HAL_RS485Ex_Init(huart1): enables DMA clock, configures DMAMUX
 * request USART1_TX, CR3.DMAT, enables RX interrupts (RXNE + errors) and NVIC. */
void mb_port_usart1_init(void);

/* Plug into mb_port_t.tx_start / .tx_busy */
bool mb_port_usart1_tx_start(const uint8_t *buf, uint16_t len);
bool mb_port_usart1_tx_busy(void);

/* Call from USART1_IRQHandler (stm32c0xx_it.c) */
void mb_port_usart1_isr(void);

#endif /* MB_PORT_USART1_H */
