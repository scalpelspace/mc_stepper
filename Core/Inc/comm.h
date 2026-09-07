/*******************************************************************************
 * @file comm.h
 * @brief USART1 Communication Interface: abstracting STM32 HAL: UART.
 *******************************************************************************
 * @note:
 * Transmit functions should be handled via `stdio.h` `printf()` function. The
 * `_write()` function is implemented in `comm.c`.
 *******************************************************************************
 */

#ifndef MC_STEPPER__COMM_H
#define MC_STEPPER__COMM_H

/** Includes. *****************************************************************/

#include "controls.h"
#include "stm32l4xx_hal.h"

/** STM32 port and pin configs. ***********************************************/

extern UART_HandleTypeDef huart1;

// UART.
#define COMM_UART_INSTANCE USART1
#define COMM_HUART huart1

/** User implementations into STM32 HAL (overwrite weak HAL functions). *******/

void HAL_UART_RxCpltCallback_comm(UART_HandleTypeDef *huart);

/** Public functions. *********************************************************/

/**
 * @brief Initialize the communication interface and arm the RX DMA.
 */
void comm_init(void);

/**
 * @brief Execute a completed command line, if one is pending.
 *
 * The RX interrupt only frames bytes into a line. Execution is deferred here
 * so that the blocking `printf()` replies and the blocking TMC2209 UART
 * transfers never run in interrupt context, where they would overrun the
 * CONTROLS_PERIOD_MS controls tick. Poll from a scheduler task. No-op when no
 * line is pending.
 */
void comm_run(void);

#endif
