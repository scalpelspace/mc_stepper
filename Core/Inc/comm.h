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

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "stm32l4xx_hal.h"

/** Definitions. **************************************************************/

#define COMM_LINE_MAX 96
#define COMM_RTOS_QUEUE_DEPTH 4

/** Public types. *************************************************************/

typedef struct {
  char line[COMM_LINE_MAX];
} comm_line_message_t;

/** STM32 port and pin configs. ***********************************************/

extern UART_HandleTypeDef huart1;

// UART.
#define COMM_HUART huart1

/** User implementations into STM32 HAL (overwrite weak HAL functions). *******/

void HAL_UART_RxCpltCallback_comm(UART_HandleTypeDef *huart);

/** Public functions. *********************************************************/

void printf_set_mutexes(osMutexId_t printf_mutex);
void comm_uart_set_queue(osMessageQueueId_t comm_queue);

void comm_init(void);
void comm_handle_line(const char *line);

#endif
