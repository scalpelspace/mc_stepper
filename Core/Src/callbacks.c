/*******************************************************************************
 * @file callbacks.c
 * @brief STM32 HAL callback implementations overriding weak declarations.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "comm.h"
#include "stm32l4xx_hal.h"
#include "tmc2209_runner.h"

/** Collection of user implementations into STM32 HAL (overwriting HAL). ******/

/** TIM. */

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
  HAL_TIM_OC_DelayElapsedCallback_tmc2209(htim);
}

/** UART. */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  HAL_UART_RxCpltCallback_comm(huart);
}
