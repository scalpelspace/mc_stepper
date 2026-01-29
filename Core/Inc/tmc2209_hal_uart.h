/*******************************************************************************
 * @file tmc2209_hal_uart.h
 * @brief TMC2209 functions: abstracting STM32 HAL: UART.
 *******************************************************************************
 */

#ifndef MC_STEPPER__TMC2209_HAL_UART_H
#define MC_STEPPER__TMC2209_HAL_UART_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_uart.h"
#include <stdint.h>

/** STM32 port and pin configs. ***********************************************/

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim15;

// UART.
#define TMC2209_HUART huart2

// Timer for step signal.
#define TMC2209_HTIM htim15
#define TMC2209_HTIM_CHANNEL TIM_CHANNEL_2

// GPIO output for enable pin.
#define TMC2209_ENN_PORT GPIOA
#define TMC2209_ENN_PIN GPIO_PIN_7

// GPIO output for direction pin.
#define TMC2209_DIR_PORT GPIOA
#define TMC2209_DIR_PIN GPIO_PIN_6

/** Public functions. *********************************************************/

HAL_StatusTypeDef tmc2209_clear_gstat(void);

HAL_StatusTypeDef tmc2209_set_microsteps(uint16_t microsteps);

HAL_StatusTypeDef tmc2209_set_ihold_irun(uint8_t ihold, uint8_t irun,
                                         uint8_t iholddelay);

#endif
