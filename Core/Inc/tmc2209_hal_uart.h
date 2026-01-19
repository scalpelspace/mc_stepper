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

// GPIO output for MS1, MS2 pins.
// Note: Currently unused, left for reference.
// #define TMC2209_MS1_PORT GPIOB
// #define TMC2209_MS1_PIN GPIO_PIN_1
// #define TMC2209_MS2_PORT GPIOB
// #define TMC2209_MS2_PIN GPIO_PIN_0

/** Definitions. **************************************************************/

// Software driven configurations.
#define TMC2209_DEFAULT_ADDRESS (0u) // MS1, MS2 both ground.
#define TMC2209_TIMEOUT_MS (10u)     // HAL blocking UART timeout.

// Hardware datasheet defined configurations.
#define TMC2209_SYNC (0x05)

#define TMC2209_REG_GCONF (0x00)
#define TMC2209_REG_GSTAT (0x01)
#define TMC2209_REG_IOIN (0x06)
#define TMC2209_REG_IHOLD_IRUN (0x10)
#define TMC2209_REG_TPOWERDOWN (0x11)
#define TMC2209_REG_TSTEP (0x12)
#define TMC2209_REG_CHOPCONF (0x6C)
#define TMC2209_REG_DRV_STATUS (0x6F)
#define TMC2209_REG_PWMCONF (0x70)
#define TMC2209_REG_PWM_SCALE (0x71)

#define TMC2209_CHOPCONF_MRES_SHIFT 24u
#define TMC2209_CHOPCONF_MRES_MASK (0x0Fu << TMC2209_CHOPCONF_MRES_SHIFT)

/** Public functions. *********************************************************/

HAL_StatusTypeDef tmc2209_clear_gstat(void);

HAL_StatusTypeDef tmc2209_set_microsteps(uint16_t microsteps);

HAL_StatusTypeDef tmc2209_set_ihold_irun(uint8_t ihold, uint8_t irun,
                                         uint8_t iholddelay);

#endif
