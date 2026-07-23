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
#include <stdbool.h>
#include <stdint.h>

/** STM32 port and pin configs. ***********************************************/

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim15;
extern DAC_HandleTypeDef hdac1;

// UART.
#define TMC2209_HUART huart2

// Timer for step signal.
#define TMC2209_TIM_INSTANCE TIM15
#define TMC2209_HTIM htim15
#define TMC2209_HTIM_CHANNEL TIM_CHANNEL_2

// DAC output for VREF current limit control.
#define TMC2209_VREF_DAC hdac1
#define TMC2209_VREF_DAC_CHANNEL DAC_CHANNEL_1

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

HAL_StatusTypeDef tmc2209_set_tpowerdown(uint8_t tpowerdown);

HAL_StatusTypeDef tmc2209_set_current_sense(bool use_internal_rsense,
                                            bool vsense_low_018v);

/**
 * @brief Set the StallGuard threshold (SGTHRS register).
 *
 * A stall is indicated when SG_RESULT falls to or below 2*sgthrs. Higher values
 * make stall detection more sensitive (trips at lighter loads).
 *
 * @param sgthrs StallGuard threshold [0,255].
 *
 * @return HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_set_sgthrs(uint8_t sgthrs);

/**
 * @brief Read the StallGuard load result (SG_RESULT register).
 *
 * Lower values indicate higher mechanical load; meaningful only while the motor
 * is running above the TCOOLTHRS velocity.
 *
 * @param out Output 10-bit load result [0,1023]. Non-null.
 *
 * @return HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_get_sg_result(uint16_t *out);

/**
 * @brief Set the lower velocity threshold for StallGuard/CoolStep (TCOOLTHRS).
 *
 * StallGuard is only valid while TSTEP <= TCOOLTHRS (i.e. above the
 * corresponding velocity). A large value keeps it active across the operating
 * range; 0 disables it.
 *
 * @param tcoolthrs 20-bit TSTEP threshold.
 *
 * @return HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_set_tcoolthrs(uint32_t tcoolthrs);

/**
 * @brief Read the global status register (GSTAT).
 *
 * Bit 0 (reset): IC has been reset since the last clear (tmc2209_clear_gstat()).
 * Bit 1 (drv_err): latched driver error (overtemperature shutdown or short
 * circuit); stays set until cleared.
 * Bit 2 (uv_cp): charge pump undervoltage.
 *
 * @param out Output raw GSTAT register value (only bits [2:0] are defined).
 * Non-null.
 *
 * @return HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_get_gstat(uint32_t *out);

/**
 * @brief Read the driver status register (DRV_STATUS).
 *
 * Bits [7:0]: OTPW, OT, S2GA, S2GB, S2VSA, S2VSB, OLA, OLB (live status, not
 * latched); the upper bits (CS_ACTUAL, STEALTH, STST) are not decoded here.
 *
 * @param out Output raw DRV_STATUS register value. Non-null.
 *
 * @return HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_get_drv_status(uint32_t *out);

#endif
