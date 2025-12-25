/*******************************************************************************
 * @file abi_encoder_hal_tim.h
 * @brief ABI (quadrature) encoder functions: abstracting STM32 HAL: TIM.
 *******************************************************************************
*/

#ifndef MC_STEPPER__ABI_ENCODER_HAL_TIM_H
#define MC_STEPPER__ABI_ENCODER_HAL_TIM_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"
#include <stdint.h>

/** STM32 port and pin configs. ***********************************************/

extern TIM_HandleTypeDef htim2;

// TIM.
#define ABI_ENCODER_HTIM htim2

/** Public types. *************************************************************/

/**
 * @brief Example definition for a single ABI quadrature encoder instance.
 */
typedef struct {
  TIM_HandleTypeDef *htim; // Timer instance.
  int32_t previous_count;  // Previous count.
  int32_t delta;           // Current delta at sample rate.
} abi_encoder_t;

/** Public functions. *********************************************************/

/**
 * @brief Initialize an ABI quadrature encoder using a TIM peripheral.
 *
 * Starts the timer in encoder mode and resets internal state and counter.
 *
 * @param encoder Pointer to encoder instance.
 * @param htim Pointer to initialized TIM encoder handle.
 */
void abi_encoder_init(abi_encoder_t *encoder, TIM_HandleTypeDef *htim);

/**
 * @brief Update encoder position delta since last call.
 *
 * Reads the timer counter and computes the incremental count change.
 *
 * @param encoder Pointer to encoder instance.
 */
void abi_encoder_update(abi_encoder_t *encoder);

/**
 * @brief Reset encoder position and internal state.
 *
 * Zeros the hardware counter and cached position values.
 *
 * @param encoder Pointer to encoder instance.
 */
void abi_encoder_reset(abi_encoder_t *encoder);

#endif
