/*******************************************************************************
 * @file tmc2209_runner.c
 * @brief TMC2209 runner: init, start reports and event handling.
 *******************************************************************************
 */
/** Includes. *****************************************************************/

#include "tmc2209_runner.h"
#include "tmc2209_hal_uart.h"

/** Private variables. ********************************************************/

volatile uint16_t g_dt_ticks; // Time space between successive step signals.

/** User implementations into STM32 HAL (overwrite weak HAL functions). *******/

void HAL_TIM_OC_DelayElapsedCallback_tmc2209(TIM_HandleTypeDef *htim) {
  if (htim == &TMC2209_HTIM) {
    if (g_dt_ticks == 0)
      return; // Don't reschedule.

    const uint16_t ccr = __HAL_TIM_GET_COMPARE(htim, TMC2209_HTIM_CHANNEL);
    __HAL_TIM_SET_COMPARE(htim, TMC2209_HTIM_CHANNEL,
                          (uint16_t)(ccr + g_dt_ticks));
  }
}

/** Public functions. *********************************************************/

HAL_StatusTypeDef tmc2209_init(void) {
  HAL_StatusTypeDef result = HAL_OK;

  // Disable.
  HAL_GPIO_WritePin(TMC2209_ENN_PORT, TMC2209_ENN_PIN, GPIO_PIN_SET);
  HAL_Delay(2); // Short delay.

  // Clear GSTAT.
  result = tmc2209_clear_gstat();
  if (result != HAL_OK)
    return result;

  // Enable and set DAC output for hardware voltage divider reference (VREF).
  // TODO(Daniel): Change this in production for current sense reference.
  HAL_DAC_Start(&TMC2209_VREF_DAC, TMC2209_VREF_DAC_CHANNEL);
  HAL_DAC_SetValue(&TMC2209_VREF_DAC, TMC2209_VREF_DAC_CHANNEL, DAC_ALIGN_12B_R,
                   2048); // (12-bit right-aligned).

  // Current sense configuration.
  result = tmc2209_set_current_sense(false, false);
  if (result != HAL_OK)
    return result;

  // Set IHOLD, IRUN, IHOLDDELAY.
  result = tmc2209_set_ihold_irun(4, 12, 8);
  if (result != HAL_OK)
    return result;

  // Set step size.
  result = tmc2209_set_microsteps(16);
  if (result != HAL_OK)
    return result;

  // Start step PWM output.
  g_dt_ticks = 0;
  result = HAL_TIM_OC_Start_IT(&TMC2209_HTIM, TMC2209_HTIM_CHANNEL);
  if (result != HAL_OK)
    return result;

  // Enable.
  HAL_GPIO_WritePin(TMC2209_ENN_PORT, TMC2209_ENN_PIN, GPIO_PIN_RESET);

  return result;
}

HAL_StatusTypeDef tmc2209_step_set_freq_hz(const uint32_t f_step_hz) {
  if (f_step_hz == 0) {
    __disable_irq();
    g_dt_ticks = 0;
    __enable_irq();
    return HAL_TIM_OC_Stop_IT(&TMC2209_HTIM, TMC2209_HTIM_CHANNEL);
  }

  uint32_t dt = 1000000u / (2u * f_step_hz);
  if (dt < 2u)
    dt = 2u;
  if (dt > 65535u)
    dt = 65535u;

  __disable_irq();
  g_dt_ticks = (uint16_t)dt;
  const uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&TMC2209_HTIM);
  __HAL_TIM_SET_COMPARE(&TMC2209_HTIM, TMC2209_HTIM_CHANNEL,
                        (uint16_t)(now + g_dt_ticks));
  __enable_irq();

  return HAL_OK;
}

HAL_StatusTypeDef tmc2209_step_direction(const bool forward) {
  if (forward) {
    HAL_GPIO_WritePin(TMC2209_DIR_PORT, TMC2209_DIR_PIN, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(TMC2209_DIR_PORT, TMC2209_DIR_PIN, GPIO_PIN_SET);
  }

  return HAL_OK;
}
