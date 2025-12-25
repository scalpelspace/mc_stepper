/*******************************************************************************
 * @file abi_encoder_hal_tim.c
 * @brief ABI (quadrature) encoder functions: abstracting STM32 HAL: TIM.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "abi_encoder_hal_tim.h"

/** Public functions. *********************************************************/

void abi_encoder_init(abi_encoder_t *encoder, TIM_HandleTypeDef *htim) {
  encoder->htim = htim;
  encoder->previous_count = 0;
  encoder->delta = 0;

  HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(encoder->htim, 0);
}

void abi_encoder_update(abi_encoder_t *encoder) {
  const int32_t current = (int32_t)__HAL_TIM_GET_COUNTER(encoder->htim);

  encoder->delta = current - encoder->previous_count;
  encoder->previous_count = current;
}

void abi_encoder_reset(abi_encoder_t *encoder) {
  __HAL_TIM_SET_COUNTER(encoder->htim, 0);
  encoder->previous_count = 0;
  encoder->delta = 0;
}
