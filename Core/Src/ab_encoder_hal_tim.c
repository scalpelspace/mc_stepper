/*******************************************************************************
 * @file ab_encoder_hal_tim.c
 * @brief AB (quadrature) encoder functions: abstracting STM32 HAL: TIM.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "ab_encoder_hal_tim.h"

/** Public functions. *********************************************************/

void ab_encoder_init(ab_encoder_t *encoder, TIM_HandleTypeDef *htim) {
  encoder->htim = htim;
  encoder->previous_count = 0;
  encoder->delta = 0;

  HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(encoder->htim, 0);
}

void ab_encoder_update(ab_encoder_t *encoder) {
  const int32_t current = (int32_t)__HAL_TIM_GET_COUNTER(encoder->htim);

  encoder->delta = current - encoder->previous_count;
  encoder->previous_count = current;
}

void ab_encoder_reset(ab_encoder_t *encoder) {
  __HAL_TIM_SET_COUNTER(encoder->htim, 0);
  encoder->previous_count = 0;
  encoder->delta = 0;
}
