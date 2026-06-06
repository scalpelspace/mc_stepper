/*******************************************************************************
 * @file tmc2209_runner.h
 * @brief TMC2209 runner: init, start reports and event handling.
 *******************************************************************************
 */

#ifndef MC_STEPPER__TMC2209_RUNNER_H
#define MC_STEPPER__TMC2209_RUNNER_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/** Public types. *************************************************************/

typedef struct {
  bool enable;
  uint16_t step_size;            // [1,256] full step to 1/256 microstep.
  uint16_t run_current_ma;       // TMC2209 current limit mA.
  uint8_t current_hold;          // TMC2209 IHOLD.
  uint8_t current_run;           // TMC2209 IRUN.
  uint8_t current_hold_delay;    // TMC2209 IHOLDDELAY.
  float max_acceleration_rad_s2; // Controls acceleration limit in rad/s^2.
} tmc2209_configuration_t;

/** User implementations into STM32 HAL (overwrite weak HAL functions). *******/

void HAL_TIM_OC_DelayElapsedCallback_tmc2209(TIM_HandleTypeDef *htim);

/** Public functions. *********************************************************/

/**
 * @brief Enable the TMC2209.
 */
void tmc2209_enable(void);

/**
 * @brief Disable the TMC2209.
 */
void tmc2209_disable(void);

/**
 * @brief Initialize peripherals for the TMC2209.
 *
 * @param config TMC2209 configuration defined by tmc2209_configuration_t.
 *
 * @returns HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_init(tmc2209_configuration_t config);

/**
 * @brief Update the run/hold currents without a full re-init.
 *
 * Recomputes the VREF DAC output from run_current_ma and current_run, then
 * writes IHOLD/IRUN/IHOLDDELAY. Used by the CAN current reconfiguration path.
 *
 * @param run_current_ma Target RMS run current in mA (drives the VREF DAC).
 * @param current_run TMC2209 IRUN scaler [0,31].
 * @param current_hold TMC2209 IHOLD scaler [0,31].
 * @param current_hold_delay TMC2209 IHOLDDELAY [0,15].
 *
 * @returns HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_set_currents(uint16_t run_current_ma,
                                       uint8_t current_run,
                                       uint8_t current_hold,
                                       uint8_t current_hold_delay);

/**
 * @brief Update the driver microstep resolution without a full re-init.
 *
 * Driver-side counterpart to controls_set_microsteps(). Used by the CAN motion
 * reconfiguration path.
 *
 * @param step_size Microstep setting [1,256].
 *
 * @returns HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_set_step_size(uint16_t step_size);

/**
 * @brief Set step frequency in steps/sec (updates compare interval).
 *
 * @param f_step_hz Target step frequency in hertz to set.
 *
 * @returns HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_step_set_freq_hz(uint32_t f_step_hz);

/**
 * @brief Set step direction.
 *
 * @param forward True for forward (low), false for backwards (high).
 *
 * @returns HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_step_direction(bool forward);

#endif
