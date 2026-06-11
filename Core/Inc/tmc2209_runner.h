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
 * @return HAL_StatusTypeDef status.
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
 * @return HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_set_currents(uint16_t run_current_ma,
                                       uint8_t current_run,
                                       uint8_t current_hold,
                                       uint8_t current_hold_delay);

/**
 * @brief Read back the last-applied current settings.
 *
 * Reports cached values: the TMC2209 IHOLD_IRUN register is write-only and the
 * run current is set via the external VREF DAC, so neither can be read from the
 * chip. All out-params are nullable.
 *
 * @param run_current_ma Output cached run current target (mA).
 * @param current_run Output cached IRUN scaler.
 * @param current_hold Output cached IHOLD scaler.
 * @param current_hold_delay Output cached IHOLDDELAY.
 */
void tmc2209_get_currents(uint16_t *run_current_ma, uint8_t *current_run,
                          uint8_t *current_hold, uint8_t *current_hold_delay);

/**
 * @brief Update the driver microstep resolution without a full re-init.
 *
 * Driver-side counterpart to controls_set_microsteps(). Used by the CAN motion
 * reconfiguration path.
 *
 * @param step_size Microstep setting [1,256].
 *
 * @return HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_set_step_size(uint16_t step_size);

/**
 * @brief Configure StallGuard detection.
 *
 * Writes the SGTHRS threshold and gates StallGuard on velocity via TCOOLTHRS.
 * Performs blocking UART writes; call from the main context, not an ISR.
 *
 * @param enable Enable StallGuard polling/detection.
 * @param sgthrs StallGuard threshold [0,255]; stall when SG_RESULT <= 2*sgthrs.
 *
 * @return HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_stallguard_config(bool enable, uint8_t sgthrs);

/**
 * @brief Read back the StallGuard configuration and latest result.
 *
 * All out-params are nullable.
 *
 * @param enable Output StallGuard enabled state.
 * @param sgthrs Output cached SGTHRS threshold.
 * @param sg_result Output the last polled SG_RESULT load value.
 * @param stalled Output the current latched stall state.
 */
void tmc2209_stallguard_get(bool *enable, uint8_t *sgthrs, uint16_t *sg_result,
                            bool *stalled);

/**
 * @brief Poll StallGuard once; detect a stall edge.
 *
 * When enabled and stepping, reads SG_RESULT and updates the latched stall
 * state. Performs a blocking UART read; call from the main context (e.g. a
 * scheduler task), not an ISR.
 *
 * @return true on a rising stall edge (newly stalled this call), else false.
 */
bool tmc2209_stallguard_poll(void);

/**
 * @brief Set step frequency in steps/sec (updates compare interval).
 *
 * @param f_step_hz Target step frequency in hertz to set.
 *
 * @return HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_step_set_freq_hz(uint32_t f_step_hz);

/**
 * @brief Arm an exact microstep budget for the step generator.
 *
 * After this call the generator emits at most @p microsteps STEP pulses and
 * then stops itself, independent of how long the commanded frequency is left
 * running. Pair the rate updates via tmc2209_step_set_freq_hz() with the budget
 * loaded here.
 *
 * @param microsteps Number of STEP pulses to emit before stopping.
 */
void tmc2209_step_load_count(uint32_t microsteps);

/**
 * @brief Remove the step budget; the generator free-runs at the set frequency.
 *
 * Used for speed and closed-loop position modes, which are rate-controlled and
 * must not self-terminate by count.
 */
void tmc2209_step_clear_count(void);

/**
 * @brief Microsteps still pending in the active budget.
 *
 * @return Remaining microsteps budget, 0 for unbounded or move complete.
 */
uint32_t tmc2209_step_remaining(void);

/**
 * @brief Set step direction.
 *
 * @param forward True for forward (low), false for backwards (high).
 *
 * @return HAL_StatusTypeDef status.
 */
HAL_StatusTypeDef tmc2209_step_direction(bool forward);

#endif
