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

/** Definitions. **************************************************************/

// TIM15 input clock after prescaler.
// dt_ticks = TMC2209_TIM_CLOCK_HZ / (2 * f_step_hz).
#define TMC2209_TIM_CLOCK_HZ (1000000ul)

// Step generator rate limits. The STEP pin toggles in hardware on each compare
// match, so a step period spans two compares of dt_ticks each, and dt_ticks
// has to fit the 16-bit compare register. Both bounds therefore follow from
// the timer tick rate alone.
#define TMC2209_STEP_MIN_RATE_HZ                                               \
  ((float)TMC2209_TIM_CLOCK_HZ / (2.0f * 65535.0f))
#define TMC2209_STEP_MAX_RATE_HZ ((float)TMC2209_TIM_CLOCK_HZ / (2.0f * 2.0f))

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
 * Performs a blocking UART write, call from the main context, not an ISR.
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
 * @brief Poll StallGuard once, detect a stall edge.
 *
 * When enabled and stepping, reads SG_RESULT and updates the latched stall
 * state. Performs a blocking UART read, call from the main context, not an ISR.
 *
 * @return true on a rising stall edge (newly stalled this call), else false.
 */
bool tmc2209_stallguard_poll(void);

/**
 * @brief Poll the TMC2209 GSTAT/DRV_STATUS registers and cache a packed fault
 * status word for CAN reporting.
 *
 *  Performs a blocking UART read, call from the main context, not an ISR. On a
 * transient UART error the previously cached value is left unchanged
 * (re-evaluate next cycle) rather than reporting a false clear.
 *
 * Packed bit layout (bits 15:11 reserved, always 0):
 *   0 gstat.reset    1 gstat.drv_err  2 gstat.uv_cp
 *   3 drv_status.otpw   4 drv_status.ot
 *   5 drv_status.s2ga   6 drv_status.s2gb
 *   7 drv_status.s2vsa  8 drv_status.s2vsb
 *   9 drv_status.ola    10 drv_status.olb
 *
 * @return true if both registers were read successfully this call.
 */
bool tmc2209_status_poll(void);

/**
 * @brief Read back the last-polled packed TMC2209 fault status word.
 *
 * @return Packed status word, see tmc2209_status_poll() for the bit layout.
 */
uint16_t tmc2209_get_status(void);

/**
 * @brief Set the step rate in steps/sec (updates the compare interval).
 *
 * Takes a fractional rate and rounds the compare period from it. Rounding the
 * rate to an integer hertz first would discard up to a full step per second,
 * which at coarse microstepping is a large fraction of the commanded velocity.
 *
 * Rates at or below zero stop the generator. Rates below
 * TMC2209_STEP_MIN_RATE_HZ cannot be produced by the 16-bit compare register
 * and are refused rather than clamped, since clamping would silently run the
 * motor faster than commanded. Rates above TMC2209_STEP_MAX_RATE_HZ are
 * clamped to the ceiling.
 *
 * @param f_step_hz Target step rate in hertz. Fractional values are honoured.
 *
 * @return HAL_StatusTypeDef status.
 * @retval HAL_ERROR -> Rate below TMC2209_STEP_MIN_RATE_HZ, generator stopped.
 */
HAL_StatusTypeDef tmc2209_step_set_rate_hz(float f_step_hz);

/**
 * @brief Arm an exact microstep budget for the step generator.
 *
 * After this call the generator emits at most @p microsteps STEP pulses and
 * then stops itself, independent of how long the commanded frequency is left
 * running. Pair the rate updates via tmc2209_step_set_rate_hz() with the budget
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
