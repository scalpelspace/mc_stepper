/*******************************************************************************
 * @file tmc2209_runner.c
 * @brief TMC2209 runner: init, start reports and event handling.
 *******************************************************************************
 */
/** Includes. *****************************************************************/

#include "tmc2209_runner.h"
#include "controls.h"
#include "tmc2209_hal_uart.h"
#include <math.h>

/** Definitions. **************************************************************/

#define R_SENSE_OHM (0.110f)       // 110 mOhm external sense resistor.
#define VFS_VOLTS (0.325f)         // 0.325V (vsense=0), 0.180V (vsense=1).
#define VREF_MAX_VOLTS (2.5f)      // Max VREF specified by datasheet 2.5V.
#define VREF_DIVIDER_GAIN (0.752f) // 3.3k / 10k divider.
#define DAC_MAX_VOLTS (3.3f)       // STM32 DAC max voltage.

// TIM15 input clock after prescaler.
// dt_ticks = TIM_CLOCK_HZ / (2 * f_step_hz).
// Verify this matches CubeMX: TIM15 APB2 clock / (PSC + 1).
#define TMC2209_TIM_CLOCK_HZ (1000000ul)

/** Private variables. ********************************************************/

static volatile uint16_t g_dt_ticks; // Time between successive step signals.

// Exact step-count budget for open-loop moves. The channel runs in TOGGLE mode,
// so the STEP pin flips once per compare match and 2 toggles == 1 microstep.
// When g_step_bounded is set, the ISR counts toggles down and stops the channel
// at zero so the emitted microstep count is exact regardless of timing. Speed
// and closed-loop position run unbounded.
static volatile bool g_step_bounded = false;
static volatile uint32_t g_toggles_remaining = 0;

// Shadow of the last-applied current settings. The TMC2209 IHOLD_IRUN register
// is write-only and the run current is set via the external VREF DAC.
static uint16_t s_run_current_ma = 0;
static uint8_t s_current_run = 0;
static uint8_t s_current_hold = 0;
static uint8_t s_current_hold_delay = 0;

// StallGuard state. Detection compares the live SG_RESULT against 2*SGTHRS
// while the motor is stepping. SGTHRS is write-only on the chip, so it is
// cached here.
#define TMC2209_SG_TCOOLTHRS (0x000FFFFFu) // Keep StallGuard on when moving.
static volatile bool s_sg_enabled = false;
static volatile uint8_t s_sgthrs = 0;
static volatile uint16_t s_sg_result = 0;
static volatile bool s_sg_stalled = false;

/** User implementations into STM32 HAL (overwrite weak HAL functions). *******/

void HAL_TIM_OC_DelayElapsedCallback_tmc2209(TIM_HandleTypeDef *htim) {
  if (htim != &TMC2209_HTIM)
    return;
  if (g_dt_ticks == 0)
    return; // Stopped; don't reschedule.

  // Bounded (open-loop) moves emit an exact number of toggles. The hardware has
  // already toggled the pin for this match; account for it and stop scheduling
  // once the budget is spent so no extra steps leak out.
  if (g_step_bounded) {
    if (g_toggles_remaining != 0u)
      g_toggles_remaining--;
    if (g_toggles_remaining == 0u) {
      g_dt_ticks = 0;
      __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
      // The control loop issues a full channel stop within the next tick; the
      // stale compare cannot re-match before then (counter wrap is ~65 ms).
      return;
    }
  }

  const uint16_t ccr = __HAL_TIM_GET_COMPARE(htim, TMC2209_HTIM_CHANNEL);
  __HAL_TIM_SET_COMPARE(htim, TMC2209_HTIM_CHANNEL,
                        (uint16_t)(ccr + g_dt_ticks));
}

/** Private functions. ********************************************************/

static float dac_voltage(const float run_current_ma, uint8_t tmc2209_irun) {
  if (tmc2209_irun > 31) {
    tmc2209_irun = 31;
  }

  const float i_rms = run_current_ma / 1000.0f;
  float vref = (i_rms * 2.5f * 32.0f * (R_SENSE_OHM + 0.02f) * sqrtf(2.0f)) /
               (((float)tmc2209_irun + 1.0f) * VFS_VOLTS);

  // Clamp to safe VREF range.
  if (vref > VREF_MAX_VOLTS) {
    vref = VREF_MAX_VOLTS;
  }
  if (vref < 0.0f) {
    vref = 0.0f;
  }

  // Convert VREF to DAC voltage using divider gain.
  float dac_v = vref / VREF_DIVIDER_GAIN;
  if (dac_v > DAC_MAX_VOLTS) {
    dac_v = DAC_MAX_VOLTS;
  }
  return dac_v;
}

static uint16_t dac_counts_from_voltage(const float dac_voltage) {
  return (uint16_t)((dac_voltage / 3.3f) * 4095.0f);
}

/** Public functions. *********************************************************/

void tmc2209_enable(void) {
  HAL_GPIO_WritePin(TMC2209_ENN_PORT, TMC2209_ENN_PIN, GPIO_PIN_RESET);
}

void tmc2209_disable(void) {
  HAL_GPIO_WritePin(TMC2209_ENN_PORT, TMC2209_ENN_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef tmc2209_init(const tmc2209_configuration_t config) {
  HAL_StatusTypeDef result = HAL_OK;

  // Disable.
  tmc2209_disable();
  HAL_Delay(2); // Short delay.

  // Clear GSTAT.
  result = tmc2209_clear_gstat();
  if (result != HAL_OK)
    return result;

  // Calculate DAC output for VREF.
  const float dac_v = dac_voltage(config.run_current_ma, config.current_run);
  const uint16_t dac_counts = dac_counts_from_voltage(dac_v);

  // Enable and set DAC output for hardware voltage divider reference (VREF).
  HAL_DAC_Start(&TMC2209_VREF_DAC, TMC2209_VREF_DAC_CHANNEL);
  HAL_DAC_SetValue(&TMC2209_VREF_DAC, TMC2209_VREF_DAC_CHANNEL, DAC_ALIGN_12B_R,
                   dac_counts); // (12-bit right-aligned).

  // Current sense configuration.
  // TMC2209_VFS_VOLTS = false = 0.325 V.
  result = tmc2209_set_current_sense(false, false);
  if (result != HAL_OK)
    return result;

  // Set IHOLD, IRUN, IHOLDDELAY.
  result = tmc2209_set_ihold_irun(config.current_hold, config.current_run,
                                  config.current_hold_delay);
  if (result != HAL_OK)
    return result;

  // Cache for read-back (write-only registers, see shadow declaration).
  s_run_current_ma = config.run_current_ma;
  s_current_run = config.current_run;
  s_current_hold = config.current_hold;
  s_current_hold_delay = config.current_hold_delay;

  // Set step size.
  result = tmc2209_set_microsteps(config.step_size);
  if (result != HAL_OK)
    return result;

  // Start step PWM output.
  g_dt_ticks = 0;
  result = HAL_TIM_OC_Start_IT(&TMC2209_HTIM, TMC2209_HTIM_CHANNEL);
  if (result != HAL_OK)
    return result;

  // Apply acceleration limit from config into the controls slew filter.
  // controls_init() must have been called before tmc2209_init().
  if (config.max_acceleration_rad_s2 > 0.0f) {
    controls_set_max_accel_rad_s2(config.max_acceleration_rad_s2);
  }

  if (config.enable) {
    // Enable.
    tmc2209_enable();
  }

  return result;
}

HAL_StatusTypeDef tmc2209_set_currents(const uint16_t run_current_ma,
                                       const uint8_t current_run,
                                       const uint8_t current_hold,
                                       const uint8_t current_hold_delay) {
  // Recompute and apply the VREF DAC output for the run current.
  const float dac_v = dac_voltage((float)run_current_ma, current_run);
  const uint16_t dac_counts = dac_counts_from_voltage(dac_v);
  HAL_DAC_SetValue(&TMC2209_VREF_DAC, TMC2209_VREF_DAC_CHANNEL, DAC_ALIGN_12B_R,
                   dac_counts); // (12-bit right-aligned).

  // Cache for read-back (write-only registers, see shadow declaration).
  s_run_current_ma = run_current_ma;
  s_current_run = current_run;
  s_current_hold = current_hold;
  s_current_hold_delay = current_hold_delay;

  // Update IHOLD, IRUN, IHOLDDELAY.
  return tmc2209_set_ihold_irun(current_hold, current_run, current_hold_delay);
}

void tmc2209_get_currents(uint16_t *const run_current_ma,
                          uint8_t *const current_run,
                          uint8_t *const current_hold,
                          uint8_t *const current_hold_delay) {
  if (run_current_ma)
    *run_current_ma = s_run_current_ma;
  if (current_run)
    *current_run = s_current_run;
  if (current_hold)
    *current_hold = s_current_hold;
  if (current_hold_delay)
    *current_hold_delay = s_current_hold_delay;
}

HAL_StatusTypeDef tmc2209_set_step_size(const uint16_t step_size) {
  return tmc2209_set_microsteps(step_size);
}

HAL_StatusTypeDef tmc2209_stallguard_config(const bool enable,
                                            const uint8_t sgthrs) {
  s_sg_enabled = enable;
  s_sgthrs = sgthrs;
  s_sg_stalled = false; // Clear any latched stall on reconfiguration.

  HAL_StatusTypeDef result = tmc2209_set_sgthrs(sgthrs);
  if (result != HAL_OK)
    return result;

  // Gate StallGuard on velocity: permissive when enabled, off otherwise.
  return tmc2209_set_tcoolthrs(enable ? TMC2209_SG_TCOOLTHRS : 0u);
}

void tmc2209_stallguard_get(bool *const enable, uint8_t *const sgthrs,
                            uint16_t *const sg_result, bool *const stalled) {
  if (enable)
    *enable = s_sg_enabled;
  if (sgthrs)
    *sgthrs = s_sgthrs;
  if (sg_result)
    *sg_result = s_sg_result;
  if (stalled)
    *stalled = s_sg_stalled;
}

bool tmc2209_stallguard_poll(void) {
  if (!s_sg_enabled)
    return false;

  // SG_RESULT is only meaningful while stepping (g_dt_ticks != 0 means the step
  // timer is active). At standstill clear the latch so a stall does not persist.
  if (g_dt_ticks == 0u) {
    s_sg_stalled = false;
    return false;
  }

  uint16_t sg = 0;
  if (tmc2209_get_sg_result(&sg) != HAL_OK) {
    return false; // Transient UART error, re-evaluate next cycle.
  }
  s_sg_result = sg;

  const bool stalled = (sg <= (uint16_t)(2u * (uint16_t)s_sgthrs));
  const bool newly_stalled = (stalled && !s_sg_stalled);
  s_sg_stalled = stalled;
  return newly_stalled;
}

HAL_StatusTypeDef tmc2209_step_set_freq_hz(const uint32_t f_step_hz) {
  if (f_step_hz == 0) {
    __disable_irq();
    g_dt_ticks = 0;
    __enable_irq();
    // Stop the channel interrupt generation first.
    __HAL_TIM_DISABLE_IT(&TMC2209_HTIM, TIM_IT_CC2);   // CH2 compare.
    __HAL_TIM_CLEAR_FLAG(&TMC2209_HTIM, TIM_FLAG_CC2); // CH2 compare flag.
    return HAL_TIM_OC_Stop_IT(&TMC2209_HTIM, TMC2209_HTIM_CHANNEL);
  }

  uint32_t dt = TMC2209_TIM_CLOCK_HZ / (2u * f_step_hz);
  if (dt < 2u)
    dt = 2u;
  if (dt > 65535u)
    dt = 65535u;

  __disable_irq();
  // The OC channel runs in TOGGLE mode: the STEP pin toggles in hardware on
  // each compare match and the OC ISR advances the compare by g_dt_ticks. This
  // function is called every control tick to update the rate, so the next
  // compare must NOT be re-seeded while already running. Doing so resets the
  // accumulated toggle phase each tick and systematically drops a fraction of a
  // step, which a closed loop hides via PID, but open-loop position cannot.
  // Only seed the first compare when starting from a stopped state. While
  // running the ISR adopts the new g_dt_ticks on its next edge.
  const bool was_running = (g_dt_ticks != 0u);
  g_dt_ticks = (uint16_t)dt;
  if (!was_running) {
    const uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&TMC2209_HTIM);
    __HAL_TIM_SET_COMPARE(&TMC2209_HTIM, TMC2209_HTIM_CHANNEL,
                          (uint16_t)(now + g_dt_ticks));
    __HAL_TIM_CLEAR_FLAG(&TMC2209_HTIM, TIM_FLAG_CC2); // Drop stale match.
  }
  __enable_irq();

  // Ensure running (idempotent if already started).
  return HAL_TIM_OC_Start_IT(&TMC2209_HTIM, TMC2209_HTIM_CHANNEL);
}

void tmc2209_step_load_count(const uint32_t microsteps) {
  __disable_irq();
  g_toggles_remaining = microsteps * 2u; // 2 toggles per microstep.
  g_step_bounded = true;
  __enable_irq();
}

void tmc2209_step_clear_count(void) {
  __disable_irq();
  g_step_bounded = false;
  g_toggles_remaining = 0u;
  __enable_irq();
}

uint32_t tmc2209_step_remaining(void) {
  __disable_irq();
  const bool bounded = g_step_bounded;
  const uint32_t toggles = g_toggles_remaining;
  __enable_irq();
  return bounded ? ((toggles + 1u) / 2u) : 0u;
}

HAL_StatusTypeDef tmc2209_step_direction(const bool forward) {
  if (forward) {
    HAL_GPIO_WritePin(TMC2209_DIR_PORT, TMC2209_DIR_PIN, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(TMC2209_DIR_PORT, TMC2209_DIR_PIN, GPIO_PIN_RESET);
  }

  return HAL_OK;
}
