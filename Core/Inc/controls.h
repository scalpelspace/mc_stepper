/*******************************************************************************
 * @file controls.h
 * @brief Low level closed-loop controls manager.
 *******************************************************************************
 */

#ifndef MC_STEPPER__CONTROLS_H
#define MC_STEPPER__CONTROLS_H

/** Includes. *****************************************************************/

#include <stdbool.h>
#include <stdint.h>

/** Definitions. **************************************************************/

// Timing.
#define CONTROLS_PERIOD_MS (1u)
#define CONTROLS_PERIOD_S ((float)CONTROLS_PERIOD_MS / 1000.0f)

// Max allowed step-rate acceleration (steps/s^2).
#ifndef MAX_STEP_ACCEL_STEPS_S2
#define MAX_STEP_ACCEL_STEPS_S2 (8000.0f)
#endif

// Controls on target constants.
#define POSITION_TOLERANCE_ENTER_RAD (0.0025f)
#define POSITION_TOLERANCE_EXIT_RAD (0.015f)
#define SETPOINT_EPSILON_RAD (1e-4f)
#define ON_TARGET_DWELL_MS (25u)
#define ON_TARGET_DWELL_TICKS                                                  \
  ((ON_TARGET_DWELL_MS + CONTROLS_PERIOD_MS - 1u) / CONTROLS_PERIOD_MS)

// PID gain-scheduling thresholds. The integrator only winds up when the rotor
// is within the fine-tune band, so a large move does not accumulate windup
// that would overshoot. Hysteresis between enter/exit prevents chattering.
#define PID_FINE_REGIME_ENTER_RAD (0.010f)
#define PID_FINE_REGIME_EXIT_RAD (0.020f)

/** Public types. *************************************************************/

typedef enum {
  CONTROLS_DISABLE = 0,
  CONTROLS_IDLE = 1,
  CONTROLS_START = 2,
  CONTROLS_EXECUTING = 3,
  CONTROLS_ON_POSITION_TARGET = 4,
  CONTROLS_FAULT = 5,
} controls_state_t;

typedef enum {
  CONTROL_MODE_NONE = 0,
  CONTROL_MODE_POSITION = 1,
  CONTROL_MODE_SPEED = 2,
  CONTROL_MODE_POSITION_RELATIVE = 3,
} control_mode_t;

typedef struct {
  control_mode_t mode; // Control mode.
  float setpoint;      // Control setpoint by mode (rad or rad/s).
} mc_stepper_command_t;

/** Public variables. *********************************************************/

extern volatile controls_state_t controls_state; // State machine.

// Controls inputs and outputs, exposed partially for diagnostics.
extern volatile float controls_position_meas_rad; // Sensor data.
extern volatile float controls_position_setpoint; // rad, outer loop target.
extern volatile float controls_velocity_setpoint; // rad/s, inner loop target.
extern volatile float controls_step_rate_command; // steps/s, control output.

/** Public functions. *********************************************************/

void controls_init(void);

bool controls_set_microsteps(uint16_t microsteps);

/**
 * @brief Set the step-rate acceleration limit at runtime.
 *
 * Converts a motor-shaft acceleration (rad/s^2) to the internal steps/s^2
 * limit used by the slew-rate filter. Call after controls_init() and
 * whenever microstep configuration changes.
 *
 * @param accel_rad_s2 Desired acceleration limit in rad/s^2.
 */
void controls_set_accel_rad_s2(float accel_rad_s2);

/**
 * @brief Set the trapezoidal motion profile peak velocity.
 *
 * Sets the maximum velocity used by the motion profile generator for position
 * moves. Pair with controls_set_accel_rad_s2() to shape the full trapezoid.
 *
 * @param v_max_rad_s Peak velocity in rad/s. Must be > 0.
 */
void controls_set_max_velocity_rad_s(float v_max_rad_s);

void controls_set_setpoint(mc_stepper_command_t command);

/**
 * @brief Live-tune the position PID gains.
 *
 * Safe to call from any context. Does not reset integrator/derivative state,
 * so use small steps to avoid step-input transients.
 */
void controls_set_gains(float k_p, float k_i, float k_d);

/**
 * @brief Read back the current position PID gains.
 *
 * @param k_p Out. Nullable.
 * @param k_i Out. Nullable.
 * @param k_d Out. Nullable.
 */
void controls_get_gains(float *k_p, float *k_i, float *k_d);

/**
 * @brief Shift the controls-layer position frame by delta_rad.
 *
 * @param delta_rad Position delta in radians.
 *
 * Adds delta_rad to the active position setpoint (and to the pending command
 * setpoint when in POSITION mode), then re-inits both PIDs to avoid a
 * derivative kick across the frame change.
 *
 * Pair with a matching shift of the sensor-side unwrapped position so the
 * physical target is preserved.
 *
 * Not safe to call while CONTROLS_EXECUTING or CONTROLS_START. The caller must
 * gate on a standstill state (DISABLE, IDLE or ON_POSITION_TARGET).
 */
void controls_redatum_shift(float delta_rad);

void controls_state_machine(void);

#endif
