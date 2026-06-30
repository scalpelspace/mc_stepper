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

// Controls on target constants.
#define POSITION_TOLERANCE_ENTER_RAD (0.003f)
#define POSITION_TOLERANCE_EXIT_RAD (0.015f)
#define SETPOINT_EPSILON_RAD (1e-4f)
#define ON_TARGET_DWELL_MS (25u)
#define ON_TARGET_DWELL_TICKS                                                  \
  ((ON_TARGET_DWELL_MS + CONTROLS_PERIOD_MS - 1u) / CONTROLS_PERIOD_MS)

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
  CONTROL_MODE_POSITION_OPEN_LOOP = 4,
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

/**
 * @brief Initialize the controls module.
 *
 * Initializes the encoder, PID stage with default gains/limits, and leaves the
 * FSM in CONTROLS_IDLE. Must be called before tmc2209_init().
 */
void controls_init(void);

/**
 * @brief Periodic control task, expected to execute every CONTROLS_PERIOD_MS.
 *
 * Samples the sensors, runs the control mode cascade and updates the H-bridge.
 */
void controls_run(void);

/**
 * @brief Set the active control mode and target.
 *
 * Switching mode or moving the target re-seeds the loop and enters
 * CONTROLS_EXECUTING. A command identical to the active one is ignored so
 * heartbeat retransmissions do not disturb the loop. Ignored while faulted.
 */
void controls_set_command(mc_stepper_command_t command);

/**
 * @brief Request a software zero, set to the current position.
 *
 * Sets a pending flag to service at the next controls tick, but only if the
 * loop is at a standstill (DISABLE, IDLE or ON_POSITION_TARGET).
 */
void controls_request_zero(void);

/**
 * @brief Set the control's utilized microstep size at runtime.
 *
 * Only adjustable when inactive (IDLE or DISABLE), never in a potential move.
 *
 * @param microsteps Desired microsteps per full step.
 */
bool controls_set_microsteps(uint16_t microsteps);

/**
 * @brief Read back the active microstep resolution.
 *
 * @return Microsteps per full step.
 */
uint16_t controls_get_microsteps(void);

/**
 * @brief Set the step-rate acceleration limit at runtime.
 *
 * Converts a motor-shaft acceleration (rad/s^2) to the internal steps/s^2
 * limit used by the slew-rate filter. Call after controls_init() and whenever
 * microstep configuration changes.
 *
 * @param max_accel_rad_s2 Desired acceleration limit in rad/s^2.
 */
void controls_set_max_accel_rad_s2(float max_accel_rad_s2);

/**
 * @brief Read back the motion acceleration limit.
 *
 * @return Acceleration limit in rad/s^2.
 */
float controls_get_max_accel_rad_s2(void);

/**
 * @brief Set the trapezoidal motion profile peak velocity.
 *
 * Sets the maximum velocity used by the motion profile generator for position
 * moves. Call after controls_init() and whenever microstep configuration
 * changes.
 *
 * @param max_velocity_rad_s Peak velocity in rad/s. Must be > 0.
 */
void controls_set_max_velocity_rad_s(float max_velocity_rad_s);

/**
 * @brief Read back the motion profile peak velocity.
 *
 * @return Peak velocity in rad/s.
 */
float controls_get_max_velocity_rad_s(void);

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
 * @brief Read back the active control mode.
 *
 * Reflects the mode of the command currently being executed (POSITION_RELATIVE
 * is resolved to POSITION once a move starts). Safe to call from any context.
 *
 * @return Active control_mode_t.
 */
control_mode_t controls_get_mode(void);

#endif
