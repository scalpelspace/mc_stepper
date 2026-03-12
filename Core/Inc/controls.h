/*******************************************************************************
 * @file controls.h
 * @brief Low level closed-loop controls manager.
 *******************************************************************************
 */

#ifndef MC_STEPPER__CONTROLS_H
#define MC_STEPPER__CONTROLS_H

/** Includes. *****************************************************************/

#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>

/** Definitions. **************************************************************/

// Timing.
#define CONTROLS_PERIOD_MS (1u)
#define CONTROLS_PERIOD_S ((float)CONTROLS_PERIOD_MS / 1000.0f)

// Max allowed step-rate acceleration (steps/s^2).
#ifndef MAX_STEP_ACCEL_STEPS_S2
#define MAX_STEP_ACCEL_STEPS_S2 (20000.0f)
#endif

// Controls on target constants.
#define POSITION_TOLERANCE_ENTER_RAD (0.005f)  // ~0.29 deg.
#define POSITION_TOLERANCE_EXIT_RAD (0.010f)   // Hysteresis.
#define VELOCITY_TOLERANCE_ENTER_RAD_S (0.05f) // ~2.9 deg/s.
#define VELOCITY_TOLERANCE_EXIT_RAD_S (0.10f)  // Hysteresis.
#define ON_TARGET_DWELL_MS (50u)
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

typedef struct {
  float position_rad;   // Measured absolute position (rad).
  float velocity_rad_s; // Measured velocity (rad/s).
} controls_sensors_t;

typedef enum {
  CONTROL_MODE_NONE = 0,
  CONTROL_MODE_POSITION = 1,
  CONTROL_MODE_SPEED = 2,
} control_mode_t;

typedef struct {
  control_mode_t mode; // Control mode.
  float setpoint;      // Control setpoint by mode (rad or rad/s).
} mc_stepper_command_t;

/** Public variables. *********************************************************/

extern volatile controls_state_t controls_state;     // State machine.
extern volatile controls_sensors_t controls_sensors; // Sensor data.
extern volatile float step_rate_command;             // Set as steps/s.

/** Public functions. *********************************************************/

void controls_set_mutexes(osMutexId_t setpoint_mutex,
                          osMutexId_t sensors_mutex);

void controls_init(void);

bool controls_set_microsteps(uint16_t microsteps);

/**
 * @brief Set the step-rate acceleration limit at runtime.
 *
 * Converts a motor-shaft acceleration (rad/s^2) to the internal steps/s^2
 * limit used by the slew-rate filter. Call after controls_init() and
 * whenever microstep configuration changes.
 *
 * @param accel_rad_s2  Desired acceleration limit in rad/s^2.
 */
void controls_set_accel_rad_s2(float accel_rad_s2);

void controls_set_setpoint(mc_stepper_command_t command);

void controls_state_machine(void);

#endif
