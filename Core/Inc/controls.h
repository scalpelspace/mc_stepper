/*******************************************************************************
 * @file controls.h
 * @brief Low level closed-loop controls manager.
 *******************************************************************************
 */

#ifndef MC_STEPPER__CONTROLS_H
#define MC_STEPPER__CONTROLS_H

/** Includes. *****************************************************************/

#include <stdint.h>

/** Definitions. **************************************************************/

// Timing.
#define CONTROLS_PERIOD_MS (1U)
#define CONTROLS_PERIOD_S ((float)CONTROLS_PERIOD_MS / 1000.0f)

// Controls on target constants.
#define POSITION_TOLERANCE_ENTER_RAD (0.005f)  // ~0.29 deg.
#define POSITION_TOLERANCE_EXIT_RAD (0.010f)   // Hysteresis.
#define VELOCITY_TOLERANCE_ENTER_RAD_S (0.05f) // ~2.9 deg/s.
#define VELOCITY_TOLERANCE_EXIT_RAD_S (0.10f)  // Hysteresis.
#define ON_TARGET_DWELL_MS (50U)
#define ON_TARGET_DWELL_TICKS                                                  \
  ((ON_TARGET_DWELL_MS + CONTROLS_PERIOD_MS - 1U) / CONTROLS_PERIOD_MS)

/** Public types. *************************************************************/

typedef enum {
  CONTROLS_DISABLE = 0,
  CONTROLS_IDLE = 1,
  CONTROLS_EXECUTING = 2,
  CONTROLS_ON_POSITION_TARGET = 3,
  CONTROLS_FAULT = 4,
} controls_state_t;

typedef enum {
  CONTROL_MODE_NONE = 0,
  CONTROL_MODE_POSITION = 1,
  CONTROL_MODE_SPEED = 2,
} control_mode_t;

typedef struct {
  control_mode_t mode; // Control mode.
  float setpoint;      // Control setpoint by mode (rad or rad/s).
  float timestamp;     // Timestamp.
} mc_stepper_command_t;

/** Public variables. *********************************************************/

extern controls_state_t controls_state;

// Sensor data.
extern float sensor_position_rad;   // Measured absolute position (rad).
extern float sensor_velocity_rad_s; // Measured velocity (rad/s).

// Controls inputs and outputs.
extern float position_setpoint; // Position setpoint (input, rad).
extern float velocity_setpoint; // Velocity setpoint (input, rad/s).
extern float step_rate_command; // Velocity command (output, steps/s).

/** Public functions. *********************************************************/

void controls_init(void);
void controls_set_microsteps(uint8_t microsteps);
void controls_command(mc_stepper_command_t command);
void controls_state_machine(void);

#endif
