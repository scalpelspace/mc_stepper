/*******************************************************************************
 * @file controls.c
 * @brief Low level closed-loop controls manager.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "controls.h"
#include "pid.h"
#include <math.h>
#include <stdbool.h>

/** Public variables. *********************************************************/

controls_state_t controls_state = CONTROLS_DISABLE;

// Sensor data.
float sensor_position_rad = 0;
float sensor_velocity_rad_s = 0;

/** Private variables. ********************************************************/

static mc_stepper_command_t control_command = {0}; // Command input.

static control_mode_t prev_mode = CONTROL_MODE_NONE; // Previous control mode.

// Controls inputs and outputs.
static float position_setpoint = 0; // Set as rad.
static float velocity_setpoint = 0; // Set as rad/s.
static float step_rate_command = 0; // Set as steps/s.

static pid_controller_t position_pd = {0}; // Outputs velocity (rad/s).
static pid_controller_t velocity_pi = {0}; // Outputs step-rate (steps/s).

// Stepper motor configuration.
static uint16_t full_steps_per_rev = 200;
static uint8_t microstep_config = 16;
static float steps_per_rad;

static uint16_t on_target_ticks = 0;   // Position on-target latch count.
static bool latched_on_target = false; // Position on-target latching check.

/** Private functions. ********************************************************/

static void control_loop(void) {
  const mc_stepper_command_t cmd = control_command;
  switch (cmd.mode) {

  case CONTROL_MODE_SPEED:
    velocity_setpoint = cmd.setpoint; // (rad/s) to steps/s.
    step_rate_command =
        pid_update(&velocity_pi, velocity_setpoint * steps_per_rad,
                   sensor_velocity_rad_s * steps_per_rad);
    break;

  case CONTROL_MODE_POSITION:
    position_setpoint = cmd.setpoint; // (rad) to steps.
    velocity_setpoint =
        pid_update(&position_pd, position_setpoint, sensor_position_rad);
    step_rate_command =
        pid_update(&velocity_pi, velocity_setpoint * steps_per_rad,
                   sensor_velocity_rad_s * steps_per_rad);
    break;

  default:
    step_rate_command = 0.0f;
    break;
  }
}

static bool on_position_target(void) {
  // Only meaningful for position moves.
  if (control_command.mode != CONTROL_MODE_POSITION)
    return false;

  // Choose hysteresis thresholds depending on current latched state.
  const float pos_tol = latched_on_target ? POSITION_TOLERANCE_EXIT_RAD
                                          : POSITION_TOLERANCE_ENTER_RAD;
  const float vel_tol = latched_on_target ? VELOCITY_TOLERANCE_EXIT_RAD_S
                                          : VELOCITY_TOLERANCE_ENTER_RAD_S;

  const float pos_err = position_setpoint - sensor_position_rad;
  const float vel_abs = fabsf(sensor_velocity_rad_s);

  const bool in_band = (fabsf(pos_err) <= pos_tol) && (vel_abs <= vel_tol);

  if (in_band) {
    if (on_target_ticks < ON_TARGET_DWELL_TICKS)
      on_target_ticks++;
  } else {
    // Reset position on target tick counter and latch variable.
    on_target_ticks = 0;
    latched_on_target = false;
  }
  if (on_target_ticks >= ON_TARGET_DWELL_TICKS) {
    latched_on_target = true;
  }

  return latched_on_target;
}

/** Public functions. *********************************************************/

void controls_init(void) {
  // Calculate step size config.
  steps_per_rad = ((float)full_steps_per_rev * (float)microstep_config) /
                  (2.0f * (float)M_PI);

  // Position.
  pid_init(&position_pd);
  position_pd.k_p = 8.0f;
  position_pd.k_i = 0.0f;
  position_pd.k_d = 0.15f;
  position_pd.tau = 0.02f;
  position_pd.output_min = -20.0f;
  position_pd.output_max = 20.0f;
  position_pd.integral_min = 0.0f;
  position_pd.integral_max = 0.0f;
  position_pd.T = CONTROLS_PERIOD_S;

  // Velocity.
  pid_init(&velocity_pi);
  velocity_pi.k_p = 0.5f;
  velocity_pi.k_i = 50.0f;
  velocity_pi.k_d = 0.0f;
  velocity_pi.tau = 0.0f;
  velocity_pi.output_min = -5000.0f;
  velocity_pi.output_max = 5000.0f;
  velocity_pi.integral_min = -5000.0f;
  velocity_pi.integral_max = 5000.0f;
  velocity_pi.T = CONTROLS_PERIOD_S;

  // Reset position on target tick counter and latch variable.
  on_target_ticks = 0;
  latched_on_target = false;
}

void controls_set_microsteps(const uint8_t microsteps) {
  // TODO: Improve to allow adjustment of microstepping at standstill.
  microstep_config = microsteps;
  steps_per_rad = ((float)full_steps_per_rev * (float)microstep_config) /
                  (2.0f * (float)M_PI);
}

void controls_command(const mc_stepper_command_t command) {
  control_command = command;
  if (control_command.mode != prev_mode) {
    pid_init(&velocity_pi); // Reset, gains uneffected by init.
    pid_init(&position_pd); // Reset, gains uneffected by init.
    prev_mode = control_command.mode;
  }

  // Reset position on target tick counter and latch variable.
  on_target_ticks = 0;
  latched_on_target = false;

  controls_state = CONTROLS_EXECUTING; // Transition state.
}

void controls_state_machine(void) {
  switch (controls_state) {
  case CONTROLS_DISABLE:
    step_rate_command = 0.0f; // Safety.
    break;
  case CONTROLS_IDLE:
    // Wait for new command.
    break;
  case CONTROLS_EXECUTING:
    control_loop();
    if (on_position_target()) {
      // Transition state.
      controls_state = CONTROLS_ON_POSITION_TARGET;
    }
    break;
  case CONTROLS_ON_POSITION_TARGET:
    // TODO: TEST! Transition condition from ON_POSITION_TARGET to IDLE.
    controls_state = CONTROLS_IDLE;
    break;
  case CONTROLS_FAULT:
    break;
  default:
    break;
  }
}
