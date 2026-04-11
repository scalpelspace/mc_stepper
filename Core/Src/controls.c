/*******************************************************************************
 * @file controls.c
 * @brief Low level closed-loop controls manager.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "controls.h"
#include "pid.h"
#include <math.h>

/** Public variables. *********************************************************/

volatile controls_state_t controls_state = CONTROLS_DISABLE;
volatile controls_sensors_t controls_sensors = {0};
volatile float step_rate_command = 0;

/** Private variables. ********************************************************/

static mc_stepper_command_t control_command = {0}; // Command input.

static control_mode_t prev_mode = CONTROL_MODE_NONE; // Previous control mode.

// Controls inputs and outputs.
static float position_setpoint = 0; // Set as rad.
static float velocity_setpoint = 0; // Set as rad/s.

static pid_controller_t position_pd = {0}; // Outputs velocity (rad/s).
static pid_controller_t velocity_pi = {0}; // Outputs step-rate (steps/s).

// Stepper motor configuration.
static uint16_t full_steps_per_rev = 200;
static uint16_t microstep_config = 16;
static float steps_per_rad;

// Runtime acceleration limit (steps/s^2). Defaults to the compile-time value.
// Overridden via controls_set_accel_rad_s2() once steps_per_rad is known.
static float s_max_step_accel = MAX_STEP_ACCEL_STEPS_S2;

static uint16_t on_target_ticks = 0;   // Position on-target latch count.
static bool latched_on_target = false; // Position on-target latching check.

/** Private snapshot functions. ***********************************************/

static void command_snapshot(mc_stepper_command_t *command) {
  *command = control_command;
}

static void sensors_snapshot(controls_sensors_t *out) {
  *out = controls_sensors;
}

/** Private functions. ********************************************************/

static float clampf(const float x, const float lo, const float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

/**
 * @brief Slew-limit the change in step rate to enforce a max acceleration.
 *
 * @param target Target step rate (steps/s).
 *
 * @return Slew-limited step rate command (steps/s).
 */
static float step_rate_slew_limit(const float target) {
  const float max_delta =
      s_max_step_accel * CONTROLS_PERIOD_S; // steps/s per tick.
  const float delta =
      clampf(target - step_rate_command, -max_delta, +max_delta);
  return step_rate_command + delta;
}

static void control_loop(const mc_stepper_command_t command,
                         const controls_sensors_t sensors) {
  float step_rate_target;

  switch (command.mode) {

  case CONTROL_MODE_SPEED:
    velocity_setpoint = command.setpoint;
    step_rate_target =
        pid_update(&velocity_pi, velocity_setpoint * steps_per_rad,
                   sensors.velocity_rad_s * steps_per_rad);
    break;

  case CONTROL_MODE_POSITION:
    position_setpoint = command.setpoint;
    velocity_setpoint =
        pid_update(&position_pd, position_setpoint, sensors.position_rad);
    step_rate_target =
        pid_update(&velocity_pi, velocity_setpoint * steps_per_rad,
                   sensors.velocity_rad_s * steps_per_rad);
    break;

  default:
    step_rate_target = 0.0f;
    break;
  }

  // Apply step-rate acceleration/slew limiting (trapezoid-like behavior).
  step_rate_command = step_rate_slew_limit(step_rate_target);
}

static bool on_position_target(const mc_stepper_command_t command,
                               const controls_sensors_t sensors) {
  // Only meaningful for position moves.
  if (command.mode != CONTROL_MODE_POSITION)
    return false;

  // Choose hysteresis thresholds depending on current latched state.
  const float pos_tol = latched_on_target ? POSITION_TOLERANCE_EXIT_RAD
                                          : POSITION_TOLERANCE_ENTER_RAD;
  const float vel_tol = latched_on_target ? VELOCITY_TOLERANCE_EXIT_RAD_S
                                          : VELOCITY_TOLERANCE_ENTER_RAD_S;

  const float pos_err = command.setpoint - sensors.position_rad;
  const float vel_abs = fabsf(sensors.velocity_rad_s);

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

void controls_set_accel_rad_s2(const float accel_rad_s2) {
  // Convert rad/s^2 to steps/s^2 using the current steps_per_rad calibration.
  // steps_per_rad is valid after controls_init() or controls_set_microsteps().
  s_max_step_accel = accel_rad_s2 * steps_per_rad;
}

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
  velocity_pi.k_p = 0.3f;
  velocity_pi.k_i = 3.0f;
  velocity_pi.k_d = 0.0f;
  velocity_pi.tau = 0.0f;
  velocity_pi.output_min = -5000.0f;
  velocity_pi.output_max = 5000.0f;
  velocity_pi.integral_min = -500.0f;
  velocity_pi.integral_max = 500.0f;
  velocity_pi.T = CONTROLS_PERIOD_S;

  // Reset position on target tick counter and latch variable.
  on_target_ticks = 0;
  latched_on_target = false;
}

bool controls_set_microsteps(const uint16_t microsteps) {
  // TODO: Improve to allow adjustment of microstepping at standstill.
  if (controls_state == CONTROLS_IDLE) {
    microstep_config = microsteps;
    steps_per_rad = ((float)full_steps_per_rev * (float)microstep_config) /
                    (2.0f * (float)M_PI);
    return true;
  }
  return false;
}

void controls_set_setpoint(const mc_stepper_command_t command) {
  // TODO: Implement command override policy.
  control_command = command;
  controls_state = CONTROLS_START; // Transition state.
}

void controls_state_machine(void) {
  mc_stepper_command_t cmd = {0};
  controls_sensors_t data = {0};

  switch (controls_state) {
  case CONTROLS_DISABLE:
    step_rate_command = 0.0f; // Safety.
    break;

  case CONTROLS_IDLE:
    // Wait for new command.
    break;

  case CONTROLS_START:
    command_snapshot(&cmd);

    // Relative position control.
    if (cmd.mode == CONTROL_MODE_POSITION_RELATIVE) {
      // Resolve a relative position command to an absolute target using the
      // current measured position.
      controls_sensors_t snap = {0};
      sensors_snapshot(&snap);
      cmd.setpoint += snap.position_rad; // offset -> absolute rad.
      cmd.mode = CONTROL_MODE_POSITION;  // Treat as absolute from here on.
      control_command = cmd;             // Persist resolved absolute target.
    }

    if (cmd.mode != prev_mode) {
      pid_init(&velocity_pi); // Reset, gains uneffected by init.
      pid_init(&position_pd); // Reset, gains uneffected by init.
      prev_mode = cmd.mode;
    }

    step_rate_command = 0.0f; // Safety.

    // Reset position on target tick counter and latch variable.
    on_target_ticks = 0;
    latched_on_target = false;

    controls_state = CONTROLS_EXECUTING;
    break;

  case CONTROLS_EXECUTING:
    command_snapshot(&cmd);
    sensors_snapshot(&data);

    control_loop(cmd, data);
    if (on_position_target(cmd, data)) {
      // Transition state.
      controls_state = CONTROLS_ON_POSITION_TARGET;
    }
    break;

  case CONTROLS_ON_POSITION_TARGET:
    command_snapshot(&cmd);
    sensors_snapshot(&data);

    // Keep running the loop so the driver actively holds position.
    control_loop(cmd, data);

    // If there is drift outside the exit band, resume normal execution.
    if (!on_position_target(cmd, data)) {
      controls_state = CONTROLS_EXECUTING;
    }
    break;

  case CONTROLS_FAULT:
    step_rate_command = 0.0f; // Safety.
    break;
  default:
    break;
  }
}
