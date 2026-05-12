/*******************************************************************************
 * @file controls.c
 * @brief Low level closed-loop controls manager.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "controls.h"
#include "pid.h"
#include "stm32l4xx.h"
#include <math.h>

/** Public variables. *********************************************************/

volatile controls_state_t controls_state = CONTROLS_DISABLE;

volatile float controls_position_meas_rad = 0;
volatile float controls_position_setpoint = 0;
volatile float controls_velocity_setpoint = 0;
volatile float controls_step_rate_command = 0;

/** Private variables. ********************************************************/

static mc_stepper_command_t control_command = {0}; // Command input.

static control_mode_t prev_mode = CONTROL_MODE_NONE; // Previous control mode.

static pid_controller_t position_pid = {0}; // Outputs velocity (rad/s).

// Stepper motor configuration.
static uint16_t full_steps_per_rev = 200;
static uint16_t microstep_config = 16;
static float steps_per_rad;

// Runtime acceleration limit (steps/s^2). Defaults to the compile-time value.
// Overridden via controls_set_accel_rad_s2() once steps_per_rad is known.
static float s_max_step_accel = MAX_STEP_ACCEL_STEPS_S2;
static float s_accel_rad_s2 = 0.0f; // Cached rad/s^2 for microstep rescale.

static uint16_t on_target_ticks = 0;   // Position on-target latch count.
static bool latched_on_target = false; // Position on-target latching check.

/** Private snapshot functions. ***********************************************/

static void command_snapshot(mc_stepper_command_t *command) {
  __disable_irq();
  *command = control_command;
  __enable_irq();
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
      clampf(target - controls_step_rate_command, -max_delta, +max_delta);
  return controls_step_rate_command + delta;
}

static void control_loop(const mc_stepper_command_t command) {
  float step_rate_target;

  switch (command.mode) {

  case CONTROL_MODE_SPEED:
    // Open-loop: map speed setpoint directly to step rate.
    controls_velocity_setpoint = command.setpoint;
    step_rate_target = controls_velocity_setpoint * steps_per_rad;
    break;

  case CONTROL_MODE_POSITION:
    controls_position_setpoint = command.setpoint;
    controls_velocity_setpoint = pid_update(
        &position_pid, controls_position_setpoint, controls_position_meas_rad);
    step_rate_target = controls_velocity_setpoint * steps_per_rad;
    break;

  default:
    step_rate_target = 0.0f;
    break;
  }

  // Apply step-rate acceleration/slew limiting (trapezoid-like behavior).
  controls_step_rate_command = step_rate_slew_limit(step_rate_target);
}

static bool on_position_target(const mc_stepper_command_t command) {
  // Only meaningful for position moves.
  if (command.mode != CONTROL_MODE_POSITION)
    return false;

  // Choose hysteresis thresholds depending on current latched state.
  const float pos_tol = latched_on_target ? POSITION_TOLERANCE_EXIT_RAD
                                          : POSITION_TOLERANCE_ENTER_RAD;

  const float pos_err = command.setpoint - controls_position_meas_rad;

  const bool in_band = (fabsf(pos_err) <= pos_tol);

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
  s_accel_rad_s2 = accel_rad_s2;
  s_max_step_accel = accel_rad_s2 * steps_per_rad;
}

void controls_init(void) {
  // Calculate step size config.
  steps_per_rad = ((float)full_steps_per_rev * (float)microstep_config) /
                  (2.0f * (float)M_PI);

  // Position.
  pid_init(&position_pid);
  position_pid.k_p = 3.5f;
  position_pid.k_i = 0.0f;
  position_pid.integral_min = -2.5f;
  position_pid.integral_max = 2.5f;
  position_pid.k_d = 0.5f;
  position_pid.tau = 0.02f;
  position_pid.output_min = -5.0f;
  position_pid.output_max = 5.0f;
  position_pid.T = CONTROLS_PERIOD_S;

  // Reset position on target tick counter and latch variable.
  on_target_ticks = 0;
  latched_on_target = false;

  // Bring the FSM out of DISABLE so standstill-gated APIs (e.g. set_microsteps)
  // can run after init.
  controls_state = CONTROLS_IDLE;
}

bool controls_set_microsteps(const uint16_t microsteps) {
  // TODO: Improve to allow adjustment of microstepping at standstill.
  if (controls_state == CONTROLS_IDLE) {
    microstep_config = microsteps;
    steps_per_rad = ((float)full_steps_per_rev * (float)microstep_config) /
                    (2.0f * (float)M_PI);
    // Re-derive the steps/s^2 limit so the rad/s^2 envelope is preserved.
    if (s_accel_rad_s2 > 0.0f) {
      s_max_step_accel = s_accel_rad_s2 * steps_per_rad;
    }
    return true;
  }
  return false;
}

void controls_redatum_shift(const float delta_rad) {
  __disable_irq();
  controls_position_setpoint += delta_rad;
  if (control_command.mode == CONTROL_MODE_POSITION) {
    control_command.setpoint += delta_rad;
  }
  // Shift prev_measurement by the same delta so the derivative path sees no
  // discontinuity. pid_init() would zero prev_measurement and cause a kick on
  // the next update.
  position_pid.prev_measurement += delta_rad;
  __enable_irq();
}

void controls_set_gains(const float k_p, const float k_i, const float k_d) {
  __disable_irq();
  position_pid.k_p = k_p;
  position_pid.k_i = k_i;
  position_pid.k_d = k_d;
  __enable_irq();
}

void controls_get_gains(float *k_p, float *k_i, float *k_d) {
  __disable_irq();
  const float p = position_pid.k_p;
  const float i = position_pid.k_i;
  const float d = position_pid.k_d;
  __enable_irq();
  if (k_p)
    *k_p = p;
  if (k_i)
    *k_i = i;
  if (k_d)
    *k_d = d;
}

void controls_set_setpoint(const mc_stepper_command_t command) {
  // Gate on change: if the incoming command matches the active one, do not
  // disturb the state machine. Repeated retransmissions of the same target
  // (commander heartbeat, enable-hold, periodic resend) would otherwise kick
  // the state back to CONTROLS_START every frame, resetting the on-target dwell
  // counter and preventing the ON_POSITION_TARGET latch from ever closing.
  //
  // Note: POSITION_RELATIVE is resolved to POSITION inside CONTROLS_START, so
  // a repeated identical POSITION_RELATIVE frame will still mismatch here (mode
  // POSITION_RELATIVE vs stored POSITION) and re-trigger a fresh move, which
  // matches the one-shot semantics of a relative command.
  if (command.mode == control_command.mode &&
      fabsf(command.setpoint - control_command.setpoint) <=
          SETPOINT_EPSILON_RAD) {
    return;
  }

  __disable_irq();
  control_command = command;
  controls_state = CONTROLS_START; // Transition state.
  __enable_irq();
}

void controls_state_machine(void) {
  mc_stepper_command_t cmd = {0};

  switch (controls_state) {
  case CONTROLS_DISABLE:
    controls_step_rate_command = 0.0f; // Safety.
    break;

  case CONTROLS_IDLE:
    // Wait for new command.
    break;

  case CONTROLS_START:
    command_snapshot(&cmd);

    // Relative position control.
    if (cmd.mode == CONTROL_MODE_POSITION_RELATIVE) {
      cmd.setpoint += controls_position_meas_rad; // offset -> absolute rad.
      cmd.mode = CONTROL_MODE_POSITION; // Treat as absolute from here on.
      control_command = cmd;            // Persist resolved absolute target.
    }

    if (cmd.mode != prev_mode) {
      pid_init(&position_pid); // Reset, gains uneffected by init.
      // Seed prev_measurement so the first derivative tick does not see a
      // (measurement - 0) jump and saturate the output.
      position_pid.prev_measurement = controls_position_meas_rad;
      prev_mode = cmd.mode;
    }

    controls_step_rate_command = 0.0f; // Safety.

    // Reset position on target tick counter and latch variable.
    on_target_ticks = 0;
    latched_on_target = false;

    controls_state = CONTROLS_EXECUTING;
    break;

  case CONTROLS_EXECUTING:
    command_snapshot(&cmd);

    control_loop(cmd);
    if (on_position_target(cmd)) {
      // Transition state.
      controls_state = CONTROLS_ON_POSITION_TARGET;
    }
    break;

  case CONTROLS_ON_POSITION_TARGET:
    command_snapshot(&cmd);

    // Keep running the loop so the driver actively holds position.
    control_loop(cmd);

    // If there is drift outside the exit band, resume normal execution.
    if (!on_position_target(cmd)) {
      controls_state = CONTROLS_EXECUTING;
    }
    break;

  case CONTROLS_FAULT:
    controls_step_rate_command = 0.0f; // Safety.
    break;
  default:
    break;
  }
}
