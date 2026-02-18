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

static osMutexId_t g_setpoint_mutex = NULL;
static osMutexId_t g_sensors_mutex = NULL;

static mc_stepper_command_t control_command = {0}; // Command input.

static control_mode_t prev_mode = CONTROL_MODE_NONE; // Previous control mode.

// Controls inputs and outputs.
static controls_sensors_t cache_sensor_data = {0};
static mc_stepper_command_t cache_command = {CONTROL_MODE_NONE, 0.0f};
static float position_setpoint = 0; // Set as rad.
static float velocity_setpoint = 0; // Set as rad/s.

static pid_controller_t position_pd = {0}; // Outputs velocity (rad/s).
static pid_controller_t velocity_pi = {0}; // Outputs step-rate (steps/s).

// Stepper motor configuration.
static uint16_t full_steps_per_rev = 200;
static uint16_t microstep_config = 16;
static float steps_per_rad;

static uint16_t on_target_ticks = 0;   // Position on-target latch count.
static bool latched_on_target = false; // Position on-target latching check.

/** Private mutex functions. **************************************************/

static bool command_snapshot(mc_stepper_command_t *command) {
  if (g_setpoint_mutex && osMutexAcquire(g_setpoint_mutex, 1) != osOK) {
    return false;
  }
  *command = control_command;
  if (g_setpoint_mutex)
    osMutexRelease(g_setpoint_mutex);
  return true;
}

static bool sensors_snapshot(controls_sensors_t *out) {
  if (g_sensors_mutex && osMutexAcquire(g_sensors_mutex, 1) != osOK) {
    return false;
  }
  *out = controls_sensors;
  if (g_sensors_mutex)
    osMutexRelease(g_sensors_mutex);
  return true;
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
      MAX_STEP_ACCEL_STEPS_S2 * CONTROLS_PERIOD_S; // steps/s per tick.
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

void controls_set_mutexes(osMutexId_t setpoint_mutex,
                          osMutexId_t sensors_mutex) {
  g_setpoint_mutex = setpoint_mutex;
  g_sensors_mutex = sensors_mutex;
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
  if (g_setpoint_mutex) {
    osMutexAcquire(g_setpoint_mutex, osWaitForever);
  }

  // TODO: Implement command override policy.
  control_command = command;

  if (g_setpoint_mutex) {
    osMutexRelease(g_setpoint_mutex);
  }

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
    if (command_snapshot(&cmd)) {
      cache_command = cmd;

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
    } else {
      controls_state = CONTROLS_IDLE;
    }
    break;

  case CONTROLS_EXECUTING:
    if (command_snapshot(&cmd))
      cache_command = cmd;
    else
      cmd = cache_command;

    if (sensors_snapshot(&data))
      cache_sensor_data = data;
    else
      data = cache_sensor_data;

    control_loop(cmd, data);
    if (on_position_target(cmd, data)) {
      // Transition state.
      controls_state = CONTROLS_ON_POSITION_TARGET;
    }
    break;

  case CONTROLS_ON_POSITION_TARGET:
    // TODO: TEST! Transition condition from ON_POSITION_TARGET to IDLE.
    controls_state = CONTROLS_IDLE;
    break;

  case CONTROLS_FAULT:
    step_rate_command = 0.0f; // Safety.
    break;
  default:
    break;
  }
}
