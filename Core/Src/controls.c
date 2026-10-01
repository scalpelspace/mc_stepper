/*******************************************************************************
 * @file controls.c
 * @brief Low level closed-loop controls manager.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "controls.h"
#include "as5047p_hal_spi.h"
#include "configuration.h"
#include "pid.h"
#include "stm32l4xx.h"
#include "tmc2209_runner.h"
#include <math.h>

/** Private variables. ********************************************************/

static mc_stepper_command_t control_command = {0}; // Command input.

static control_mode_t prev_mode = CONTROL_MODE_NONE; // Previous control mode.

static pid_controller_t position_pid = {0}; // Outputs velocity (rad/s).

// Stepper motor configuration.
static uint16_t full_steps_per_rev = 200;
static uint16_t microstep_config = 16;
static float steps_per_rad;

// Runtime acceleration limit. Seeded from the configured default (rad/s^2),
// shared with the TMC2209 config, and converted to the steps/s^2 slew cap in
// controls_init() once steps_per_rad is known.
static float s_max_accel_rad_s2 = MC_STEPPER_MAX_ACCELERATION_RAD_S2;
static float s_max_accel_step = 0.0f; // steps/s^2 slew cap, set in init.

static uint16_t on_target_ticks = 0;   // Position on-target latch count.
static bool latched_on_target = false; // Position on-target latching check.

// Position on-target hysteresis band (rad), re-derived from the microstep
// configuration by position_tolerance_update(). Seeded with the encoder floor
// so the values are sane before controls_init() runs.
static float s_pos_tol_enter_rad = POSITION_TOLERANCE_FLOOR_RAD;
static float s_pos_tol_exit_rad =
    POSITION_TOLERANCE_FLOOR_RAD * POSITION_TOLERANCE_EXIT_RATIO;

// Trapezoidal motion profile state. The profile plans a feasible position and
// velocity trajectory toward the setpoint that respects s_max_accel_rad_s2 and
// s_max_velocity_rad_s. PID then corrects deviation between profile and
// measured position, so the loop never has to bang-bang against motor limits.
static float s_max_velocity_rad_s = MC_STEPPER_MAX_VELOCITY_RAD_S;
static float s_profile_pos = 0.0f; // Planned position (rad).
static float s_profile_vel = 0.0f; // Planned velocity (rad/s).

// True open-loop state. The commanded position lives in an internal,
// encoder-independent frame measured in microsteps (starts at 0). Each move
// emits an exact microstep budget (driver-counted). s_ol_step_vel is the
// step-domain rate ramp and s_ol_dir the fixed move direction.
static int32_t s_ol_pos_steps = 0; // Open-loop commanded position (microsteps).
static bool s_ol_dir = true;       // Move direction (true = forward).
static float s_ol_step_vel = 0.0f; // Step-domain rate (steps/s).

// Encoder position tracking (multi-turn unwrapped).
static float s_prev_raw_rad = 0.0f;
static float s_pos_unwrapped_rad = 0.0f;
static bool s_encoder_initialized = false;

// Step output: direction change inhibit state. s_last_dir_valid forces the
// first bridge pass to assert DIR explicitly, since the TMC2209 DIR pin powers
// up in an undefined state that may not match s_last_dir's default.
static bool s_last_dir = true;
static bool s_last_dir_valid = false;
static bool s_dir_inhibit = false;

// Software zero (re-datum) request. Set from any context (incl. CAN ISR),
// serviced at the top of the next controls tick when state is at standstill.
static volatile bool s_redatum_pending = false;

/** Public variables. *********************************************************/

volatile controls_state_t controls_state = CONTROLS_DISABLE;

volatile float controls_position_meas_rad = 0;
volatile float controls_position_setpoint = 0;
volatile float controls_velocity_setpoint = 0;
volatile float controls_step_rate_command = 0;

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
      s_max_accel_step * CONTROLS_PERIOD_S; // steps/s per tick.
  const float delta =
      clampf(target - controls_step_rate_command, -max_delta, +max_delta);
  return controls_step_rate_command + delta;
}

/**
 * @brief Re-derive the position on-target band from the microstep setting.
 *
 * Takes the larger of the encoder noise floor and the step-grid term, see the
 * tolerance notes in controls.h. Call whenever steps_per_rad changes, the band
 * is meaningless against a stale calibration.
 */
static void position_tolerance_update(void) {
  // steps_per_rad is microsteps per radian, so its reciprocal is the angular
  // size of one microstep. Guarded against a zero calibration, which would
  // otherwise divide by zero before controls_init() sets it.
  const float microstep_rad =
      (steps_per_rad > 0.0f) ? (1.0f / steps_per_rad) : 0.0f;
  const float grid_tol = POSITION_TOLERANCE_MICROSTEPS * microstep_rad;

  s_pos_tol_enter_rad = (grid_tol > POSITION_TOLERANCE_FLOOR_RAD)
                            ? grid_tol
                            : POSITION_TOLERANCE_FLOOR_RAD;
  s_pos_tol_exit_rad = s_pos_tol_enter_rad * POSITION_TOLERANCE_EXIT_RATIO;
}

/**
 * @brief Reset the motion profile to the current measured position at rest.
 *
 * Call on each new move (CONTROLS_START) so the planned trajectory begins from
 * where the rotor actually is.
 */
static void profile_reset(void) {
  s_profile_pos = controls_position_meas_rad;
  s_profile_vel = 0.0f;
}

/**
 * @brief Advance the trapezoidal profile one tick toward the given target.
 *
 * Generates the next planned (position, velocity) point respecting the
 * acceleration and peak velocity limits. Uses the standard brake-distance test
 * (v^2/2a) to decide acceleration/cruise vs deceleration.
 *
 * @param target Target position (rad).
 */
static void profile_step(const float target) {
  const float accel = (s_max_accel_rad_s2 > 0.0f) ? s_max_accel_rad_s2 : 100.0f;
  const float dt = CONTROLS_PERIOD_S;
  const float remaining = target - s_profile_pos;
  const float v = s_profile_vel;

  const float brake_dist = (v * v) / (2.0f * accel);

  float desired_vel;
  if (fabsf(remaining) <= brake_dist) {
    // Decelerate, aiming to arrive at target with v=0.
    desired_vel = copysignf(sqrtf(2.0f * accel * fabsf(remaining)), remaining);
  } else {
    // Accelerate/cruise toward max velocity in the direction of remaining.
    desired_vel = copysignf(s_max_velocity_rad_s, remaining);
  }

  // Limit per-tick velocity change to the acceleration cap.
  const float max_dv = accel * dt;
  const float dv = clampf(desired_vel - v, -max_dv, +max_dv);
  s_profile_vel = clampf(v + dv, -s_max_velocity_rad_s, +s_max_velocity_rad_s);
  s_profile_pos += s_profile_vel * dt;
}

/**
 * @brief Shift the controls-layer position frame by delta_rad.
 *
 * Adds delta_rad to the active position setpoint (and to the pending command
 * setpoint when in POSITION mode), and to the motion profile and PID derivative
 * state, so the frame change introduces no error transient or derivative kick.
 *
 * The caller pairs this with a matching shift of the sensor-side unwrapped
 * position so the physical (absolute) target is preserved.
 *
 * @param delta_rad Position delta in radians.
 */
static void controls_redatum_shift(const float delta_rad) {
  __disable_irq();
  controls_position_setpoint += delta_rad;
  if (control_command.mode == CONTROL_MODE_POSITION) {
    control_command.setpoint += delta_rad;
  }
  // Keep the profile and PID derivative path in the new frame. Otherwise, the
  // next tick would see the planned position and prev_measurement in the old
  // frame and the measurement in the new one, kicking both terms.
  s_profile_pos += delta_rad;
  position_pid.prev_measurement += delta_rad;
  __enable_irq();
}

static bool on_position_target(const mc_stepper_command_t command) {
  // Open-loop: the move is complete once the exact microstep budget has been
  // emitted by the driver. No encoder is consulted.
  if (command.mode == CONTROL_MODE_POSITION_OPEN_LOOP) {
    return (tmc2209_step_remaining() == 0u);
  }

  // Only meaningful for position moves.
  if (command.mode != CONTROL_MODE_POSITION)
    return false;

  // Choose hysteresis thresholds depending on current latched state.
  const float pos_tol =
      latched_on_target ? s_pos_tol_exit_rad : s_pos_tol_enter_rad;

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

static void control_loop(const mc_stepper_command_t command) {
  float step_rate_target;

  switch (command.mode) {

  case CONTROL_MODE_SPEED:
    // Open-loop: map speed setpoint directly to step rate.
    controls_velocity_setpoint = command.setpoint;
    step_rate_target = controls_velocity_setpoint * steps_per_rad;
    break;

  case CONTROL_MODE_POSITION_OPEN_LOOP: {
    // True open-loop: the exact microstep budget was loaded in CONTROLS_START
    // and is counted down by the driver. Here we only shape the rate with a
    // step-domain trapezoid. Decelerate so the rate reaches ~0 as the remaining
    // count does, while the driver guarantees the final count regardless of
    // loop timing.
    controls_position_setpoint = command.setpoint;
    const uint32_t remaining = tmc2209_step_remaining();
    if (remaining == 0u) {
      s_ol_step_vel = 0.0f;
      controls_velocity_setpoint = 0.0f;
      controls_step_rate_command = 0.0f;
      return;
    }
    const float a = (s_max_accel_step > 0.0f) ? s_max_accel_step : 1000.0f;
    const float v_max = s_max_velocity_rad_s * steps_per_rad;
    const float brake = (s_ol_step_vel * s_ol_step_vel) / (2.0f * a);
    const float target_v = ((float)remaining <= brake)
                               ? sqrtf(2.0f * a * (float)remaining)
                               : v_max;
    const float max_dv = a * CONTROLS_PERIOD_S;
    const float dv = clampf(target_v - s_ol_step_vel, -max_dv, +max_dv);
    s_ol_step_vel = clampf(s_ol_step_vel + dv, 0.0f, v_max);
    const float signed_rate = s_ol_dir ? s_ol_step_vel : -s_ol_step_vel;
    controls_velocity_setpoint = signed_rate / steps_per_rad; // rad/s (diag).
    controls_step_rate_command = signed_rate;                 // steps/s output.
    return; // Bypass the slew limiter; the ramp above already bounds accel.
  }

  case CONTROL_MODE_POSITION:
    controls_position_setpoint = command.setpoint;
    // Plan the next profile point toward the setpoint.
    profile_step(controls_position_setpoint);
    // PID corrects deviation between planned and measured position. With a
    // healthy stepper this term stays small handling step loss, encoder drift,
    // and load disturbance.
    const float correction =
        pid_update(&position_pid, s_profile_pos, controls_position_meas_rad);
    controls_velocity_setpoint =
        clampf(s_profile_vel + correction, -s_max_velocity_rad_s,
               +s_max_velocity_rad_s);
    step_rate_target = controls_velocity_setpoint * steps_per_rad;
    break;

  default:
    step_rate_target = 0.0f;
    break;
  }

  // Apply step-rate acceleration/slew limiting (trapezoid-like behavior).
  controls_step_rate_command = step_rate_slew_limit(step_rate_target);
}

static void controls_state_machine(void) {
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

    if (cmd.mode == CONTROL_MODE_POSITION_OPEN_LOOP) {
      // True open-loop: plan an exact microstep budget toward the target in the
      // internal (encoder-independent) frame. Account for any steps still
      // pending from a preempted move so the new budget starts from the actual
      // commanded position. The driver emits exactly this many steps.
      const uint32_t pending = tmc2209_step_remaining();
      const int32_t current_steps =
          s_ol_pos_steps - (s_ol_dir ? (int32_t)pending : -(int32_t)pending);
      const int32_t target_steps =
          (int32_t)lroundf(cmd.setpoint * steps_per_rad);
      const int32_t delta = target_steps - current_steps;
      s_ol_dir = (delta >= 0);
      tmc2209_step_load_count((uint32_t)(delta >= 0 ? delta : -delta));
      s_ol_pos_steps = target_steps;
      s_ol_step_vel = 0.0f;
      prev_mode = cmd.mode;
    } else {
      // Speed/closed-loop position are rate-controlled and run unbounded.
      tmc2209_step_clear_count();

      if (cmd.mode != prev_mode) {
        pid_init(&position_pid); // Reset, gains uneffected by init.
        // Seed prev_measurement so the first derivative tick does not see a
        // (measurement - 0) jump and saturate the output.
        position_pid.prev_measurement = controls_position_meas_rad;
        prev_mode = cmd.mode;
      }

      // Reset the trapezoidal profile to begin from the current measured
      // position at rest. The new move plans from here.
      profile_reset();
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

    // Stop emitting steps. The TMC2209 holds position via IHOLD current, and
    // with no STEP edges the driver also relaxes to hold current after
    // IHOLDDELAY. Re-engaging the loop on drift relies on the on-target
    // hysteresis exit band.
    controls_step_rate_command = 0.0f;
    controls_velocity_setpoint = 0.0f;

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

/** Public functions. *********************************************************/

void controls_init(void) {
  // Sensors.
  as5047p_init(0x01, 0x00);
  HAL_Delay(1);

  // Calculate step size config.
  steps_per_rad = ((float)full_steps_per_rev * (float)microstep_config) /
                  (2.0f * (float)M_PI);

  // Seed the steps/s^2 slew cap from the configured rad/s^2 default now that
  // steps_per_rad is known. Delegates to keep the rad/s^2 -> steps/s^2
  // conversion in a single place.
  controls_set_max_accel_rad_s2(s_max_accel_rad_s2);

  // Size the on-target band against the same calibration.
  position_tolerance_update();

  // Position.
  pid_init(&position_pid);
  position_pid.k_p = 50.0f;
  position_pid.k_i = 3.0f;
  position_pid.integral_min = -3.0f;
  position_pid.integral_max = 3.0f;
  position_pid.k_d = 17.0f;
  position_pid.tau = 0.02f;
  position_pid.output_min = -100.0f;
  position_pid.output_max = 100.0f;
  position_pid.T = CONTROLS_PERIOD_S;

  // Reset position on target tick counter and latch variable.
  on_target_ticks = 0;
  latched_on_target = false;

  // Reset the internal open-loop frame.
  s_ol_pos_steps = 0;
  s_ol_step_vel = 0.0f;
  s_ol_dir = true;

  // Bring the FSM out of DISABLE so standstill-gated APIs (e.g. set_microsteps)
  // can run after init.
  controls_state = CONTROLS_IDLE;
}

void controls_run(void) {
  // Update sensor data.
  as5047p_get_angle();

  // Convert angle to radians at the source.
  // as5047p_angle_step is the raw 14-bit count (0-16383).
  const float raw_rad =
      (float)as5047p_angle_step * (2.0f * (float)M_PI / 16384.0f);

  if (!s_encoder_initialized) {
    s_prev_raw_rad = raw_rad;
    s_pos_unwrapped_rad = raw_rad;
    s_encoder_initialized = true;
  }

  // Wrap detection threshold is half a revolution in radians.
  float delta = raw_rad - s_prev_raw_rad;
  if (delta > (float)M_PI)
    delta -= 2.0f * (float)M_PI;
  if (delta < -(float)M_PI)
    delta += 2.0f * (float)M_PI;

  s_pos_unwrapped_rad += delta;
  s_prev_raw_rad = raw_rad;

  // Service pending software-zero request. Only apply at standstill so the
  // setpoint frame is stable while the loop is actively commanding steps.
  if (s_redatum_pending) {
    if (controls_state == CONTROLS_DISABLE || controls_state == CONTROLS_IDLE ||
        controls_state == CONTROLS_ON_POSITION_TARGET) {
      const float shift = -s_pos_unwrapped_rad;
      s_pos_unwrapped_rad = 0.0f;
      controls_redatum_shift(shift);
    }
    s_redatum_pending = false;
  }

  // Write sensor data for the controls loop, controls_sensors are volatile.
  controls_position_meas_rad = s_pos_unwrapped_rad;

  // Run the closed-loop control state machine.
  controls_state_machine();

  // Bridge: controls_step_rate_command (signed float, steps/s) to hardware.
  // Direction changes require a one-tick inhibit so the DIR pin is settled
  // before the next step edge. The TMC2209 datasheet requires DIR to be stable
  // >= 20 ns before a rising STEP edge; stopping for one full 1 ms tick is far
  // more than sufficient and avoids a missed-step on reversal.
  const float sr = controls_step_rate_command; // Snapshot the volatile once.
  const bool new_dir = (sr >= 0.0f);
  const uint32_t freq_hz = (uint32_t)fabsf(sr);

  if (new_dir != s_last_dir || !s_last_dir_valid) {
    tmc2209_step_set_freq_hz(0); // Halt steps immediately.
    tmc2209_step_direction(new_dir);
    s_last_dir = new_dir;
    s_last_dir_valid = true;
    s_dir_inhibit = true; // Suppress output for one additional tick.
    return;
  }

  if (s_dir_inhibit) {
    s_dir_inhibit = false; // One dead tick elapsed; resume next call.
    return;
  }

  tmc2209_step_set_freq_hz(freq_hz);
}

void controls_set_command(const mc_stepper_command_t command) {
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

void controls_request_zero(void) { s_redatum_pending = true; }

bool controls_set_microsteps(const uint16_t microsteps) {
  // Only adjust microstepping at standstill (IDLE or DISABLE), never mid-move.
  if (controls_state == CONTROLS_IDLE || controls_state == CONTROLS_DISABLE) {
    const float old_steps_per_rad = steps_per_rad;
    microstep_config = microsteps;
    steps_per_rad = ((float)full_steps_per_rev * (float)microstep_config) /
                    (2.0f * (float)M_PI);
    // Re-derive the steps/s^2 limit so the rad/s^2 envelope is preserved.
    if (s_max_accel_rad_s2 > 0.0f) {
      s_max_accel_step = s_max_accel_rad_s2 * steps_per_rad;
    }
    // Re-derive the on-target band, a coarser grid needs a wider one.
    position_tolerance_update();
    // Rescale the internal open-loop position so it keeps the same rad value
    // across a microstep change.
    if (old_steps_per_rad > 0.0f) {
      s_ol_pos_steps = (int32_t)lroundf((float)s_ol_pos_steps *
                                        (steps_per_rad / old_steps_per_rad));
    }
    return true;
  }
  return false;
}

uint16_t controls_get_microsteps(void) { return microstep_config; }

void controls_set_max_accel_rad_s2(const float max_accel_rad_s2) {
  // Convert rad/s^2 to steps/s^2 using the current steps_per_rad calibration.
  // steps_per_rad is valid after controls_init() or controls_set_microsteps().
  s_max_accel_rad_s2 = max_accel_rad_s2;
  s_max_accel_step = max_accel_rad_s2 * steps_per_rad;
}

float controls_get_max_accel_rad_s2(void) { return s_max_accel_rad_s2; }

void controls_set_max_velocity_rad_s(const float max_velocity_rad_s) {
  if (max_velocity_rad_s > 0.0f) {
    s_max_velocity_rad_s = max_velocity_rad_s;
  }
}

float controls_get_max_velocity_rad_s(void) { return s_max_velocity_rad_s; }

void controls_get_position_tolerance(float *enter_rad, float *exit_rad) {
  if (enter_rad) {
    *enter_rad = s_pos_tol_enter_rad;
  }
  if (exit_rad) {
    *exit_rad = s_pos_tol_exit_rad;
  }
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

control_mode_t controls_get_mode(void) {
  __disable_irq();
  const control_mode_t mode = control_command.mode;
  __enable_irq();
  return mode;
}
