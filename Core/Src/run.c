/*******************************************************************************
 * @file run.c
 * @brief Centralized main loop run logic running in main.c.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "run.h"
#include "abi_encoder_hal_tim.h"
#include "as5047p_hal_spi.h"
#include "can.h"
#include "comm.h"
#include "configuration.h"
#include "controls.h"
#include "tmc2209_runner.h"
#include "uid_hash48.h"
#include "ws2812b_hal_pwm.h"

/** Private variables. ********************************************************/

// Sensor variables.
static abi_encoder_t abi_encoder = {0};

// Encoder position tracking (multi-turn unwrapped).
static float s_prev_raw_rad = 0.0f;
static float s_pos_unwrapped_rad = 0.0f;
static float s_prev_pos_rad = 0.0f;
static bool s_encoder_initialized = false;

// Step output: direction change inhibit state.
static bool s_last_dir = true;
static bool s_dir_inhibit = false;

/** Private functions. *********************************************************/

void can_tx_absolute_angle(void) {
  const can_message_t msg =
      mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_ABSOLUTE_ENCODER];
  uint32_t sigs[3] = {0};
  const float source_sigs[3] = {can_node_id, as5047p_angle_step,
                                0}; // TODO: Hardcoded status.
  for (int i = 0; i < msg.signal_count; ++i) {
    sigs[i] = physical_to_raw(source_sigs[i], &msg.signals[i]);
  }
  can_send_message_raw32(&HCAN, &msg, sigs);
}

void sensors_run(void) {
  // Absolute angle sensor.
  as5047p_get_angle();
#ifdef MC_STEPPER_FULL_CAN_TELEMETRY
  can_tx_absolute_angle();
#endif
}

/** Public functions. *********************************************************/

void mc_stepper_init(void) {
  // Low level peripherals.
  can_init();
  can_db_init();

  // UART debug/developer interface.
  comm_init();

  // On-board miscellaneous components.
  ws2812b_init();
  ws2812b_set_colour(0, 2, 1, 3); // Init colour.
  ws2812b_update();

  // Sensors.
#if MC_STEPPER_USE_ABSOLUTE_ENCODER
  as5047p_init(0x01, 0x00);
  HAL_Delay(1);
#endif
#if MC_STEPPER_USE_RELATIVE_ENCODER
  abi_encoder_init(&abi_encoder, &htim2);
#endif

  // Closed-loop controls. (Must be init before tmc2209_init()).
  controls_init();

  // CAN allocatee begin.
  const allocatee_config_t l_config = {
      can_tx_direct,
      get_uid_hash48_parts,
      allocatee_complete,
  };
  can_id_allocatee_start(l_config);

  // TMC2209. (Must be init after controls_init()).
  const tmc2209_configuration_t default_config = {false, 16, 500,     4,
                                                  12,    8,  20000.0f};
  tmc2209_init(default_config);
}

void controls_run(void) {
  // Update sensor data.
  sensors_run();

  // Convert angle to radians at the source.
  // as5047p_angle_step is the raw 14-bit count (0-16383).
  const float raw_rad =
      (float)as5047p_angle_step * (2.0f * (float)M_PI / 16384.0f);

  if (!s_encoder_initialized) {
    s_prev_raw_rad = raw_rad;
    s_pos_unwrapped_rad = raw_rad;
    s_prev_pos_rad = raw_rad;
    s_encoder_initialized = true;
  }

  // Wrap detection threshold is half a revolution in radians.
  float delta = -(raw_rad - s_prev_raw_rad);
  if (delta > (float)M_PI)
    delta -= 2.0f * (float)M_PI;
  if (delta < -(float)M_PI)
    delta += 2.0f * (float)M_PI;

  s_pos_unwrapped_rad += delta;
  s_prev_raw_rad = raw_rad;

  // Backward-difference velocity estimate. At 1 ms this is low-noise for a
  // 14-bit encoder; the position PD derivative filter handles residual noise.
  const float velocity_rad_s =
      (s_pos_unwrapped_rad - s_prev_pos_rad) / CONTROLS_PERIOD_S;
  s_prev_pos_rad = s_pos_unwrapped_rad;

  // Write sensor data for the controls loop, controls_sensors are volatile.
  controls_sensors.position_rad = s_pos_unwrapped_rad;
  controls_sensors.velocity_rad_s = velocity_rad_s;

  // Run the closed-loop control state machine.
  controls_state_machine();

  // Bridge: step_rate_command (signed float, steps/s) to hardware.
  // Direction changes require a one-tick inhibit so the DIR pin is settled
  // before the next step edge. The TMC2209 datasheet requires DIR to be stable
  // >= 20 ns before a rising STEP edge; stopping for one full 1 ms tick is far
  // more than sufficient and avoids a missed-step on reversal.
  const float sr = step_rate_command; // Snapshot the volatile once.
  const bool new_dir = (sr >= 0.0f);
  const uint32_t freq_hz = (uint32_t)fabsf(sr);

  if (new_dir != s_last_dir) {
    tmc2209_step_set_freq_hz(0); // Halt steps immediately.
    tmc2209_step_direction(new_dir);
    s_last_dir = new_dir;
    s_dir_inhibit = true; // Suppress output for one additional tick.
    return;
  }

  if (s_dir_inhibit) {
    s_dir_inhibit = false; // One dead tick elapsed; resume next call.
    return;
  }

  tmc2209_step_set_freq_hz(freq_hz);
}
