/*******************************************************************************
 * @file init.c
 * @brief Centralized init logic running in main.c.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "init.h"
#include "as5047p_hal_spi.h"
#include "can.h"
#include "comm.h"
#include "configuration.h"
#include "controls.h"
#include "mcu_temp_hal_adc.h"
#include "scheduler.h"
#include "tmc2209_runner.h"
#include "ws2812b_hal_pwm.h"
#include <math.h>

/** Definitions. **************************************************************/

// Counter triggered task (can_tx_state).
#define CAN_TX_STATE_TARGET_PERIOD_MS (1000u) // Schedule time desired.
#define CAN_TX_STATE_TASK_PERIOD_MS (25u)     // Schedule time utilized.
#define CAN_TX_STATE_TASK_DIVIDER                                              \
  (CAN_TX_STATE_TARGET_PERIOD_MS / CAN_TX_STATE_TASK_PERIOD_MS) // Trigger.

/** Private functions. ********************************************************/

/**
 * @brief Transmit state message.
 */
void can_tx_state(void) {
  tmc2209_status_poll();

  // Internal firmware fault byte.
  // bit 0 = controls latched into CONTROLS_FAULT.
  // TODO: Bits 1-7 reserved for future internal fault codes.
  const uint8_t internal_fault =
      (controls_state == CONTROLS_FAULT) ? 0x01u : 0x00u;

  const can_message_t *msg = &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_STATE];
  const uint32_t values[] = {
      physical_to_raw((float)controls_state, &msg->signals[0]),
      physical_to_raw(get_mcu_temp(), &msg->signals[1]),
      physical_to_raw(internal_fault, &msg->signals[2]),
      physical_to_raw(tmc2209_get_status(), &msg->signals[3]),
  };
  can_send_message_raw32(&HCAN, msg, values);
}

static void can_tx_state_trigger(void) {
  static uint16_t can_tx_state_tick_count = 0;

  if (++can_tx_state_tick_count < CAN_TX_STATE_TASK_DIVIDER) {
    return; // Not time yet, exit.
  }
  can_tx_state_tick_count = 0;

  can_tx_state();   // Trigger task.
  mcu_temp_start(); // Trigger next temperature read.
}

static void can_tx_sensor(void) {
  const can_message_t *msg = &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_SENSOR];
  const float raw_rad =
      (float)as5047p_angle_step * (2.0f * (float)M_PI / 16384.0f);
  const uint32_t values[] = {
      (uint32_t)as5047p_angle_step,
      physical_to_raw(raw_rad, &msg->signals[1]),
  };
  can_send_message_raw32(&HCAN, msg, values);
}

static void can_tx_controls_diagnostic(void) {
  const float pos_sp = controls_position_setpoint;
  const float pos_meas = controls_position_meas_rad;
  const float vel_sp = controls_velocity_setpoint;
  const float step_rate = controls_step_rate_command;

  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CONTROLS_DIAGNOSTIC];
  const uint32_t values[] = {
      physical_to_raw(pos_sp, &msg->signals[0]),
      physical_to_raw(pos_sp - pos_meas, &msg->signals[1]),
      physical_to_raw(vel_sp, &msg->signals[2]),
      physical_to_raw(step_rate, &msg->signals[3]),
  };
  can_send_message_raw32(&HCAN, msg, values);
}

static void can_tx_stallguard_event(void) {
  bool enable, stalled;
  uint8_t sgthrs;
  uint16_t sg_result;
  tmc2209_stallguard_get(&enable, &sgthrs, &sg_result, &stalled);

  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_STALLGUARD_EVENT];
  const uint32_t values[] = {
      physical_to_raw(can_node_id, &msg->signals[0]),
      physical_to_raw(sg_result, &msg->signals[1]),
      physical_to_raw(sgthrs, &msg->signals[2]),
      physical_to_raw(stalled, &msg->signals[3]),
  };
  can_send_message_raw32(&HCAN, msg, values);
}

/**
 * @brief Poll StallGuard; broadcast a stallguard_event on a stall edge.
 */
static void stallguard_run(void) {
  if (tmc2209_stallguard_poll()) {
    can_tx_stallguard_event();
  }
}

/**
 * @brief Drive the on-board WS2812B to reflect stepper control status.
 */
static void led_status_run(void) {
  // Map controls_state to a status colour. Updates the LED only on state
  // change to avoid hammering the WS2812B DMA path each tick. Dim values keep
  // the on-board LED visible without being blinding at room lighting.
  static controls_state_t last_state = CONTROLS_FAULT; // Force first update.
  const controls_state_t state = controls_state;

  if (state == last_state)
    return;
  last_state = state;

  // Open-loop position runs without encoder feedback, so its "moving" and
  // "holding" indications are kept distinct from the encoder-verified yellow/
  // green of closed-loop control.
  const bool open_loop =
      (controls_get_mode() == CONTROL_MODE_POSITION_OPEN_LOOP);

  switch (state) {
  case CONTROLS_START:
    break;
  case CONTROLS_EXECUTING:
    if (open_loop) {
      ws2812b_set_colour(0, 1, 4, 2); // Cyan: open-loop moving.
    } else {
      ws2812b_set_colour(0, 3, 3, 0); // Yellow: actively moving.
    }
    break;
  case CONTROLS_ON_POSITION_TARGET:
    if (open_loop) {
      ws2812b_set_colour(0, 0, 1, 4); // Blue: open-loop holding.
    } else {
      ws2812b_set_colour(0, 0, 5, 0); // Green: on target, holding.
    }
    break;
  case CONTROLS_FAULT:
    ws2812b_set_colour(0, 5, 0, 0); // Red: fault.
    break;
  case CONTROLS_IDLE:
  case CONTROLS_DISABLE:
  default:
    ws2812b_set_colour(0, 2, 1, 3); // Idle: dim init colour.
    break;
  }
  ws2812b_update();
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

  // Closed-loop controls. (Must be init before tmc2209_init()). Owns and
  // initializes the absolute encoder.
  controls_init();

  // CAN allocatee begin (configuration check internally).
  auto_can_id_allocatee_start();

  // MCU internal core temperature sense (ADC self-calibration), then kick the
  // first DMA conversion so a reading is cached before first task trigger.
  mcu_temp_init();
  mcu_temp_start();

  // TMC2209. (Must be init after controls_init()), start disabled.
  const tmc2209_configuration_t default_config = {
      false,
      MC_STEPPER_STEP_SIZE,
      MC_STEPPER_RUN_CURRENT_MA,
      MC_STEPPER_CURRENT_HOLD,
      MC_STEPPER_CURRENT_RUN,
      MC_STEPPER_CURRENT_HOLD_DELAY,
      MC_STEPPER_MAX_ACCELERATION_RAD_S2};
  tmc2209_init(default_config);

  // Scheduler.
  scheduler_init(); // Initialize scheduler.
  scheduler_add_task(controls_run, CONTROLS_PERIOD_MS);
  scheduler_add_task(can_tx_state_trigger, CAN_TX_STATE_TASK_PERIOD_MS);
  scheduler_add_task(can_id_allocatee_state_machine, 20);
#ifdef MC_STEPPER_FULL_CAN_TELEMETRY
  scheduler_add_task(can_tx_sensor, 10);
  scheduler_add_task(can_tx_controls_diagnostic, 20);
#endif
  scheduler_add_task(can_stepper_reconfig_run, 20);
  scheduler_add_task(stallguard_run, 10);
  scheduler_add_task(led_status_run, 25);
}
