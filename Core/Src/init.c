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
#include "telemetry.h"
#include "tmc2209_runner.h"
#include "ws2812b_hal_pwm.h"

/** Definitions. **************************************************************/

// Counter triggered task (can_tx_state).
#define CAN_TX_STATE_TARGET_PERIOD_MS (1000u) // Schedule time desired.
#define CAN_TX_STATE_TASK_PERIOD_MS (25u)     // Schedule time utilized.
#define CAN_TX_STATE_TASK_DIVIDER                                              \
  (CAN_TX_STATE_TARGET_PERIOD_MS / CAN_TX_STATE_TASK_PERIOD_MS) // Trigger.

// Counter triggered task (comm_tx_status).
#define COMM_TX_STATUS_TARGET_PERIOD_MS (100u) // Schedule time desired.
#define COMM_TX_STATUS_TASK_PERIOD_MS (25u)    // Schedule time utilized.
#define COMM_TX_STATUS_TASK_DIVIDER                                            \
  (COMM_TX_STATUS_TARGET_PERIOD_MS / COMM_TX_STATUS_TASK_PERIOD_MS) // Trigger.

/** Private functions. ********************************************************/

static void can_tx_state_trigger(void) {
  static uint16_t can_tx_state_tick_count = 0;

  if (++can_tx_state_tick_count < CAN_TX_STATE_TASK_DIVIDER) {
    return; // Not time yet, exit.
  }
  can_tx_state_tick_count = 0;

  can_tx_state();   // Trigger task.
  mcu_temp_start(); // Trigger next temperature read.
}

static void comm_tx_status_trigger(void) {
  static uint16_t comm_tx_status_tick_count = 0;

  if (++comm_tx_status_tick_count < COMM_TX_STATUS_TASK_DIVIDER) {
    return; // Not time yet, exit.
  }
  comm_tx_status_tick_count = 0;

  comm_tx_status(); // Trigger task.
}

/**
 * @brief Poll StallGuard; broadcast a stallguard_event on a stall edge.
 */
static void stallguard_run(void) {
  if (tmc2209_stallguard_poll()) {
    can_tx_stallguard_event();
  }
}

#ifndef LED_EXTERNAL_CONTROL_ONLY
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
#endif

/** Public functions. *********************************************************/

void mc_stepper_init(void) {
  // Low level peripherals.
  can_init();
  can_db_init();

  // UART debug/developer interface.
  comm_init();

  // On-board miscellaneous components.
  ws2812b_init();
#ifndef LED_EXTERNAL_CONTROL_ONLY
  ws2812b_set_colour(0, 2, 1, 3); // Init colour.
  ws2812b_update();
#endif

  // Closed-loop controls. (Must be init before tmc2209_init()). Owns and
  // initializes the absolute encoder.
  controls_init();

  // CAN allocatee begin (advertises the ALLOW_CAN_NODE_ID_ALLOCATION mode).
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
#ifdef MC_STEPPER_FULL_COMM_TELEMETRY
  scheduler_add_task(comm_tx_status_trigger, COMM_TX_STATUS_TASK_PERIOD_MS);
#endif
  scheduler_add_task(can_stepper_reconfig_run, 20);
  scheduler_add_task(comm_run, 20);
  scheduler_add_task(stallguard_run, 10);
#ifndef LED_EXTERNAL_CONTROL_ONLY
  scheduler_add_task(led_status_run, 25);
#endif
}
