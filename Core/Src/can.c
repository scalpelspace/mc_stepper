/*******************************************************************************
 * @file can.c
 * @brief General STM32L432KC driver abstracting STM32 HAL: CAN.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "can.h"
#include "configuration.h"
#include "controls.h"
#include "rtc.h"
#include "tmc2209_runner.h"
#include "uid_hash48.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Private variables. ********************************************************/

// Transmit variables.
static CAN_TxHeaderTypeDef tx_header;
static uint32_t tx_mailbox;

// Deferred stepper reconfiguration. CAN RX handlers run in interrupt context,
// but the TMC2209 register updates perform blocking UART transfers. Requests
// are captured here and applied from the main context via
// can_stepper_reconfig_run().
static volatile bool currents_pending = false;
static uint16_t currents_run_ma;    // VREF DAC target current (mA).
static uint8_t currents_run;        // IRUN scaler [0,31].
static uint8_t currents_hold;       // IHOLD scaler [0,31].
static uint8_t currents_hold_delay; // IHOLDDELAY [0,15].

static volatile bool motion_pending = false;
static uint16_t motion_step_size;       // Microsteps [1,256].
static float motion_max_accel_rad_s2;   // rad/s^2.
static float motion_max_velocity_rad_s; // rad/s.

static volatile bool stallguard_pending = false;
static bool stallguard_enable;    // StallGuard detection enabled.
static uint8_t stallguard_sgthrs; // SGTHRS threshold [0,255].

/** Public variables. *********************************************************/

// Mutable runtime copy.
can_message_t mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_COUNT];
can_node_id_t can_node_id = DEFAULT_CAN_NODE_ID;

/** Private functions. ********************************************************/

/**
 * @brief Process CAN bus messages based on the configured message structs.
 *
 * This function iterates over the statically allocated CAN bus messages and
 * checks whether the incoming message's ID and DLC match the configuration.
 * If a match is found and a receiver handler exists, that handler is invoked.
 *
 * @param header Pointer to the CAN RX header.
 * @param data Pointer to the raw data of the CAN message.
 */
void process_can_message(CAN_RxHeaderTypeDef *header, uint8_t *data) {
  // CAN ID allocatee callbacks.
  // TODO: Overhead here, reduce via control flag or extended state variable?
  const can_header_t l_header = {
      header->StdId, header->ExtId, 0, header->DLC, header->RTR,
  };
  can_rx_can_id_allocatee_discovery(&l_header, data);
  can_rx_can_id_allocatee_assignment(&l_header, data);

  // CAN PWM Node DBC written callbacks.
  for (int i = 0; i < MC_STEPPER_CAN_DBC_IDX_COUNT; i++) {
    // Check if the message ID matches.
    if ((header->StdId & mod_dbc_messages[i].id_mask) ==
        mod_dbc_messages[i].message_id) {

      // Check if the message DLC matches, or if no check required (dlc == 0).
      if (mod_dbc_messages[i].dlc == 0 ||
          header->DLC == mod_dbc_messages[i].dlc) {

        // Call the rx_handler if it exists.
        // TODO: Use of function pointers as CAN Rx and Tx handlers need to be
        //  fully evaluated.
        if (mod_dbc_messages[i].rx_handler) {
          can_header_t reduced_header = {header->StdId, header->ExtId,
                                         header->IDE, header->DLC, header->RTR};
          mod_dbc_messages[i].rx_handler(&reduced_header, data);
        }
      } else {
        // Handle ID/DLC mismatch fault.
        // can_fault(); // TODO: IMPLEMENT.
        return;
      }
    }
  }
}

void can_rx_command_stepper(can_header_t *header, uint8_t *data) {
  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_COMMAND_STEPPER];

  const can_node_id_t node_id =
      (can_node_id_t)decode_signal(&msg->signals[0], data);
  const control_mode_t control_mode =
      (control_mode_t)decode_signal(&msg->signals[1], data);
  const bool enable = (bool)decode_signal(&msg->signals[2], data);
  const bool clear_faults = (bool)decode_signal(&msg->signals[3], data);
  // &msg->signals[4] is reserved0.
  const float target_position_rad =
      (float)decode_signal(&msg->signals[5], data);
  const float target_velocity_rad_s =
      (float)decode_signal(&msg->signals[6], data);

  // Node ID filter.
  if (node_id != can_node_id || node_id == CAN_ID_NODE_ID_BROADCAST
#ifdef REQUIRE_NODE_ID_ASSIGNMENT
      || node_id == CAN_ID_NODE_ID_UNASSIGNED
#endif
  ) {
    return;
  }

  // Fault clear: must be processed before any enable/setpoint logic so that a
  // single frame can atomically clear and re-enable.
  if (clear_faults && controls_state == CONTROLS_FAULT) {
    controls_state = CONTROLS_IDLE;
  }

  // Enable/disable gate. Disabling suppresses the setpoint update below, a
  // disabled motor should not silently accept a new target.
  if (enable) {
    tmc2209_enable();
    // Bring the state machine out of DISABLE so it can accept setpoints.
    if (controls_state == CONTROLS_DISABLE) {
      controls_state = CONTROLS_IDLE;
    }
  } else {
    tmc2209_disable();
    // Park the state machine so the control loop stops commanding steps.
    controls_state = CONTROLS_DISABLE;
    return; // No setpoint update when disabled.
  }

  // Setpoint dispatch. Only send a setpoint when the state machine is live
  // (IDLE, EXECUTING, or ON_POSITION_TARGET). Skip FAULT, caller must send
  // clear_faults first to recover, then send a new setpoint in a follow-up
  // frame or a combined frame (clear_faults=1 + valid mode in same message).
  if (controls_state == CONTROLS_FAULT) {
    return;
  }

  switch (control_mode) {
  case CONTROL_MODE_POSITION: {
    const mc_stepper_command_t cmd = {
        .mode = CONTROL_MODE_POSITION,
        .setpoint = target_position_rad,
    };
    controls_set_setpoint(cmd);
    break;
  }

  case CONTROL_MODE_SPEED: {
    const mc_stepper_command_t cmd = {
        .mode = CONTROL_MODE_SPEED,
        .setpoint = target_velocity_rad_s,
    };
    controls_set_setpoint(cmd);
    break;
  }

  case CONTROL_MODE_POSITION_RELATIVE: {
    const mc_stepper_command_t cmd = {
        .mode = CONTROL_MODE_POSITION_RELATIVE,
        .setpoint = target_position_rad,
    };
    controls_set_setpoint(cmd);
    break;
  }

  case CONTROL_MODE_POSITION_OPEN_LOOP: {
    const mc_stepper_command_t cmd = {
        .mode = CONTROL_MODE_POSITION_OPEN_LOOP,
        .setpoint = target_position_rad,
    };
    controls_set_setpoint(cmd);
    break;
  }

  case CONTROL_MODE_NONE: {
    // Explicit stop: command zero velocity so the control loop decelerates
    // cleanly via the slew filter, rather than cutting steps abruptly.
    const mc_stepper_command_t cmd = {
        .mode = CONTROL_MODE_SPEED,
        .setpoint = 0.0f,
    };
    controls_set_setpoint(cmd);
    break;
  }

  default:
    // Unknown mode - ignore to avoid driving the motor unexpectedly.
    break;
  }
}

void can_rx_command_motion_config_set(can_header_t *header, uint8_t *data) {
  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_MOTION_CONFIG_SET];

  const uint8_t node_id = (uint8_t)decode_signal(&msg->signals[0], data);
  const uint16_t step_size = (uint16_t)decode_signal(&msg->signals[1], data);
  const float max_acceleration_rad_s2 =
      (float)decode_signal(&msg->signals[2], data);
  const float max_velocity_rad_s = (float)decode_signal(&msg->signals[3], data);

  // Node ID filter.
  if (node_id != can_node_id || node_id == CAN_ID_NODE_ID_BROADCAST
#ifdef REQUIRE_NODE_ID_ASSIGNMENT
      || node_id == CAN_ID_NODE_ID_UNASSIGNED
#endif
  ) {
    return;
  }

  // Safety: only reconfigure while the driver is disabled. Changing the
  // microstep resolution mid-move would glitch the step stream.
  if (controls_state != CONTROLS_DISABLE) {
    return;
  }

  // Reject an implausible microstep setting; zero would collapse steps_per_rad.
  if (step_size == 0) {
    return;
  }

  // Capture the request; applied from the main context (see notes above).
  motion_step_size = step_size;
  motion_max_accel_rad_s2 = max_acceleration_rad_s2;
  motion_max_velocity_rad_s = max_velocity_rad_s;
  motion_pending = true;
}

void can_rx_command_motion_config_get(can_header_t *header, uint8_t *data) {
  const can_message_t *req =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_MOTION_CONFIG_GET];

  const uint8_t node_id = (uint8_t)decode_signal(&req->signals[0], data);

  // Node ID filter.
  if (node_id != can_node_id || node_id == CAN_ID_NODE_ID_BROADCAST
#ifdef REQUIRE_NODE_ID_ASSIGNMENT
      || node_id == CAN_ID_NODE_ID_UNASSIGNED
#endif
  ) {
    return;
  }

  const float source_sigs[3] = {(float)controls_get_microsteps(),
                                controls_get_max_accel_rad_s2(),
                                controls_get_max_velocity_rad_s()};

  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_MOTION_CONFIG_RESPONSE];
  uint32_t sigs[3] = {0};
  for (int i = 0; i < msg->signal_count; ++i) {
    sigs[i] = physical_to_raw(source_sigs[i], &msg->signals[i]);
  }
  can_send_message_raw32(&HCAN, msg, sigs);
}

void can_rx_command_current_config_set(can_header_t *header, uint8_t *data) {
  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CURRENT_CONFIG_SET];

  const uint8_t node_id = (uint8_t)decode_signal(&msg->signals[0], data);
  const uint16_t run_current_ma =
      (uint16_t)decode_signal(&msg->signals[1], data);
  const uint8_t current_run = (uint8_t)decode_signal(&msg->signals[2], data);
  const uint8_t current_hold = (uint8_t)decode_signal(&msg->signals[3], data);
  const uint8_t current_hold_delay =
      (uint8_t)decode_signal(&msg->signals[4], data);

  // Node ID filter.
  if (node_id != can_node_id || node_id == CAN_ID_NODE_ID_BROADCAST
#ifdef REQUIRE_NODE_ID_ASSIGNMENT
      || node_id == CAN_ID_NODE_ID_UNASSIGNED
#endif
  ) {
    return;
  }

  // Safety: only reconfigure while the driver is disabled.
  if (controls_state != CONTROLS_DISABLE) {
    return;
  }

  // Capture the request; applied from the main context (see notes above).
  currents_run_ma = run_current_ma;
  currents_run = current_run;
  currents_hold = current_hold;
  currents_hold_delay = current_hold_delay;
  currents_pending = true;
}

void can_rx_command_current_config_get(can_header_t *header, uint8_t *data) {
  const can_message_t *req =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CURRENT_CONFIG_GET];

  const uint8_t node_id = (uint8_t)decode_signal(&req->signals[0], data);

  // Node ID filter.
  if (node_id != can_node_id || node_id == CAN_ID_NODE_ID_BROADCAST
#ifdef REQUIRE_NODE_ID_ASSIGNMENT
      || node_id == CAN_ID_NODE_ID_UNASSIGNED
#endif
  ) {
    return;
  }

  uint16_t run_current_ma;
  uint8_t current_run, current_hold, current_hold_delay;
  tmc2209_get_currents(&run_current_ma, &current_run, &current_hold,
                       &current_hold_delay);

  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CURRENT_CONFIG_RESPONSE];
  const float source_sigs[4] = {(float)run_current_ma, (float)current_run,
                                (float)current_hold, (float)current_hold_delay};
  uint32_t sigs[4] = {0};
  for (int i = 0; i < msg->signal_count; ++i) {
    sigs[i] = physical_to_raw(source_sigs[i], &msg->signals[i]);
  }
  can_send_message_raw32(&HCAN, msg, sigs);
}

void can_rx_controls_config_set(can_header_t *header, uint8_t *data) {
  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CONTROLS_CONFIG_SET];

  const uint8_t node_id = (uint8_t)decode_signal(&msg->signals[0], data);
  // signals[1] is reserved0.
  const float k_p = (float)decode_signal(&msg->signals[2], data);
  const float k_i = (float)decode_signal(&msg->signals[3], data);
  const float k_d = (float)decode_signal(&msg->signals[4], data);

  // Node ID filter.
  if (node_id != can_node_id || node_id == CAN_ID_NODE_ID_BROADCAST
#ifdef REQUIRE_NODE_ID_ASSIGNMENT
      || node_id == CAN_ID_NODE_ID_UNASSIGNED
#endif
  ) {
    return;
  }

  controls_set_gains(k_p, k_i, k_d);
}

void can_rx_controls_config_get(can_header_t *header, uint8_t *data) {
  const can_message_t *req =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CONTROLS_CONFIG_GET];

  const uint8_t node_id = (uint8_t)decode_signal(&req->signals[0], data);

  // Node ID filter.
  if (node_id != can_node_id || node_id == CAN_ID_NODE_ID_BROADCAST
#ifdef REQUIRE_NODE_ID_ASSIGNMENT
      || node_id == CAN_ID_NODE_ID_UNASSIGNED
#endif
  ) {
    return;
  }

  float k_p, k_i, k_d;
  controls_get_gains(&k_p, &k_i, &k_d);

  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CONTROLS_CONFIG_RESPONSE];
  const float source_sigs[3] = {k_p, k_i, k_d};
  uint32_t sigs[3] = {0};
  for (int i = 0; i < msg->signal_count; ++i) {
    sigs[i] = physical_to_raw(source_sigs[i], &msg->signals[i]);
  }
  can_send_message_raw32(&HCAN, msg, sigs);
}

void can_rx_stallguard_set(can_header_t *header, uint8_t *data) {
  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_STALLGUARD_SET];

  const uint8_t node_id = (uint8_t)decode_signal(&msg->signals[0], data);
  const bool enable = (bool)decode_signal(&msg->signals[1], data);
  const uint8_t sgthrs = (uint8_t)decode_signal(&msg->signals[2], data);

  // Node ID filter.
  if (node_id != can_node_id || node_id == CAN_ID_NODE_ID_BROADCAST
#ifdef REQUIRE_NODE_ID_ASSIGNMENT
      || node_id == CAN_ID_NODE_ID_UNASSIGNED
#endif
  ) {
    return;
  }

  // Capture applied from the main context (blocking UART writes).
  stallguard_enable = enable;
  stallguard_sgthrs = sgthrs;
  stallguard_pending = true;
}

void can_rx_stallguard_get(can_header_t *header, uint8_t *data) {
  const can_message_t *req =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_STALLGUARD_GET];

  const uint8_t node_id = (uint8_t)decode_signal(&req->signals[0], data);

  // Node ID filter.
  if (node_id != can_node_id || node_id == CAN_ID_NODE_ID_BROADCAST
#ifdef REQUIRE_NODE_ID_ASSIGNMENT
      || node_id == CAN_ID_NODE_ID_UNASSIGNED
#endif
  ) {
    return;
  }

  bool enable, stalled;
  uint8_t sgthrs;
  uint16_t sg_result;
  tmc2209_stallguard_get(&enable, &sgthrs, &sg_result, &stalled);

  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_STALLGUARD_RESPONSE];
  const float source_sigs[4] = {(float)enable, (float)sgthrs, (float)sg_result,
                                (float)stalled};
  uint32_t sigs[4] = {0};
  for (int i = 0; i < msg->signal_count; ++i) {
    sigs[i] = physical_to_raw(source_sigs[i], &msg->signals[i]);
  }
  can_send_message_raw32(&HCAN, msg, sigs);
}

void can_rx_datetime_set(can_header_t *header, uint8_t *data) {
  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_DATETIME_SET];

  const uint8_t year = (uint8_t)decode_signal(&msg->signals[0], data);
  const uint8_t month = (uint8_t)decode_signal(&msg->signals[1], data);
  const uint8_t date = (uint8_t)decode_signal(&msg->signals[2], data);
  const uint8_t weekday = (uint8_t)decode_signal(&msg->signals[3], data);
  const uint8_t hours = (uint8_t)decode_signal(&msg->signals[4], data);
  const uint8_t minutes = (uint8_t)decode_signal(&msg->signals[5], data);
  const uint8_t seconds = (uint8_t)decode_signal(&msg->signals[6], data);

  set_date(year, month, date, weekday);
  set_time(hours, minutes, seconds);
}

void can_rx_datetime_get(can_header_t *header, uint8_t *data) {
  RTC_DateTypeDef sDate = {0};
  RTC_TimeTypeDef sTime = {0};

  // NOTE: Time must be read before date on STM32 HAL to unlock the shadow
  // registers correctly - reading date first can latch stale values.
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_DATETIME_RESPONSE];
  const uint32_t values[] = {
      sDate.Year,  sDate.Month,   sDate.Date,    sDate.WeekDay,
      sTime.Hours, sTime.Minutes, sTime.Seconds,
  };

  can_send_message_raw32(&HCAN, msg, values);
}

void can_rx_version_get(can_header_t *header, uint8_t *data) {
  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_VERSION_RESPONSE];
  const uint32_t values[] = {
      MC_STEPPER_VERSION_MAJOR,
      MC_STEPPER_VERSION_MINOR,
      MC_STEPPER_VERSION_PATCH,
      MC_STEPPER_VERSION_IDENTIFIER,
  };

  can_send_message_raw32(&HCAN, msg, values);
}

/** User implementations into STM32 HAL (overwrite weak HAL functions). *******/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  // Receive the CAN message.
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
    // Process the received message.
    process_can_message(&rx_header, rx_data);
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  // Receive the CAN message.
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK) {
    // Process the received message.
    process_can_message(&rx_header, rx_data);
  }
}

/** Public functions. *********************************************************/

void can_init(void) {
  // Configure CAN bus filters.
  CAN_FilterTypeDef can_filter_config;

  can_filter_config.FilterActivation = CAN_FILTER_ENABLE; // Enable the filter.
  can_filter_config.FilterBank = 0; // Assign filter to bank 0.
  can_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0; // Use FIFO0.
  // Set filter to accept all IDs.
  can_filter_config.FilterIdHigh = 0x0000;
  can_filter_config.FilterIdLow = 0x0000;
  // Set mask to 0 so that no bits are filtered, allow all messages.
  can_filter_config.FilterMaskIdHigh = 0x0000;
  can_filter_config.FilterMaskIdLow = 0x0000;
  // Use mask mode (compare message ID with mask).
  can_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  // Use 32-bit filter scale.
  can_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  // Filter bank config for single CAN setup.
  can_filter_config.SlaveStartFilterBank = 14;

  // Apply filter settings to CAN1.
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_config);

  // Start CAN1.
  HAL_CAN_Start(&hcan1);

  // Enable interrupts.
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // Setup CAN general members on the transmit header.
  tx_header.IDE = CAN_ID_STD;
  // Remaining members (DLC and StdId) are configured per message on transmit.
}

void can_db_init(void) {
  // Create the mutable copy of the DBC into RAM.
  memcpy(mod_dbc_messages, dbc_messages,
         MC_STEPPER_CAN_DBC_IDX_COUNT * sizeof(dbc_messages[0]));

  // Custom overrides.
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_COMMAND_STEPPER].rx_handler =
      (can_rx_handler_t)can_rx_command_stepper;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_MOTION_CONFIG_SET].rx_handler =
      (can_rx_handler_t)can_rx_command_motion_config_set;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_MOTION_CONFIG_GET].rx_handler =
      (can_rx_handler_t)can_rx_command_motion_config_get;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CURRENT_CONFIG_SET].rx_handler =
      (can_rx_handler_t)can_rx_command_current_config_set;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CURRENT_CONFIG_GET].rx_handler =
      (can_rx_handler_t)can_rx_command_current_config_get;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CONTROLS_CONFIG_SET].rx_handler =
      (can_rx_handler_t)can_rx_controls_config_set;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CONTROLS_CONFIG_GET].rx_handler =
      (can_rx_handler_t)can_rx_controls_config_get;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_STALLGUARD_SET].rx_handler =
      (can_rx_handler_t)can_rx_stallguard_set;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_STALLGUARD_GET].rx_handler =
      (can_rx_handler_t)can_rx_stallguard_get;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_DATETIME_SET].rx_handler =
      (can_rx_handler_t)can_rx_datetime_set;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_DATETIME_GET].rx_handler =
      (can_rx_handler_t)can_rx_datetime_get;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_VERSION_GET].rx_handler =
      (can_rx_handler_t)can_rx_version_get;
}

void can_stepper_reconfig_run(void) {
  // Currents: update VREF DAC and IHOLD/IRUN/IHOLDDELAY.
  if (currents_pending) {
    // Snapshot atomically against the RX interrupt, then clear the flag so a
    // concurrent request is not lost mid-apply.
    __disable_irq();
    const uint16_t run_ma = currents_run_ma;
    const uint8_t run = currents_run;
    const uint8_t hold = currents_hold;
    const uint8_t hold_delay = currents_hold_delay;
    currents_pending = false;
    __enable_irq();

    // Re-check the safety gate; the driver may have been enabled since the
    // request was queued.
    if (controls_state == CONTROLS_DISABLE) {
      tmc2209_set_currents(run_ma, run, hold, hold_delay);
    }
  }

  // Motion: update microstep resolution and the controls motion envelope.
  if (motion_pending) {
    __disable_irq();
    const uint16_t step_size = motion_step_size;
    const float max_accel_rad_s2 = motion_max_accel_rad_s2;
    const float max_velocity_rad_s = motion_max_velocity_rad_s;
    motion_pending = false;
    __enable_irq();

    if (controls_state == CONTROLS_DISABLE) {
      // controls_set_microsteps() updates steps_per_rad first; the accel set
      // then re-derives the step-rate cap against the new calibration.
      controls_set_microsteps(step_size);
      controls_set_max_accel_rad_s2(max_accel_rad_s2);
      controls_set_max_velocity_rad_s(max_velocity_rad_s);
      tmc2209_set_step_size(step_size);
    }
  }

  // StallGuard: register writes are non-disruptive, so apply in any state.
  if (stallguard_pending) {
    __disable_irq();
    const bool enable = stallguard_enable;
    const uint8_t sgthrs = stallguard_sgthrs;
    stallguard_pending = false;
    __enable_irq();

    tmc2209_stallguard_config(enable, sgthrs);
  }
}

HAL_StatusTypeDef can_send_message_raw32(CAN_HandleTypeDef *h_can_x,
                                         const can_message_t *msg,
                                         const uint32_t signal_values[]) {
  uint8_t data[8] = {0};

  for (int i = 0; i < msg->signal_count; ++i) {
    pack_signal_raw32(&msg->signals[i], data, signal_values[i]);
  }

  // Prepare the CAN transmit header.
  tx_header.StdId = msg->message_id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = msg->dlc;

  return HAL_CAN_AddTxMessage(h_can_x, &tx_header, data, &tx_mailbox);
}

bool can_tx_direct(const can_message_t *msg, const uint8_t data[8]) {
  // Prepare the CAN transmit header.
  tx_header.StdId = msg->message_id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = msg->dlc;

  // Transmit.
  const HAL_StatusTypeDef status =
      HAL_CAN_AddTxMessage(&HCAN, &tx_header, data, &tx_mailbox);
  if (status == HAL_OK) {
    return true;
  }
  return false;
}

void auto_can_id_allocatee_start(void) {
#ifndef ALLOW_CAN_NODE_ID_REASSIGNMENT
  return;
#endif
  const allocatee_config_t l_config = {
      can_tx_direct,
      get_uid_hash48_parts,
      allocatee_complete,
  };
  can_id_allocatee_start(l_config);
}

void allocatee_complete(const can_node_id_t node_id) {
  if (node_id == CAN_ID_NODE_ID_UNASSIGNED ||
      node_id == CAN_ID_NODE_ID_BROADCAST) {
    return;
  }

  can_node_id = node_id; // Update CAN node ID.

  // Patch all DBC message IDs with the assigned node_id.
  for (int i = 0; i < MC_STEPPER_CAN_DBC_IDX_COUNT; i++) {
    can_message_id_t msg_id;
    can_id_unpack(mod_dbc_messages[i].message_id, &msg_id, NULL);
    can_id_pack(msg_id, node_id, (can_id_t *)&mod_dbc_messages[i].message_id);
  }
}
