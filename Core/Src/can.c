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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Private variables. ********************************************************/

// Transmit variables.
CAN_TxHeaderTypeDef tx_header;
uint8_t tx_buffer[8];
uint32_t tx_mailbox;

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

  // Node ID filter, reject messages not explicitly referencing this node.
  if (node_id != can_node_id && node_id != CAN_ID_NODE_ID_BROADCAST) {
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

void can_rx_command_stepper_current(can_header_t *header, uint8_t *data) {
  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_COMMAND_STEPPER_CURRENT];

  const uint8_t node_id = (uint8_t)decode_signal(&msg->signals[0], data);
  const uint8_t update_current = (uint8_t)decode_signal(&msg->signals[1], data);
  const uint8_t persist = (uint8_t)decode_signal(&msg->signals[2], data);
  // &msg->signals[3] is reserved0.
  const uint8_t run_ma = (uint8_t)decode_signal(&msg->signals[4], data);
  const uint8_t hold_ma = (uint8_t)decode_signal(&msg->signals[5], data);
  const uint8_t hold_delay = (uint8_t)decode_signal(&msg->signals[6], data);

  // TODO(Daniel): Implement controls/re-init calls.
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
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_DATETIME_GET_RESPONSE];
  const uint32_t values[] = {
      sDate.Year,  sDate.Month,   sDate.Date,    sDate.WeekDay,
      sTime.Hours, sTime.Minutes, sTime.Seconds,
  };

  can_send_message_raw32(&HCAN, msg, values);
}

void can_rx_version_get(can_header_t *header, uint8_t *data) {
  const can_message_t *msg =
      &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_VERSION_GET_RESPONSE];
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
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_COMMAND_STEPPER_CURRENT].rx_handler =
      (can_rx_handler_t)can_rx_command_stepper_current;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_DATETIME_SET].rx_handler =
      (can_rx_handler_t)can_rx_datetime_set;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_DATETIME_GET].rx_handler =
      (can_rx_handler_t)can_rx_datetime_get;
  mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_VERSION_GET].rx_handler =
      (can_rx_handler_t)can_rx_version_get;
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
