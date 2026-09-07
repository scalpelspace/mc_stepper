/*******************************************************************************
 * @file telemetry.c
 * @brief General CAN/UART (comm) telemetry message transmission functions.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "telemetry.h"
#include "as5047p_hal_spi.h"
#include "can.h"
#include "comm.h"
#include "controls.h"
#include "mcu_temp_hal_adc.h"
#include "tmc2209_runner.h"
#include <stdio.h>

/** Public functions. *********************************************************/

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

void can_tx_sensor(void) {
  const can_message_t *msg = &mod_dbc_messages[MC_STEPPER_CAN_DBC_IDX_SENSOR];
  const uint32_t values[] = {
      (uint32_t)as5047p_angle_step,
      physical_to_raw(controls_position_meas_rad, &msg->signals[1]),
  };
  can_send_message_raw32(&HCAN, msg, values);
}

void can_tx_controls_diagnostic(void) {
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

void can_tx_stallguard_event(void) {
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

void comm_tx_status(void) {
  // Snapshot the volatiles once so every field describes the same tick.
  const controls_state_t state = controls_state;
  const float position = controls_position_meas_rad;
  const float position_setpoint = controls_position_setpoint;
  const float velocity_setpoint = controls_velocity_setpoint;
  const float step_rate = controls_step_rate_command;

  printf("state=%s\r\n", comm_state_name(state));
  printf("mode=%s\r\n", comm_mode_name(controls_get_mode()));
  printf("pos=%.4f\r\n", (double)position);
  printf("pos_sp=%.4f\r\n", (double)position_setpoint);
  printf("pos_err=%.4f\r\n", (double)(position_setpoint - position));
  printf("vel_sp=%.4f\r\n", (double)velocity_setpoint);
  printf("step_rate=%.1f\r\n", (double)step_rate);
  printf("drv_status=0x%04X\r\n", tmc2209_get_status());
}
