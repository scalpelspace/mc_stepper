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

  // TMC2209.
  const tmc2209_configuration_t default_config = {true, 16, 500,     4,
                                                  12,   8,  20000.0f};
  tmc2209_init(default_config);

  // Sensors.
#if MC_STEPPER_USE_ABSOLUTE_ENCODER
  as5047p_init(0x01, 0x00);
  HAL_Delay(1);
#endif
#if MC_STEPPER_USE_RELATIVE_ENCODER
  abi_encoder_init(&abi_encoder, &htim2);
#endif

  // CAN allocatee begin.
  const allocatee_config_t l_config = {
      can_tx_direct,
      get_uid_hash48_parts,
      allocatee_complete,
  };
  can_id_allocatee_start(l_config);

  // Closed-loop controls.
  controls_init();
}

void sensors_run(void) {
  // Absolute angle sensor.
  as5047p_get_angle();
#ifdef MC_STEPPER_FULL_CAN_TELEMETRY
  can_tx_absolute_angle();
#endif
}
