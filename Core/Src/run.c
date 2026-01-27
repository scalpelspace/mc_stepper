/*******************************************************************************
 * @file run.c
 * @brief Centralized main loop run logic running in main.c.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "run.h"
#include "abi_encoder_hal_tim.h"
#include "as5047p_hal_spi.h"
#include "configuration.h"
#include "ws2812b_hal_pwm.h"

/** Private types. ************************************************************/

typedef enum mc_stepper_state_e {
  MC_STEPPER_INIT,
  MC_STEPPER_RUN,
} mc_stepper_state_t;

/** Private variables. ********************************************************/

// Central state machine.
static mc_stepper_state_t sys_state = MC_STEPPER_INIT;

// Sensor variables.
static abi_encoder_t abi_encoder = {0};

/** Private functions. ********************************************************/

static void state_init(void) {
  // TODO Low level peripherals.
  // can_init();

  // On-board miscellaneous components.
  ws2812b_init();
  ws2812b_set_colour(0, 2, 1, 3); // Init colour.
  ws2812b_update();

  // TODO Scheduler.
  // scheduler_init(); // Initialize scheduler.
  // scheduler_add_task(bmp390_get_data, 10);

  // Sensors.
#if MC_STEPPER_USE_ABSOLUTE_ENCODER
  as5047p_init(0x01, 0x00);
  HAL_Delay(1);
#endif
#if MC_STEPPER_USE_RELATIVE_ENCODER
  abi_encoder_init(&abi_encoder, &htim2);
#endif
}

static void run_state_machine(void) {
  switch (sys_state) {
  case MC_STEPPER_INIT:
    state_init();
    sys_state = MC_STEPPER_RUN;
    break;
  case MC_STEPPER_RUN:
    as5047p_get_angle();
    break;
  }
}

/** Public functions. *********************************************************/

void mc_stepper_run(void) {
  run_state_machine();
  // TODO scheduler_run(); // Run the scheduler.
}
