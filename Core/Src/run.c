/*******************************************************************************
 * @file run.c
 * @brief Centralized main loop run logic running in main.c.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "run.h"
#include "scheduler.h"

/** Public functions. *********************************************************/

void mc_stepper_run(void) {
  scheduler_run(); // Run the scheduler.
}
