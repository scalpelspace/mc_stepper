/*******************************************************************************
 * @file run.h
 * @brief Centralized main loop run logic running in main.c.
 *******************************************************************************
 */

#ifndef MC_STEPPER__RUN_H
#define MC_STEPPER__RUN_H

/** Definitions. **************************************************************/

#define VELOCITY_AVERAGE_N (4u)

/** Public functions. *********************************************************/

void mc_stepper_init(void);
void mc_stepper_run(void);

#endif
