/*******************************************************************************
 * @file telemetry.h
 * @brief General CAN/UART (comm) telemetry message transmission functions.
 *******************************************************************************
 */

#ifndef MC_STEPPER__TELEMETRY_H
#define MC_STEPPER__TELEMETRY_H

/** Public functions. *********************************************************/

void can_tx_state(void);
void can_tx_sensor(void);
void can_tx_controls_diagnostic(void);
void can_tx_stallguard_event(void);

void comm_tx_status(void);

#endif
