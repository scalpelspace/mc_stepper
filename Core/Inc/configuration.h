/*******************************************************************************
 * @file configuration.h <- TODO: DEV CONFIGURATIONS!
 * @brief High level hardcoded configuration related declarations/definitions.
 *******************************************************************************
 */

#ifndef MC_STEPPER__CONFIGURATION_H
#define MC_STEPPER__CONFIGURATION_H

/** Includes. *****************************************************************/

#include <math.h>

/** Configuration definitions. ************************************************/

// Centralized short name.
#define SCALPELSPACE_SHORT_NAME "mc_stepper" // Short name.

// Software version: `MAJOR.MINOR.PATCH-IDENTIFIER`.
#define MC_STEPPER_VERSION_MAJOR (0u)     // uint8_t number.
#define MC_STEPPER_VERSION_MINOR (1u)     // uint8_t number.
#define MC_STEPPER_VERSION_PATCH (0u)     // uint8_t number.
#define MC_STEPPER_VERSION_IDENTIFIER 'a' // ASCII character.

// CAN ID standard.
#define DEFAULT_CAN_NODE_ID (0u) // Unassigned.
#define ALLOW_CAN_NODE_ID_REASSIGNMENT // Allow node ID reassignment always.

// Full telemetry flood on CAN bus intended for debug/development purposes.
#define MC_STEPPER_FULL_CAN_TELEMETRY

#define MC_STEPPER_USE_ABSOLUTE_ENCODER 1 // Magnetic absolute encoder.
#define MC_STEPPER_USE_RELATIVE_ENCODER 1 // ABI quadrature encoder.

// TMC2209 default configuration.
#define DEFAULT_STEP_SIZE (16u)
#define DEFAULT_RUN_CURRENT_MA (300u)
#define DEFAULT_CURRENT_HOLD (1u)
#define DEFAULT_CURRENT_RUN (8u)
#define DEFAULT_CURRENT_HOLD_DELAY (1u)
#define DEFAULT_MAX_ACCELERATION_RAD_S2 (300.0f)

// Controls default configuration.
#define DEFAULT_MAX_VELOCITY_RAD_S (5.0f)

#endif
