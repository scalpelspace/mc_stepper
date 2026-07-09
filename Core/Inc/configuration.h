/*******************************************************************************
 * @file configuration.h <- TODO: DEV CONFIGURATIONS!
 * @brief High level hardcoded configuration related declarations/definitions.
 *******************************************************************************
 */

#ifndef MC_STEPPER__CONFIGURATION_H
#define MC_STEPPER__CONFIGURATION_H

/** Configuration definitions. ************************************************/

// Centralized short name.
#define SCALPELSPACE_SHORT_NAME "mc_stepper" // Short name.

// Software version: `MAJOR.MINOR.PATCH-IDENTIFIER`.
#define MC_STEPPER_VERSION_MAJOR (0u)     // uint8_t number.
#define MC_STEPPER_VERSION_MINOR (2u)     // uint8_t number.
#define MC_STEPPER_VERSION_PATCH (1u)     // uint8_t number.
#define MC_STEPPER_VERSION_IDENTIFIER 'p' // ASCII character.

// CAN ID standard.
#define DEFAULT_CAN_NODE_ID (0u)       // Unassigned.
#define ALLOW_CAN_NODE_ID_REASSIGNMENT // Allow node ID reassignment always.

// Motor controller safety.
#define REQUIRE_NODE_ID_ASSIGNMENT // Require node ID assignment for control.

// Full telemetry flood on CAN bus intended for debug/development purposes.
#define MC_STEPPER_FULL_CAN_TELEMETRY

/** TMC2209 configuration. ****************************************************/

// TMC2209 default configuration.
#define MC_STEPPER_STEP_SIZE (16u)
#define MC_STEPPER_RUN_CURRENT_MA (300u)
#define MC_STEPPER_CURRENT_HOLD (1u)
#define MC_STEPPER_CURRENT_RUN (8u)
#define MC_STEPPER_CURRENT_HOLD_DELAY (1u)

/** Motion profile configuration. *********************************************/

// Controls motion profile default configuration.
#define MC_STEPPER_MAX_ACCELERATION_RAD_S2 (300.0f)
#define MC_STEPPER_MAX_VELOCITY_RAD_S (5.0f)

#endif
