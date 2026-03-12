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
#define MC_STEPPER_VERSION_MINOR (1u)     // uint8_t number.
#define MC_STEPPER_VERSION_PATCH (0u)     // uint8_t number.
#define MC_STEPPER_VERSION_IDENTIFIER 'a' // ASCII character.

// CAN ID standard.
#define DEFAULT_CAN_NODE_ID (0u) // Unassigned.

// Full telemetry flood on CAN bus intended for debug/development purposes.
#define MC_STEPPER_FULL_CAN_TELEMETRY

#define MC_STEPPER_USE_ABSOLUTE_ENCODER 1 // Magnetic absolute encoder.
#define MC_STEPPER_USE_RELATIVE_ENCODER 1 // ABI quadrature encoder.

#endif
