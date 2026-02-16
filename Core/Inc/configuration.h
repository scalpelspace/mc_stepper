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
#define MC_STEPPER_VERSION_MAJOR (0U)     // uint8_t number.
#define MC_STEPPER_VERSION_MINOR (1U)     // uint8_t number.
#define MC_STEPPER_VERSION_PATCH (0U)     // uint8_t number.
#define MC_STEPPER_VERSION_IDENTIFIER 'a' // ASCII character.

#define MC_STEPPER_USE_ABSOLUTE_ENCODER 1 // Magnetic absolute encoder.
#define MC_STEPPER_USE_RELATIVE_ENCODER 1 // ABI quadrature encoder.

#endif
