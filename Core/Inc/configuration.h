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
#define MC_STEPPER_VERSION_MINOR (5u)     // uint8_t number.
#define MC_STEPPER_VERSION_PATCH (0u)     // uint8_t number.
#define MC_STEPPER_VERSION_IDENTIFIER 'p' // ASCII character.

// CAN ID standard.
// Node ID held from boot, in range [0, 30]. Assigned pre-compile time, the
// allocation protocol overwrites it at runtime if allowed (see below).
#define DEFAULT_CAN_NODE_ID (0u) // Unassigned.
// Allow the CAN ID allocation protocol to (re)assign this node's Node ID. Sets
// the advertised can_alloc_mode_t. The node participates in allocation either
// way, so the allocator can see which Node IDs are already taken on the bus.
//   Defined   -> CAN_ALLOC_MODE_REASSIGNABLE, DEFAULT_CAN_NODE_ID is only the
//                ID held until the allocator assigns one.
//   Undefined -> CAN_ALLOC_MODE_NOT_REASSIGNABLE, the node advertises and holds
//                DEFAULT_CAN_NODE_ID for its lifetime, and the allocator
//                reserves that ID instead of handing it to another node.
//                DEFAULT_CAN_NODE_ID must be in [1, 30].
#define ALLOW_CAN_NODE_ID_ALLOCATION

// Disable firmware driven WS2812B control (ie, boot colour, status, etc). The
// LED is then only set externally via CAN or UART (comm).
// #define LED_EXTERNAL_CONTROL_ONLY

/** Node ID allocation and control behaviour configuration. *******************/

// Motor controller safety.
#define REQUIRE_NODE_ID_ASSIGNMENT // Require node ID assignment for control.

/** Telemetry configuration. **************************************************/

// Full telemetry flood on CAN bus intended for debug/development purposes.
#define MC_STEPPER_FULL_CAN_TELEMETRY

// Full telemetry flood on UART intended for debug/development purposes.
// #define MC_STEPPER_FULL_COMM_TELEMETRY

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
