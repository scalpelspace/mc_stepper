/*******************************************************************************
 * @file as5047p_hal_spi.h
 * @brief AS5047P functions: abstracting STM32 HAL: SPI.
 *******************************************************************************
 */

#ifndef MC_STEPPER__AS5047P_HAL_SPI_H
#define MC_STEPPER__AS5047P_HAL_SPI_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"
#include <stdint.h>

/** STM32 port and pin configs. ***********************************************/

extern SPI_HandleTypeDef hspi1;

// SPI.
#define AS5047P_HSPI hspi1
// TODO: CSn pin subject to change for production (PA15, set high).
#define AS5047P_CSN_PORT GPIOB
#define AS5047P_CSN_PIN GPIO_PIN_0

/** Definitions. **************************************************************/

#define AS5047P_USE_DAEC 1 // Enable dynamic angle error compensation (DAEC).

/** Public variables. *********************************************************/

extern float as5047p_angle_deg;

/** Public functions. *********************************************************/

/**
 * @brief Reset.
 */
void as5047p_reset(void);

/**
 * @brief Setup AS5047P.
 *
 * @param settings1 Config 1.
 * @param settings2 Config 2.
 */
void as5047p_init(uint8_t settings1, uint8_t settings2);

/**
 * @brief Set specify position as zero.
 *
 * @param position Position raw value.
 */
void as5047p_set_zero(uint16_t position);

/**
 * @brief Read current angle in degree.
 */
void as5047p_get_angle(void);

/**
 * @brief Reading error flags.
 *
 * @return Error flags. 0 for no errors occurred.
 */
uint16_t as5047p_get_error_status(void);

#endif
