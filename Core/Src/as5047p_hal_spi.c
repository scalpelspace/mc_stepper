/*******************************************************************************
 * @file as5047p_hal_spi.c
 * @brief AS5047P functions: abstracting STM32 HAL: SPI.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "as5047p_hal_spi.h"

/** Definitions. **************************************************************/

#define BIT_MODITY(src, i, val) ((src) ^= (-(val) ^ (src)) & (1UL << (i)))
#define BIT_READ(src, i) (((src) >> (i) & 1U))
#define BIT_TOGGLE(src, i) ((src) ^= 1UL << (i))

// Volatile register address.
#define AS5047P_NOP ((uint16_t)0x0000)
#define AS5047P_ERRFL ((uint16_t)0x0001)
#define AS5047P_PROG ((uint16_t)0x0003)
#define AS5047P_DIAAGC ((uint16_t)0x3FFC)
#define AS5047P_MAG ((uint16_t)0x3FFD)
#define AS5047P_ANGLEUNC ((uint16_t)0x3FFE)
#define AS5047P_ANGLECOM ((uint16_t)0x3FFF)

// Non-Volatile register address.
#define AS5047P_ZPOSM ((uint16_t)0x0016)
#define AS5047P_ZPOSL ((uint16_t)0x0017)
#define AS5047P_SETTINGS1 ((uint16_t)0x0018)
#define AS5047P_SETTINGS2 ((uint16_t)0x0019)

#define AS5047P_STEEINGS1_DEFAULT ((uint8_t)0x01)
#define AS5047P_STEEINGS2_DEFAULT ((uint8_t)0x00)

#define OP_WRITE ((uint8_t)0)
#define OP_READ ((uint8_t)1)

/** Public variables. *********************************************************/

float as5047p_angle_deg = 0;
uint16_t as5047p_angle_step = 0;

/** Private functions. ********************************************************/

/**
 * @brief Check data even parity.
 */
static uint8_t is_even_parity(uint16_t data) {
  uint8_t shift = 1;
  while (shift < (sizeof(data) * 8)) {
    data ^= (data >> shift);
    shift <<= 1;
  }
  return !(data & 0x1);
}

/**
 * @brief Transfer one 16-bit word over SPI (full-duplex).
 *
 * @param tx Word to transmit.
 * @return Word received during the transfer.
 */
static uint16_t as5047p_spi_transfer16(uint16_t tx) {
  uint16_t rx = 0;
  HAL_SPI_TransmitReceive(&AS5047P_HSPI, (uint8_t *)&tx, (uint8_t *)&rx, 1,
                          HAL_MAX_DELAY);
  return rx;
}

/**
 * @brief Sending read or write command to AS5047P.
 *
 * @param address Register address.
 * @param op_read_write Read of write operation.
 *
 * @notes SPI CS pin managed externally.
 */
static void as5047p_send_command(const uint16_t address,
                                 const uint8_t op_read_write) {
  uint16_t frame = address & 0x3FFF;

  // R/W: 0 for write, 1 for read.
  BIT_MODITY(frame, 14, op_read_write);

  // Parity bit(even) calculated on the lower 15 bits.
  if (!is_even_parity(frame)) {
    BIT_TOGGLE(frame, 15);
  }

  HAL_SPI_Transmit(&AS5047P_HSPI, (uint8_t *)&frame, 1, HAL_MAX_DELAY);
}

/**
 * @brief No operation instruction.
 */
static void as5047p_nop(void) {
  // NOP: clock one frame to flush/pipeline without changing registers.
  uint16_t nop_frame = AS5047P_NOP & 0x3FFF;
  BIT_MODITY(nop_frame, 14, OP_READ);
  if (!is_even_parity(nop_frame & 0x7FFF)) {
    BIT_TOGGLE(nop_frame, 15);
  }

  HAL_GPIO_WritePin(AS5047P_CSN_PORT, AS5047P_CSN_PIN, GPIO_PIN_RESET);
  (void)as5047p_spi_transfer16(nop_frame);
  HAL_GPIO_WritePin(AS5047P_CSN_PORT, AS5047P_CSN_PIN, GPIO_PIN_SET);
}

/**
 * @brief Sending data to register.
 *
 * @param address Register address.
 * @param data Data.
 */
static void as5047p_send_data(const uint16_t address, const uint16_t data) {
  uint16_t frame = data & 0x3FFF;

  // Data frame bit 14 always low.
  BIT_MODITY(frame, 14, 0);

  // Parity bit (even) over lower 15 bits.
  if (!is_even_parity(frame & 0x7FFF)) {
    BIT_TOGGLE(frame, 15);
  }

  HAL_GPIO_WritePin(AS5047P_CSN_PORT, AS5047P_CSN_PIN, GPIO_PIN_RESET);
  as5047p_send_command(address, OP_WRITE);
  HAL_GPIO_WritePin(AS5047P_CSN_PORT, AS5047P_CSN_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(AS5047P_CSN_PORT, AS5047P_CSN_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&AS5047P_HSPI, (uint8_t *)&frame, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(AS5047P_CSN_PORT, AS5047P_CSN_PIN, GPIO_PIN_SET);
}

/**
 * @brief Reading data from register.
 *
 * @param address Register address.
 *
 * @return Data.
 */
static uint16_t as5047p_read_data(uint16_t address) {
  uint16_t cmd = address & 0x3FFF;

  // R/W: 0 for write, 1 for read.
  BIT_MODITY(cmd, 14, OP_READ);

  // Parity bit (even) over lower 15 bits.
  if (!is_even_parity(cmd & 0x7FFF)) {
    BIT_TOGGLE(cmd, 15);
  }

  // Frame 1: send read command (response is returned on the next frame).
  HAL_GPIO_WritePin(AS5047P_CSN_PORT, AS5047P_CSN_PIN, GPIO_PIN_RESET);
  (void)as5047p_spi_transfer16(cmd);
  HAL_GPIO_WritePin(AS5047P_CSN_PORT, AS5047P_CSN_PIN, GPIO_PIN_SET);

  // Frame 2: clock out the response using NOP.
  uint16_t nop = AS5047P_NOP & 0x3FFF;
  BIT_MODITY(nop, 14, OP_READ);
  if (!is_even_parity(nop & 0x7FFF)) {
    BIT_TOGGLE(nop, 15);
  }

  HAL_GPIO_WritePin(AS5047P_CSN_PORT, AS5047P_CSN_PIN, GPIO_PIN_RESET);
  uint16_t data = as5047p_spi_transfer16(nop);
  HAL_GPIO_WritePin(AS5047P_CSN_PORT, AS5047P_CSN_PIN, GPIO_PIN_SET);

  return data;
}

/**
 * @brief Read current position.
 *
 * @param position Current position raw value.
 *
 * @return Status.
 * @retval == 0 -> Success.
 * @retval < 0  -> Error.
 */
static int8_t as5047p_get_position(uint16_t *position) {
#if AS5047P_USE_DAEC
  // Measured counts with dynamic angle error compensation.
  const uint16_t address = AS5047P_ANGLECOM;
#else
  // Measured counts with dynamic angle error compensation.
  const uint16_t address = AS5047P_ANGLEUNC;
#endif
  const uint16_t data = as5047p_read_data(address);
  if (BIT_READ(data, 14) == 0) {
    *position = data & 0x3FFF;
    return 0; // No error.
  }
  return -1; // Error occurred.
}

/** Public functions. *********************************************************/

void as5047p_reset(void) {
  HAL_GPIO_WritePin(AS5047P_CSN_PORT, AS5047P_CSN_PIN, GPIO_PIN_SET);
  HAL_Delay(5);

  // Clear pipeline.
  as5047p_nop();
}

void as5047p_init(uint8_t settings1, uint8_t settings2) {
  HAL_GPIO_WritePin(AS5047P_CSN_PORT, AS5047P_CSN_PIN, GPIO_PIN_SET);
  HAL_Delay(5);

  // SETTINGS1 bit 0 -> Factory Setting: Pre-Programmed to 1.
  BIT_MODITY(settings1, 0, 1);

  // SETTINGS1 bit 1 -> Not Used: Pre-Programmed to 0, must not be overwritten.
  BIT_MODITY(settings1, 1, 0);

  as5047p_send_data(AS5047P_SETTINGS1, (uint16_t)(settings1 & 0x00FF));
  as5047p_send_data(AS5047P_SETTINGS2, (uint16_t)(settings2 & 0x00FF));
}

uint16_t as5047p_get_error_status(void) {
  return as5047p_read_data(AS5047P_ERRFL);
}

void as5047p_get_angle(void) {
  // Update raw angle step count.
  const int8_t as5047p_error = as5047p_get_position(&as5047p_angle_step);
  if (as5047p_error == 0) {
    // Angle (degrees) = value * (360 / 2^14).
    as5047p_angle_deg = (float)(as5047p_angle_step * (360.0 / 0x4000));
  }
}

void as5047p_set_zero(const uint16_t position) {
  // 8 most significant bits of the zero position.
  as5047p_send_data(AS5047P_ZPOSM, ((position >> 6) & 0x00FF));

  // 6 least significant bits of the zero position.
  as5047p_send_data(AS5047P_ZPOSL, (position & 0x003F));

  as5047p_nop();
}
