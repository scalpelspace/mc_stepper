/*******************************************************************************
 * @file tmc2209_hal_uart.c
 * @brief TMC2209 functions: abstracting STM32 HAL: UART.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "tmc2209_hal_uart.h"

/** Definitions. **************************************************************/

// Software driven configurations.
#define TMC2209_DEFAULT_ADDRESS (0u) // MS1, MS2 both ground.
#define TMC2209_TIMEOUT_MS (10u)     // HAL blocking UART timeout.

// Hardware datasheet defined configurations.
#define TMC2209_SYNC (0x05)

#define TMC2209_REG_GCONF (0x00)
#define TMC2209_REG_GSTAT (0x01)
#define TMC2209_REG_IOIN (0x06)
#define TMC2209_REG_IHOLD_IRUN (0x10)
#define TMC2209_REG_TPOWERDOWN (0x11)
#define TMC2209_REG_TSTEP (0x12)
#define TMC2209_REG_CHOPCONF (0x6C)
#define TMC2209_REG_DRV_STATUS (0x6F)
#define TMC2209_REG_PWMCONF (0x70)
#define TMC2209_REG_PWM_SCALE (0x71)

#define TMC2209_CHOPCONF_MRES_SHIFT 24u
#define TMC2209_CHOPCONF_MRES_MASK (0x0Fu << TMC2209_CHOPCONF_MRES_SHIFT)
#define TMC2209_CHOPCONF_VSENSE_BIT (1u << 17)

#define TMC2209_GCONF_I_SCALE_ANALOG_BIT (1u << 0)
#define TMC2209_GCONF_INTERNAL_RSENSE_BIT (1u << 6)

/** Private functions. ********************************************************/

/**
 * @brief Map microsteps -> MRES encoding.
 *
 * 256->0, 128->1, 64->2, 32->3, 16->4, 8->5, 4->6, 2->7, 1->8.
 */
static bool microsteps_to_mres(const uint16_t microsteps, uint8_t *mres_out) {
  switch (microsteps) {
  case 256:
    *mres_out = 0;
    return true;
  case 128:
    *mres_out = 1;
    return true;
  case 64:
    *mres_out = 2;
    return true;
  case 32:
    *mres_out = 3;
    return true;
  case 16:
    *mres_out = 4;
    return true;
  case 8:
    *mres_out = 5;
    return true;
  case 4:
    *mres_out = 6;
    return true;
  case 2:
    *mres_out = 7;
    return true;
  case 1:
    *mres_out = 8;
    return true;
  default:
    return false;
  }
}

/**
 * @brief Compute CRC8 (polynomial 0x07, init 0x00), LSB-first.
 *
 * @param data Pointer to the data bytes to checksum.
 * @param len Number of bytes in buf.
 *
 * @return The calculated CRC8.
 */
static uint8_t tmc_crc8(const uint8_t *data, const uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t cur = data[i];
    for (uint8_t b = 0; b < 8; b++) {
      const uint8_t mix = (crc ^ cur) & 0x01;
      crc >>= 1;
      if (mix)
        crc ^= 0x8C; // Reversed poly of 0x07 for LSB-first.
      cur >>= 1;
    }
  }
  return crc;
}

/**
 * @brief Write a frame (8 bytes).
 *
 * [0]=0x05, [1]=slave, [2]=reg|0x80, [3..6]=data MSB..LSB, [7]=crc over [0..6].
 */
static HAL_StatusTypeDef tmc2209_write_reg(const uint8_t reg,
                                           const uint32_t value) {
  uint8_t tx[8];
  tx[0] = TMC2209_SYNC;
  tx[1] = TMC2209_DEFAULT_ADDRESS & 0x7F;
  tx[2] = ((reg & 0x7F) | 0x80); // Write bit.
  tx[3] = (uint8_t)(value >> 24);
  tx[4] = (uint8_t)(value >> 16);
  tx[5] = (uint8_t)(value >> 8);
  tx[6] = (uint8_t)(value >> 0);
  tx[7] = tmc_crc8(tx, 7);

  // Half-duplex: just transmit.
  HAL_HalfDuplex_EnableTransmitter(&TMC2209_HUART);
  return HAL_UART_Transmit(&TMC2209_HUART, tx, sizeof(tx), TMC2209_TIMEOUT_MS);
}

/**
 * @brief Send read request frame (4 bytes) and read frame (8 bytes).
 *
 * [0]=0x05, [1]=slave, [2]=reg (no write bit), [3]=crc over [0..2]
 * [0]=0x05, [1]=0xFF? (or slave), [2]=reg, [3..6]=data, [7]=crc over [0..6]
 */
static HAL_StatusTypeDef tmc2209_read_reg(const uint8_t reg,
                                          uint32_t *out_value) {
  uint8_t req[4];
  req[0] = TMC2209_SYNC;
  req[1] = TMC2209_DEFAULT_ADDRESS & 0x7F;
  req[2] = (reg & 0x7F); // Read.
  req[3] = tmc_crc8(req, 3);

  HAL_HalfDuplex_EnableTransmitter(&TMC2209_HUART);
  HAL_StatusTypeDef st =
      HAL_UART_Transmit(&TMC2209_HUART, req, 4, TMC2209_TIMEOUT_MS);
  if (st != HAL_OK)
    return st;

  HAL_HalfDuplex_EnableReceiver(&TMC2209_HUART);
  uint8_t rx[8] = {0};
  st = HAL_UART_Receive(&TMC2209_HUART, rx, 8, TMC2209_TIMEOUT_MS);
  if (st != HAL_OK)
    return st;

  // Basic validation.
  if (rx[0] != TMC2209_SYNC)
    return HAL_ERROR;
  if ((rx[2] & 0x7F) != (reg & 0x7F)) // rx[2] should match reg.
    return HAL_ERROR;

  const uint8_t crc = tmc_crc8(rx, 7);
  if (crc != rx[7])
    return HAL_ERROR;

  *out_value = ((uint32_t)rx[3] << 24) | ((uint32_t)rx[4] << 16) |
               ((uint32_t)rx[5] << 8) | ((uint32_t)rx[6] << 0);

  return HAL_OK;
}

/** Public functions. *********************************************************/

HAL_StatusTypeDef tmc2209_clear_gstat(void) {
  // Writing 1s clears flags in GSTAT.
  return tmc2209_write_reg(TMC2209_REG_GSTAT, 0x00000007u);
}

HAL_StatusTypeDef tmc2209_set_microsteps(const uint16_t microsteps) {
  uint8_t mres = 0;
  if (!microsteps_to_mres(microsteps, &mres))
    return HAL_ERROR;

  uint32_t chopconf = 0;
  const HAL_StatusTypeDef st =
      tmc2209_read_reg(TMC2209_REG_CHOPCONF, &chopconf);
  if (st != HAL_OK)
    return st;

  chopconf &= ~TMC2209_CHOPCONF_MRES_MASK;
  chopconf |= ((uint32_t)mres << TMC2209_CHOPCONF_MRES_SHIFT);

  return tmc2209_write_reg(TMC2209_REG_CHOPCONF, chopconf);
}

HAL_StatusTypeDef tmc2209_set_ihold_irun(const uint8_t ihold,
                                         const uint8_t irun,
                                         const uint8_t iholddelay) {
  // IHOLD_IRUN bits:
  // [4:0] IHOLD, [12:8] IRUN, [19:16] IHOLDDELAY.
  uint32_t v = 0;
  v |= ((uint32_t)(ihold & 0x1F) << 0);
  v |= ((uint32_t)(irun & 0x1F) << 8);
  v |= ((uint32_t)(iholddelay & 0x0F) << 16);
  return tmc2209_write_reg(TMC2209_REG_IHOLD_IRUN, v);
}

HAL_StatusTypeDef tmc2209_set_current_sense(const bool use_internal_rsense,
                                            const bool vsense_low_018v) {
  uint32_t gconf = 0;
  HAL_StatusTypeDef st = tmc2209_read_reg(TMC2209_REG_GCONF, &gconf);
  if (st != HAL_OK)
    return st;

  if (use_internal_rsense)
    gconf |= TMC2209_GCONF_INTERNAL_RSENSE_BIT;
  else
    gconf &= ~TMC2209_GCONF_INTERNAL_RSENSE_BIT;

  st = tmc2209_write_reg(TMC2209_REG_GCONF, gconf);
  if (st != HAL_OK)
    return st;

  uint32_t chopconf = 0;
  st = tmc2209_read_reg(TMC2209_REG_CHOPCONF, &chopconf);
  if (st != HAL_OK)
    return st;

  if (vsense_low_018v)
    chopconf |= TMC2209_CHOPCONF_VSENSE_BIT;
  else
    chopconf &= ~TMC2209_CHOPCONF_VSENSE_BIT;

  return tmc2209_write_reg(TMC2209_REG_CHOPCONF, chopconf);
}
