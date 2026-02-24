/*******************************************************************************
 * @file comm.c
 * @brief USART1 Communication Interface: abstracting STM32 HAL: UART.
 *******************************************************************************
 * @note:
 * Transmit functions should be handled via `stdio.h` `printf()` function. The
 * `_write()` function is implemented in `comm.c`.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "comm.h"
#include "configuration.h"
#include "tmc2209_runner.h"
#include "ws2812b_hal_pwm.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tmc2209_hal_uart.h>

/** Private variables. ********************************************************/

static osMutexId_t s_printf_mutex = NULL;
static osMessageQueueId_t s_comm_queue = NULL;

static char comm_line[COMM_LINE_MAX];
static volatile uint16_t comm_line_len = 0;
static uint8_t comm_rx_byte;

/** Private functions. ********************************************************/

int _write(int file, char *ptr, int len) {
  osMutexAcquire(s_printf_mutex, osWaitForever);
  HAL_UART_Transmit(&COMM_HUART, (uint8_t *)ptr, len, 20);
  osMutexRelease(s_printf_mutex);
  return len;
}

/** User implementations into STM32 HAL (overwrite weak HAL functions). *******/

void HAL_UART_RxCpltCallback_comm(UART_HandleTypeDef *huart) {
  if (huart == &COMM_HUART) {
    uint8_t b = comm_rx_byte;

    HAL_UART_Receive_DMA(&COMM_HUART, &comm_rx_byte, 1);

    // Basic line editing/framing.
    if (b == '\r') {
      return; // Ignore CR.
    }

    if (b == '\n') {
      // End of line: null-terminate and handle command.
      if (comm_line_len >= COMM_LINE_MAX)
        comm_line_len = COMM_LINE_MAX - 1;
      comm_line[comm_line_len] = '\0';

      // Copy into queue message.
      comm_line_message_t msg;
      memcpy(msg.line, comm_line, COMM_LINE_MAX);

      // Reset for next line.
      comm_line_len = 0;

      if (s_comm_queue != NULL) {
        osMessageQueuePut(s_comm_queue, &msg, 0U, 0U);
      }
      return;
    }

    // Normal character.
    if (comm_line_len < (COMM_LINE_MAX - 1)) {
      comm_line[comm_line_len++] = (char)b;
    } else {
      // Overflow: drop line.
      comm_line_len = 0;
    }
  }
}

/** Public functions. *********************************************************/

void printf_set_mutexes(osMutexId_t printf_mutex) {
  s_printf_mutex = printf_mutex;
}

void comm_uart_set_queue(osMessageQueueId_t comm_queue) {
  s_comm_queue = comm_queue;
}

void comm_init(void) {
  comm_line_len = 0;
  // Initialize single byte DMA.
  HAL_UART_Receive_DMA(&COMM_HUART, &comm_rx_byte, 1);
}

void comm_handle_line(const char *line) {
  // Skip leading spaces.
  while (*line == ' ' || *line == '\t')
    line++;

  if (strcmp(line, "version") == 0 || strcmp(line, "ver") == 0) {
    printf("%s %u.%u.%u.%c\r\n", SCALPELSPACE_SHORT_NAME,
           MC_STEPPER_VERSION_MAJOR, MC_STEPPER_VERSION_MINOR,
           MC_STEPPER_VERSION_PATCH, MC_STEPPER_VERSION_IDENTIFIER);
  }

  else if (strncmp(line, "rgb", 3) == 0) {
    unsigned int r_raw, g_raw, b_raw;
    // Accept formats like:
    // "rgb 255,0,128"
    // "rgb 255, 0, 128"
    if (sscanf(line + 3, "%u , %u , %u", &r_raw, &g_raw, &b_raw) == 3) {
      if (r_raw <= 255 && g_raw <= 255 && b_raw <= 255) {
        uint8_t r = (uint8_t)r_raw;
        uint8_t g = (uint8_t)g_raw;
        uint8_t b = (uint8_t)b_raw;
        ws2812b_set_colour(0, r, g, b);
        ws2812b_update();
      } else {
        printf("Error: RGB values must be [0,255]\r\n");
      }
    } else {
      printf("Error: expected `rgb <R>, <G>, <B>`\r\n");
    }
  }

  else if (strncmp(line, "sr", 1) == 0) {
    int step_rate = 0;
    if (sscanf(line + 2, "%d", &step_rate) == 1) {
      if (step_rate >= -65535 && step_rate <= 65535) {
        if (step_rate >= 0) { // Check sign for direction.
          tmc2209_step_direction(true);
        } else {
          tmc2209_step_direction(false);
        }
        const uint16_t step_rate_abs = (uint16_t)abs(step_rate);
        tmc2209_step_set_freq_hz(step_rate_abs);
      } else {
        printf("Error: Step rate must be [-65535,65535]\r\n");
      }
    } else {
      printf("Error: expected `sr <step_rate>`\r\n");
    }
  }

  else { // Error: unknown command.
  }
}
