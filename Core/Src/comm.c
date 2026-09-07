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
#include "controls.h"
#include "tmc2209_runner.h"
#include "uid_hash48.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** Definitions. **************************************************************/

#define COMM_LINE_MAX 96

/** Private variables. ********************************************************/

// Line assembly, written from the RX interrupt only.
static char comm_line[COMM_LINE_MAX];
static volatile uint16_t comm_line_len = 0;
static uint8_t comm_rx_byte;

// Completed line handed over to the main context. The interrupt publishes into
// comm_command only while comm_command_ready is clear, so the buffer stays
// stable for the whole of comm_run(). A line arriving while one is still
// pending is dropped and reported rather than overwriting it mid-parse.
static char comm_command[COMM_LINE_MAX];
static volatile bool comm_command_ready = false;
static volatile bool comm_command_dropped = false;

/** Private functions. ********************************************************/

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&COMM_HUART, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

/**
 * @brief Human readable controls_state_t name.
 */
static const char *comm_state_name(const controls_state_t state) {
  switch (state) {
  case CONTROLS_DISABLE:
    return "disable";
  case CONTROLS_IDLE:
    return "idle";
  case CONTROLS_START:
    return "start";
  case CONTROLS_EXECUTING:
    return "executing";
  case CONTROLS_ON_POSITION_TARGET:
    return "on_target";
  case CONTROLS_FAULT:
    return "fault";
  default:
    return "unknown";
  }
}

/**
 * @brief Human readable control_mode_t name, matching the command words.
 */
static const char *comm_mode_name(const control_mode_t mode) {
  switch (mode) {
  case CONTROL_MODE_NONE:
    return "none";
  case CONTROL_MODE_POSITION:
    return "pos";
  case CONTROL_MODE_SPEED:
    return "spd";
  case CONTROL_MODE_POSITION_RELATIVE:
    return "rel";
  case CONTROL_MODE_POSITION_OPEN_LOOP:
    return "ol";
  default:
    return "unknown";
  }
}

/**
 * @brief Match the first whole word of a line against a command name.
 *
 * Whole-word matching, so a short command never shadows a longer one sharing
 * its prefix (e.g. `sg` against `sgthrs`, or `pos` against `p`).
 *
 * @param line Command line, leading whitespace already stripped.
 * @param command Command word to match.
 * @param args Output pointer to the first argument character, leading
 *             whitespace stripped. Points at the terminator when no arguments
 *             were given. Nullable.
 *
 * @return true when the line's first word is command.
 */
static bool comm_match(const char *line, const char *command,
                       const char **args) {
  const size_t length = strlen(command);

  if (strncmp(line, command, length) != 0) {
    return false;
  }

  const char *tail = line + length;

  // Reject a partial match against a longer word.
  if (*tail != '\0' && *tail != ' ' && *tail != '\t') {
    return false;
  }

  while (*tail == ' ' || *tail == '\t') {
    tail++;
  }

  if (args) {
    *args = tail;
  }
  return true;
}

/**
 * @brief Enable or disable the driver and park the control loop accordingly.
 *
 * Mirrors the enable gate of the CAN command_stepper handler: enabling lifts
 * the FSM out of DISABLE so it can accept setpoints, disabling parks it so the
 * loop stops commanding steps.
 *
 * @param enable True to enable the driver, false to disable and park.
 */
static void comm_set_enable(const bool enable) {
  if (enable) {
    tmc2209_enable();
    if (controls_state == CONTROLS_DISABLE) {
      controls_state = CONTROLS_IDLE;
    }
  } else {
    tmc2209_disable();
    controls_state = CONTROLS_DISABLE;
  }
}

/**
 * @brief Parse a setpoint and dispatch it to the control loop.
 *
 * Shared by every motion command word. Rejects the states in which the loop
 * would silently swallow the setpoint, so the operator gets a reason back
 * instead of a motor that does not move.
 *
 * @param args Argument tail holding the setpoint.
 * @param mode Control mode to command.
 * @param usage Usage string quoted back on a parse failure.
 */
static void comm_command_motion(const char *args, const control_mode_t mode,
                                const char *usage) {
  float setpoint = 0.0f;

  if (sscanf(args, "%f", &setpoint) != 1) {
    printf("Error: expected `%s`\r\n", usage);
    return;
  }

  if (!isfinite(setpoint)) {
    printf("Error: setpoint must be finite\r\n");
    return;
  }

  // The loop only accepts setpoints while live. Disabled and faulted are
  // reported rather than dropped, matching the CAN handler's precedence.
  if (controls_state == CONTROLS_DISABLE) {
    printf("Error: driver disabled, run `en` first\r\n");
    return;
  }
  if (controls_state == CONTROLS_FAULT) {
    printf("Error: faulted, run `clr` first\r\n");
    return;
  }

  const mc_stepper_command_t command = {.mode = mode, .setpoint = setpoint};
  controls_set_command(command);

  printf("%s %.4f\r\n", comm_mode_name(mode), (double)setpoint);
}

/**
 * @brief Report the control loop, motion and driver status (`stat`).
 */
static void comm_print_status(void) {
  // Snapshot the volatiles once so every field describes the same tick.
  const controls_state_t state = controls_state;
  const float position = controls_position_meas_rad;
  const float position_setpoint = controls_position_setpoint;
  const float velocity_setpoint = controls_velocity_setpoint;
  const float step_rate = controls_step_rate_command;

  printf("state %s\r\n", comm_state_name(state));
  printf("mode %s\r\n", comm_mode_name(controls_get_mode()));
  printf("pos %.4f rad\r\n", (double)position);
  printf("pos_sp %.4f rad\r\n", (double)position_setpoint);
  printf("pos_err %.4f rad\r\n", (double)(position_setpoint - position));
  printf("vel_sp %.4f rad/s\r\n", (double)velocity_setpoint);
  printf("step_rate %.1f steps/s\r\n", (double)step_rate);
  printf("drv_status 0x%04X\r\n", tmc2209_get_status());
}

/**
 * @brief Report or update the motion envelope (`motion`).
 *
 * @param args Argument tail, empty for a read back.
 */
static void comm_command_motion_config(const char *args) {
  if (*args == '\0') { // Read back.
    printf("microsteps %u\r\n", controls_get_microsteps());
    printf("max_accel %.3f rad/s^2\r\n",
           (double)controls_get_max_accel_rad_s2());
    printf("max_vel %.3f rad/s\r\n", (double)controls_get_max_velocity_rad_s());
    return;
  }

  unsigned int microsteps = 0;
  float max_accel_rad_s2 = 0.0f;
  float max_velocity_rad_s = 0.0f;
  if (sscanf(args, "%u %f %f", &microsteps, &max_accel_rad_s2,
             &max_velocity_rad_s) != 3) {
    printf("Error: expected `motion <microsteps> <accel_rad_s2> "
           "<vel_rad_s>`\r\n");
    return;
  }

  // Zero microsteps would collapse steps_per_rad, and the TMC2209 tops out at
  // 1/256 microstepping.
  if (microsteps == 0 || microsteps > 256) {
    printf("Error: microsteps must be [1,256]\r\n");
    return;
  }
  if (!(max_accel_rad_s2 > 0.0f) || !(max_velocity_rad_s > 0.0f)) {
    printf("Error: accel and velocity must be > 0\r\n");
    return;
  }

  // Safety: changing the microstep resolution mid-move would glitch the step
  // stream, so reconfigure only while the driver is disabled.
  if (controls_state != CONTROLS_DISABLE) {
    printf("Error: disable the driver first, run `dis`\r\n");
    return;
  }

  // controls_set_microsteps() updates steps_per_rad first; the accel set then
  // re-derives the step-rate cap against the new calibration.
  controls_set_microsteps((uint16_t)microsteps);
  controls_set_max_accel_rad_s2(max_accel_rad_s2);
  controls_set_max_velocity_rad_s(max_velocity_rad_s);

  if (tmc2209_set_step_size((uint16_t)microsteps) != HAL_OK) {
    printf("Error: TMC2209 step size write failed\r\n");
    return;
  }

  comm_command_motion_config(""); // Echo the applied configuration.
}

/**
 * @brief Report or update the run/hold currents (`cur`).
 *
 * @param args Argument tail, empty for a read back.
 */
static void comm_command_current_config(const char *args) {
  if (*args == '\0') { // Read back.
    uint16_t run_current_ma = 0;
    uint8_t current_run = 0;
    uint8_t current_hold = 0;
    uint8_t current_hold_delay = 0;
    tmc2209_get_currents(&run_current_ma, &current_run, &current_hold,
                         &current_hold_delay);
    printf("run_current %u mA\r\n", run_current_ma);
    printf("irun %u\r\n", current_run);
    printf("ihold %u\r\n", current_hold);
    printf("iholddelay %u\r\n", current_hold_delay);
    return;
  }

  unsigned int run_current_ma = 0;
  unsigned int current_run = 0;
  unsigned int current_hold = 0;
  unsigned int current_hold_delay = 0;
  if (sscanf(args, "%u %u %u %u", &run_current_ma, &current_run, &current_hold,
             &current_hold_delay) != 4) {
    printf("Error: expected `cur <run_ma> <irun> <ihold> <iholddelay>`\r\n");
    return;
  }

  if (run_current_ma > UINT16_MAX) {
    printf("Error: run current must be [0,65535] mA\r\n");
    return;
  }
  if (current_run > 31 || current_hold > 31) {
    printf("Error: irun and ihold must be [0,31]\r\n");
    return;
  }
  if (current_hold_delay > 15) {
    printf("Error: iholddelay must be [0,15]\r\n");
    return;
  }

  // Safety: a current change mid-move would step the torque envelope under
  // load, so hold to the same disabled-only gate as the CAN path.
  if (controls_state != CONTROLS_DISABLE) {
    printf("Error: disable the driver first, run `dis`\r\n");
    return;
  }

  if (tmc2209_set_currents((uint16_t)run_current_ma, (uint8_t)current_run,
                           (uint8_t)current_hold,
                           (uint8_t)current_hold_delay) != HAL_OK) {
    printf("Error: TMC2209 current write failed\r\n");
    return;
  }

  comm_command_current_config(""); // Echo the applied configuration.
}

/**
 * @brief Report or update the position PID gains (`gains`).
 *
 * @param args Argument tail, empty for a read back.
 */
static void comm_command_gains(const char *args) {
  float k_p = 0.0f;
  float k_i = 0.0f;
  float k_d = 0.0f;

  if (*args == '\0') { // Read back.
    controls_get_gains(&k_p, &k_i, &k_d);
    printf("kp %.4f\r\n", (double)k_p);
    printf("ki %.4f\r\n", (double)k_i);
    printf("kd %.4f\r\n", (double)k_d);
    return;
  }

  if (sscanf(args, "%f %f %f", &k_p, &k_i, &k_d) != 3) {
    printf("Error: expected `gains <kp> <ki> <kd>`\r\n");
    return;
  }

  if (!isfinite(k_p) || !isfinite(k_i) || !isfinite(k_d)) {
    printf("Error: gains must be finite\r\n");
    return;
  }

  // Live tuning by design: controls_set_gains() does not reset the integrator
  // or derivative state, so large jumps land as a step input on a moving loop.
  controls_set_gains(k_p, k_i, k_d);

  comm_command_gains(""); // Echo the applied gains.
}

/**
 * @brief Report or update the StallGuard configuration (`sg`).
 *
 * @param args Argument tail, empty for a read back.
 */
static void comm_command_stallguard(const char *args) {
  if (*args == '\0') { // Read back.
    bool enable = false;
    bool stalled = false;
    uint8_t sgthrs = 0;
    uint16_t sg_result = 0;
    tmc2209_stallguard_get(&enable, &sgthrs, &sg_result, &stalled);
    printf("stallguard %u\r\n", (unsigned int)enable);
    printf("sgthrs %u\r\n", sgthrs);
    printf("sg_result %u\r\n", sg_result);
    printf("stalled %u\r\n", (unsigned int)stalled);
    return;
  }

  unsigned int enable = 0;
  unsigned int sgthrs = 0;
  if (sscanf(args, "%u %u", &enable, &sgthrs) != 2) {
    printf("Error: expected `sg <0|1> <sgthrs>`\r\n");
    return;
  }

  if (enable > 1) {
    printf("Error: enable must be 0 or 1\r\n");
    return;
  }
  if (sgthrs > 255) {
    printf("Error: sgthrs must be [0,255]\r\n");
    return;
  }

  // Register writes here are non-disruptive, so no standstill gate, matching
  // the CAN StallGuard path.
  if (tmc2209_stallguard_config((bool)enable, (uint8_t)sgthrs) != HAL_OK) {
    printf("Error: TMC2209 StallGuard write failed\r\n");
    return;
  }

  comm_command_stallguard(""); // Echo the applied configuration.
}

/**
 * @brief List the supported commands (`help`).
 */
static void comm_print_help(void) {
  printf("Motion:\r\n");
  printf("  pos <rad>       Absolute closed-loop position\r\n");
  printf("  rel <rad>       Relative closed-loop position\r\n");
  printf("  spd <rad/s>     Constant speed\r\n");
  printf("  ol <rad>        Absolute open-loop position\r\n");
  printf("  stop            Command zero speed, loop stays live\r\n");
  printf("  zero            Software zero at the current position\r\n");
  printf("Driver:\r\n");
  printf("  en | dis        Enable/disable the driver\r\n");
  printf("  clr             Clear a latched fault\r\n");
  printf("Config (omit arguments to read back):\r\n");
  printf("  motion [<microsteps> <accel_rad_s2> <vel_rad_s>]\r\n");
  printf("  cur [<run_ma> <irun> <ihold> <iholddelay>]\r\n");
  printf("  gains [<kp> <ki> <kd>]\r\n");
  printf("  sg [<0|1> <sgthrs>]\r\n");
  printf("Status:\r\n");
  printf("  stat            Control loop and driver status\r\n");
  printf("  ver | version   Firmware version\r\n");
  printf("  uid             48-bit UID hash parts\r\n");
}

/**
 * @brief Parse and execute one command line.
 *
 * Runs in the main context via comm_run(), never in interrupt context.
 *
 * @param line Null-terminated command line.
 */
static void comm_handle_line(const char *line) {
  const char *args = NULL;

  // Skip leading spaces.
  while (*line == ' ' || *line == '\t') {
    line++;
  }

  if (*line == '\0') {
    return; // Blank line.
  }

  /** Status. */

  if (comm_match(line, "help", NULL) || comm_match(line, "?", NULL)) {
    comm_print_help();
  }

  else if (comm_match(line, "version", NULL) || comm_match(line, "ver", NULL)) {
    printf("%s %u.%u.%u.%c\r\n", SCALPELSPACE_SHORT_NAME,
           MC_STEPPER_VERSION_MAJOR, MC_STEPPER_VERSION_MINOR,
           MC_STEPPER_VERSION_PATCH, MC_STEPPER_VERSION_IDENTIFIER);
  }

  else if (comm_match(line, "uid", NULL)) {
    uint16_t uid0 = 0;
    uint16_t uid1 = 0;
    uint16_t uid2 = 0;
    get_uid_hash48_parts(&uid0, &uid1, &uid2);
    printf("%u %u %u\r\n", uid0, uid1, uid2);
  }

  else if (comm_match(line, "stat", NULL)) {
    comm_print_status();
  }

  /** Motion. */

  else if (comm_match(line, "pos", &args)) {
    comm_command_motion(args, CONTROL_MODE_POSITION, "pos <rad>");
  }

  else if (comm_match(line, "rel", &args)) {
    comm_command_motion(args, CONTROL_MODE_POSITION_RELATIVE, "rel <rad>");
  }

  else if (comm_match(line, "spd", &args)) {
    comm_command_motion(args, CONTROL_MODE_SPEED, "spd <rad/s>");
  }

  else if (comm_match(line, "ol", &args)) {
    comm_command_motion(args, CONTROL_MODE_POSITION_OPEN_LOOP, "ol <rad>");
  }

  else if (comm_match(line, "stop", NULL)) {
    // Zero speed rather than a disable, so the slew limiter decelerates the
    // motor and the loop stays live for the next command.
    comm_command_motion("0", CONTROL_MODE_SPEED, "stop");
  }

  else if (comm_match(line, "zero", NULL)) {
    // Serviced on the next controls tick, and only at a standstill (DISABLE,
    // IDLE or ON_POSITION_TARGET). Dropped by controls in any other state, so
    // the reply reports the request rather than the outcome.
    controls_request_zero();
    printf("zero requested, applied at standstill\r\n");
  }

  /** Driver. */

  else if (comm_match(line, "en", NULL)) {
    comm_set_enable(true);
    printf("state %s\r\n", comm_state_name(controls_state));
  }

  else if (comm_match(line, "dis", NULL)) {
    comm_set_enable(false);
    printf("state %s\r\n", comm_state_name(controls_state));
  }

  else if (comm_match(line, "clr", NULL)) {
    if (controls_state == CONTROLS_FAULT) {
      controls_state = CONTROLS_IDLE;
    }
    printf("state %s\r\n", comm_state_name(controls_state));
  }

  /** Configuration. */

  else if (comm_match(line, "motion", &args)) {
    comm_command_motion_config(args);
  }

  else if (comm_match(line, "cur", &args)) {
    comm_command_current_config(args);
  }

  else if (comm_match(line, "gains", &args)) {
    comm_command_gains(args);
  }

  else if (comm_match(line, "sg", &args)) {
    comm_command_stallguard(args);
  }

  else { // Error: unknown command.
    printf("Error: unknown command, run `help`\r\n");
  }
}

/** User implementations into STM32 HAL (overwrite weak HAL functions). *******/

void HAL_UART_RxCpltCallback_comm(UART_HandleTypeDef *huart) {
  if (huart->Instance != COMM_UART_INSTANCE) {
    return;
  }

  uint8_t b = comm_rx_byte;

  HAL_UART_Receive_DMA(&COMM_HUART, &comm_rx_byte, 1);

  // Basic line editing/framing.
  if (b == '\r') {
    return; // Ignore CR.
  }

  if (b == '\n') {
    // End of line: null-terminate and publish to the main context.
    if (comm_line_len >= COMM_LINE_MAX)
      comm_line_len = COMM_LINE_MAX - 1;
    comm_line[comm_line_len] = '\0';

    if (comm_command_ready) {
      // The previous line has not executed yet. Dropping keeps the pending
      // buffer stable while comm_run() parses it.
      comm_command_dropped = true;
    } else {
      memcpy(comm_command, comm_line, (size_t)comm_line_len + 1u);
      comm_command_ready = true;
    }

    // Reset buffer.
    comm_line_len = 0;
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

/** Public functions. *********************************************************/

void comm_init(void) {
  comm_line_len = 0;
  comm_command_ready = false;
  comm_command_dropped = false;
  // Initialize single byte DMA.
  HAL_UART_Receive_DMA(&COMM_HUART, &comm_rx_byte, 1);
}

void comm_run(void) {
  if (comm_command_dropped) {
    comm_command_dropped = false;
    printf("Error: command dropped, previous line still executing\r\n");
  }

  if (!comm_command_ready) {
    return;
  }

  comm_handle_line(comm_command);

  // Cleared last: the interrupt only refills comm_command once this clears, so
  // the buffer stays stable for the whole of comm_handle_line().
  comm_command_ready = false;
}
