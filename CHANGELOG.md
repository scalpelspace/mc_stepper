# Changelog

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [Changelog](#changelog)
  * [v0.1.0 (2026-06-30)](#v010--2026-06-30-)
  * [v0.1.2 (2026-07-04)](#v012--2026-07-04-)
  * [v0.2.0 (2026-07-06)](#v020--2026-07-06-)
  * [v0.2.1 (2026-07-08)](#v021--2026-07-08-)
  * [v0.2.2 (2026-07-10)](#v022--2026-07-10-)
  * [v0.2.3 (2026-08-02)](#v023--2026-08-02-)
  * [v0.2.4 (2026-08-16)](#v024--2026-08-16-)
  * [v0.3.0 (2026-09-01)](#v030--2026-09-01-)
  * [v0.3.1 (2026-09-06)](#v031--2026-09-06-)
  * [v0.4.0 (2026-10-01)](#v040--2026-10-01-)
  * [v0.4.1 (2026-10-06)](#v041--2026-10-06-)
  * [v0.4.2 (2026-10-20)](#v042--2026-10-20-)
<!-- TOC -->

</details>

---

## [v0.1.0 (2026-06-30)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.1.0)

- Initial release.

---

## [v0.1.2 (2026-07-04)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.1.2)

- Guard the shared CAN TX header/mailbox with interrupt disable to prevent
  ISR/main-loop races.
- Change DLC mismatch handling to skip only that entry instead of aborting the
  whole dispatch loop.
- Update for `mc_stepper_driver` tagged release v0.1.0.
    - Implement TMC2209 status reporting in state message.
- Refactor CAN telemetry transmit to use const pointer message reference and
  direct signal initialization.
- Reduce TMC2209 UART timeout from 10 ms to 2 ms.

---

## [v0.2.0 (2026-07-06)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.2.0)

- Update for `mc_stepper_driver` tagged release v0.2.0.
    - Update `can_send_message_raw32()` for muxed signal CAN transmit.
    - Update `can_rx_command_stepper()` for muxed signal CAN recive.
    - Implement `RGB_LED_SET` CAN message receive handler.
    - Implement `SENSOR` CAN message transmit with scheduler task.
- Remove use of unused `as5047p_angle_deg` from the AS5047P driver.
- Adjust controls behaviour where `CONTROL_MODE_NONE` no longer commands a stop,
  allowing CAN frames to clear faults/enable without a new setpoint.

---

## [v0.2.1 (2026-07-08)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.2.1)

- Cleanup `CHANGELOG.md` for formatting and syntax consistency.
- Enable `AutoBusOff` and `AutoRetransmission` via CubeMX.
- Update `mc_stepper_driver` to tagged release v0.2.1.

---

## [v0.2.2 (2026-07-10)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.2.2)

- Update `mc_stepper_driver` to tagged version v0.2.2.

---

## [v0.2.3 (2026-08-02)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.2.3)

- Cleanup peripheral checking to compare `->Instance` instead of handle pointers
  in HAL callbacks.
    - Match callbacks to their peripheral by register instance (via new
      `*_INSTANCE` macros) rather than by handle address, and normalize all
      checkers to the early-return guard-clause form.
- Update table of contents in `CHANGELOG.md` (outdated on previous release).
- Fix WS2812B PWM DMA memory data width to match the `uint16_t` DMA buffer.
    - Previously was BYTE, corrected to HALFWORD.

---

## [v0.2.4 (2026-08-16)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.2.4)

- Rename `ALLOW_CAN_NODE_ID_REASSIGNMENT` to `ALLOW_CAN_NODE_ID_ALLOCATION`, now
  acts as a single switch for the CAN ID allocation protocol.
    - Undefined: the node never participates in allocation and operates on
      `DEFAULT_CAN_NODE_ID` for its lifetime. The allocatee scheduler task and
      the allocatee receive callbacks are compiled out.
    - Defined: unchanged behaviour, the allocatee runs and restarts after every
      assignment.
- Support pre-compile time CAN Node IDs via `DEFAULT_CAN_NODE_ID`.
    - A non-zero `DEFAULT_CAN_NODE_ID` is now applied to the DBC message IDs in
      `can_db_init()`. Previously the value only set `can_node_id` and the DBC
      was left unpatched, so the node transmitted under the authored ID.
    - Add a compile time range check rejecting IDs outside `[0, 30]`.

---

## [v0.3.0 (2026-09-01)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.3.0)

- Update `mc_stepper_driver` to tagged version v0.3.0.
- Carry the `ALLOW_CAN_NODE_ID_ALLOCATION` configuration forward onto the new
  `can_alloc_mode_t` advertised by the allocatee.
    - Undefined now maps to `CAN_ALLOC_MODE_NOT_REASSIGNABLE` instead of
      compiling the protocol out. The node still advertises, so the allocator
      reserves `DEFAULT_CAN_NODE_ID` rather than handing it to another node.
      Assignments are refused inside the allocatee.
    - Defined maps to `CAN_ALLOC_MODE_REASSIGNABLE`, unchanged behaviour.
    - The allocatee scheduler task and the allocatee receive callbacks are no
      longer compiled out, they are required in both modes to answer DISCOVER.
    - Add a compile time check rejecting `DEFAULT_CAN_NODE_ID` of `0` while
      `ALLOW_CAN_NODE_ID_ALLOCATION` is undefined, a node that refuses
      reassignment has to hold a real Node ID to advertise.
- Seed `allocatee_config_t::node_id` with the Node ID currently held rather than
  `DEFAULT_CAN_NODE_ID`, so a restarted allocatee re-advertises the ID it was
  assigned.
- Remove the allocatee restart from `allocatee_complete()`.
    - The allocatee now returns to awaiting discovery on its own after ACKing.
      Restarting reset the held Node ID back to the configured seed, so the node
      would advertise as unassigned on the next session.
- Update `arm_gcc_build.yaml` workflow file to run on submodule updates.

---

## [v0.3.1 (2026-09-06)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.3.1)

- Expand the `comm` UART interface to drive the stepper in every control scheme,
  reaching parity with the CAN command surface.
    - Motion: `pos <rad>`, `rel <rad>`, `spd <rad/s>`, `ol <rad>`, `stop` and
      `zero`.
    - Driver: `en`, `dis` and `clr`, mirroring the enable and fault-clear gates
      of the CAN `command_stepper` handler.
    - Configuration, reporting on no arguments and applying on a full argument
      set: `motion`, `cur`, `gains` and `sg`.
    - Status: `stat` (control state, mode, position, setpoints, step rate and
      the packed TMC2209 status word) and `help`.
    - Commands rejected by a state gate now report the reason instead of being
      silently dropped.
- Defer command execution from the RX interrupt to the new `comm_run()`
  scheduler task.
    - `printf()` transmits over blocking `HAL_UART_Transmit()`, and the TMC2209
      configuration writes are blocking UART transfers. Both ran in interrupt
      context and would overrun the 1 ms `controls_run()` tick. The interrupt
      now only frames a line and publishes it.
    - A line received while one is still pending is dropped and reported, rather
      than overwriting the buffer mid-parse.
- Fix command matching to compare whole words.
    - `strncmp(line, "sr", 1)` compared a single character, so every command
      beginning with `s` dispatched to `sr`.
- Remove `sr <step_rate>`, superseded by `spd <rad/s>`.
    - It wrote the step generator directly, but `controls_run()` rewrites the
      step frequency from `controls_step_rate_command` every tick, so the
      commanded rate was overwritten within 1 ms and the motor never moved.
- Refactor structure to merge repeating continuous telemetry into the unified
  `telemetry` module.
    - Add `MC_STEPPER_FULL_COMM_TELEMETRY` macro (disabled by default) for
      general UART (comm) telemetry, transmitting a combined controller status
      similar to existing CAN telemetry.

---

## [v0.4.0 (2026-10-01)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.4.0)

- Fix the TMC2209 microstep resolution never actually changing.
    - `GCONF.mstep_reg_select` was never set, so the resolution stayed under the
      MS1/MS2 pins and every `MRES` write was ignored. Both pins are strapped to
      GND, selecting 1/8 regardless of the configured `step_size`.
    - Add `tmc2209_set_uart_control()`, called from `tmc2209_init()` before the
      first `tmc2209_set_microsteps()`.
    - Every angular unit was scaled by `step_size / 8`. At the default 16 the
      motor ran at twice the commanded velocity and acceleration, and
      `CONTROL_MODE_POSITION_OPEN_LOOP` travelled twice the requested angle.
    - Existing PID gains were tuned against the doubled loop gain and will need
      retuning.
- Fix the `GCONF` bit map, `internal_Rsense` is bit 1 and not bit 6.
    - Bit 6 is `pdn_disable`, which the datasheet requires to be set for UART
      operation. `tmc2209_init()` was actively clearing it via
      `tmc2209_set_current_sense(false, ...)`.
    - `tmc2209_set_uart_control()` now sets `pdn_disable` and
      `tmc2209_set_current_sense()` acts on the correct bit. Bits annotated
      against the datasheet.
- Seed `controls.c` `microstep_config` from `MC_STEPPER_STEP_SIZE` instead of a
  hardcoded `16`.
    - The control loop and `tmc2209_init()` held independent copies that only
      agreed at the default. Changing the macro desynchronized them and scaled
      every angular unit by the ratio. The runtime reconfiguration paths were
      unaffected, they update both together.
- Derive the position on-target hysteresis band from the microstep setting
  instead of fixing it at compile time.
    - Replace `POSITION_TOLERANCE_ENTER_RAD` and `POSITION_TOLERANCE_EXIT_RAD`
      with `POSITION_TOLERANCE_FLOOR_RAD`, `POSITION_TOLERANCE_MICROSTEPS` and
      `POSITION_TOLERANCE_EXIT_RATIO`, backwards compatibility loss.
    - The band is the larger of an encoder noise floor and the microstep
      standing error, re-derived by `position_tolerance_update()` whenever the
      microstep configuration changes.
    - Fixes the on-target latch never closing at `step_size` 4 and coarser,
      where half a microstep exceeded the fixed 3 mrad entry band. The loop
      stayed in `CONTROLS_EXECUTING` and hunted around the target indefinitely.
    - `step_size` 16 and finer keep the previous 0.003 / 0.015 rad band, 8 and
      coarser widen.
    - Add `controls_get_position_tolerance()` and report the band in the comm
      `motion` read back, it is no longer visible in `configuration.h`. Update
      the `motion` command description in `README.md` accordingly.
- Fix the commanded step rate being truncated to an integer hertz.
    - `tmc2209_step_set_freq_hz()` renamed to `tmc2209_step_set_rate_hz()` and
      now takes a `float`, rounding the compare period once from the fractional
      rate, backwards compatibility loss.
    - Worst case error at 1/16 microstepping improves from 1.8% to under 0.1%.
- Refuse step rates below the generator floor instead of silently clamping.
    - The 16-bit compare register cannot produce below
      `TMC2209_STEP_MIN_RATE_HZ` (7.63 steps/s). Clamping ran the motor faster
      than commanded, by up to 12x at full stepping. The generator now stops and
      returns `HAL_ERROR`.
    - Add `TMC2209_STEP_MIN_RATE_HZ` and `TMC2209_STEP_MAX_RATE_HZ`, derived
      from `TMC2209_TIM_CLOCK_HZ`, which moves to `tmc2209_runner.h`.
    - Add `controls_get_min_velocity_rad_s()` and reject a `spd` command below
      the floor, rather than leaving a silently motionless motor.

---

## [v0.4.1 (2026-10-06)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.4.1)

- Fix re-enabling with the previous target leaving the controller `IDLE`.
    - `controls_set_command()` dropped any command matching the last accepted
      one, but disabling (or clearing a fault) left that command in place. The
      driver re-energized while the setpoint was discarded as a duplicate, so
      nothing held position and an external load could move the shaft.
    - The duplicate check now only applies while the loop is tracking
      (`CONTROLS_EXECUTING` or `CONTROLS_ON_POSITION_TARGET`), where it still
      stops heartbeat retransmissions restarting the state machine.
    - The issue previously affected every control mode and both the CAN and comm
      interfaces.
        - For example `spd 2`, `dis`, `en`, `spd 2` left the motor stationary.

---

## [v0.4.2 (2026-10-20)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.4.2)

- Restore the comm `rgb` command, `rgb <R>, <G>, <B>` with each channel
  `[0,255]`, and list it in `help`.
- Cleanup the comm `help` command print.
- Add `LED_EXTERNAL_CONTROL_ONLY` to `configuration.h` (undefined by default).
    - When defined, the firmware no longer drives the on-board WS2812B, the boot
      colour and the `led_status_run()` control state indication are compiled
      out.
    - The LED is then only set externally, via the CAN `rgb_led_set` message or
      the comm `rgb` command.
- Clean up documentation.
