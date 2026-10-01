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
  * [v0.3.3 (2026-10-01)](#v033--2026-10-01-)
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

- Derive the position on-target hysteresis band from the microstep setting
  instead of fixing it at compile time.
    - Replace `POSITION_TOLERANCE_ENTER_RAD` and `POSITION_TOLERANCE_EXIT_RAD`
      with `POSITION_TOLERANCE_FLOOR_RAD`, `POSITION_TOLERANCE_MICROSTEPS` and
      `POSITION_TOLERANCE_EXIT_RATIO`, backwards compatibility loss.
    - The band is now
      `max(POSITION_TOLERANCE_FLOOR_RAD, POSITION_TOLERANCE_MICROSTEPS * microstep_rad)`,
      re-derived by `position_tolerance_update()` from
      `controls_init()` and `controls_set_microsteps()`.
    - The floor is the encoder limit, the AS5147P resolves `2*pi/16384` rad
      (0.38 mrad) with roughly 1 mrad of transition noise, so a tighter band is
      unachievable no matter how fine the microstepping.
    - The microstep term is the actuator limit, the rotor only rests on
      microstep boundaries so the standing error reaches half a microstep.
- Fix the on-target latch never closing at 1/4 microstepping and coarser.
    - At `step_size` 1, 2 and 4 the half-microstep standing error (15.7, 7.9 and
      3.9 mrad) exceeded the fixed 3 mrad entry band, so `on_target_ticks` reset
      every tick and `latched_on_target` never became true.
    - The loop stayed in `CONTROLS_EXECUTING` indefinitely, so the step stop in
      `CONTROLS_ON_POSITION_TARGET` never engaged and the motor hunted around
      the target. The status LED stayed on the moving colour and the CAN state
      message never reported `CONTROLS_ON_POSITION_TARGET`.
    - Both configuration paths accepted these settings without complaint, the
      comm `motion` command validates only `[1,256]` and the CAN
      `motion_config_set` handler only rejects `0`.
- Hold `step_size` 16 (the default) and finer to the previous 0.003 / 0.015 rad
  band, where the encoder floor governs. Only `step_size` 8 and coarser widen.
    - `step_size` 8 widens from 3 mrad to 5.9 mrad. It latched before, but with
      only 0.76 microsteps of band against a 0.5 microstep standing error, a
      1.5x margin. It now carries the same 3x margin as the default.
- Add `controls_get_position_tolerance()` and report the derived band in the
  comm `motion` read back, the value is no longer visible in `configuration.h`.
    - Update the `motion` command description in `README.md` accordingly.
- Fix the TMC2209 microstep resolution never actually changing.
    - `GCONF.mstep_reg_select` (bit 7) was never set, so the microstep
      resolution stayed under the control of the MS1/MS2 pins and every `MRES`
      write to `CHOPCONF` was accepted but ignored by the sequencer. Both pins
      are strapped to GND, which selects 8 microsteps, so the driver ran at 1/8
      regardless of the configured `step_size`.
    - Add `tmc2209_set_uart_control()`, called from `tmc2209_init()` before the
      first `tmc2209_set_microsteps()`.
    - Every angular unit was scaled by `step_size / 8` as a result. At the
      default `MC_STEPPER_STEP_SIZE` of 16 the motor ran at twice the commanded
      velocity and acceleration, and `CONTROL_MODE_POSITION_OPEN_LOOP` travelled
      twice the requested angle. `CONTROL_MODE_POSITION` still converged to the
      correct angle, the encoder measures real radians, but carried twice the
      intended loop gain.
    - Existing PID gains were tuned against that doubled gain and will need
      retuning.
- Fix the `GCONF` bit map, `internal_Rsense` is bit 1 and not bit 6.
    - Bit 6 is `pdn_disable`, which the datasheet requires set whenever the UART
      interface is used. `tmc2209_init()` calls
      `tmc2209_set_current_sense(false, ...)`, so it was actively clearing that
      bit and leaving `PDN_UART` on its standstill current reduction function.
    - `tmc2209_set_uart_control()` now sets `pdn_disable`, and
      `tmc2209_set_current_sense()` acts on the correct `internal_Rsense` bit.
      The argument previously had no effect on the intended register field.
    - Bit definitions annotated against datasheet rev 1.08 section 5.1.
- Seed `controls.c` `microstep_config` from `MC_STEPPER_STEP_SIZE` instead of a
  hardcoded `16`.
    - The control loop and `tmc2209_init()` held independent copies of the
      microstep setting that only agreed at the default. Changing
      `MC_STEPPER_STEP_SIZE` in `configuration.h` desynchronized them and scaled
      every angular unit by the ratio. The runtime paths (comm `motion` and CAN
      `motion_config_set`) were unaffected, they update both together.
