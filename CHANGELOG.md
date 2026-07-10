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
<!-- TOC -->

</details>

---

## [v0.1.0 (2026-06-30)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.1.0)

- Initial release.

---

## [v0.1.2 (2026-07-04)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.1.2)

- Guard the shared CAN TX header/mailbox with interrupt disable to prevent
  ISR/main-loop races.
- Change DLC mismatch handling to skip only that entry instead of aborting
  the whole dispatch loop.
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
- Adjust controls behaviour where `CONTROL_MODE_NONE` no longer commands a
  stop, allowing CAN frames to clear faults/enable without a new setpoint.

---

## [v0.2.1 (2026-07-08)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.2.1)

- Cleanup `CHANGELOG.md` for formatting and syntax consistency.
- Enable `AutoBusOff` and `AutoRetransmission` via CubeMX.
- Update `mc_stepper_driver` to tagged release v0.2.1.

---

## [v0.2.2 (2026-07-10)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.2.2)

- Update `mc_stepper_driver` to tagged version v0.2.2.
