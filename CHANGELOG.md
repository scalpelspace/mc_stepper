# Changelog

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [Changelog](#changelog)
  * [v0.1.0 (2026-06-30)](#v010--2026-06-30-)
  * [v0.1.2 (2026-07-04)](#v012--2026-07-04-)
  * [v0.2.0 (2026-07-06)](#v020--2026-07-06-)
<!-- TOC -->

</details>

---

## [v0.1.0 (2026-06-30)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.1.0)

- Initial release.

---

## [v0.1.2 (2026-07-04)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.1.2)

- **Modifications:**
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

- **Modifications:**
    - Update for `mc_stepper_driver` tagged release v0.2.0.
        - Update `can_send_message_raw32()` for muxed signal CAN transmit.
        - Update `can_rx_command_stepper()` for muxed signal CAN recive.
    - Implement `RGB_LED_SET` CAN receive handler.
    - Remove use of unused `as5047p_angle_deg` from the AS5047P driver.
