# Changelog

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [Changelog](#changelog)
  * [v0.1.0 (2026-06-30)](#v010--2026-06-30-)
  * [v0.1.2 (TBD)](#v012--tbd-)
<!-- TOC -->

</details>

---

## [v0.1.0 (2026-06-30)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.1.0)

- Initial release.

---

## [v0.1.2 (TBD)](https://github.com/scalpelspace/mc_stepper/releases/tag/v0.1.2)

- **Modifications:**
    - Guard the shared CAN TX header/mailbox with interrupt disable to prevent
      ISR/main-loop races.
    - Change DLC mismatch handling to skip only that entry instead of aborting
      the whole dispatch loop.
    - Implement TMC2209 status reporting in state message.
