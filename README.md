# mc_stepper

![arm_gcc_build](https://github.com/scalpelspace/mc_stepper/actions/workflows/arm_gcc_build.yaml/badge.svg)

STM32G431RB microcontroller firmware for `mc_stepper_pcb`.

- CAN bus and UART enabled stepper motor closed loop controller.

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [mc_stepper](#mc_stepper)
  * [1 Overview](#1-overview)
    * [1.1 Bill of Materials (BOM)](#11-bill-of-materials-bom)
    * [1.2 Block Diagram](#12-block-diagram)
    * [1.3 Pin Configurations](#13-pin-configurations)
    * [1.4 Clock Configurations](#14-clock-configurations)
  * [Third-Party Licenses](#third-party-licenses)
<!-- TOC -->

</details>

---

## 1 Overview

### 1.1 Bill of Materials (BOM)

| Manufacturer Part Number | Manufacturer       | Description | Quantity | Notes |
|--------------------------|--------------------|-------------|---------:|-------|
| STM32G431RB              | STMicroelectronics | 32-bit MCU  |        1 |       |

### 1.2 Block Diagram

![mc_stepper.drawio.png](docs/mc_stepper.drawio.png)

> Drawio file here: [mc_stepper.drawio](docs/mc_stepper.drawio).

### 1.3 Pin Configurations

<details markdown="1">
  <summary>CubeMX Pinout</summary>

![CubeMX Pinout.png](docs/CubeMX%20Pinout.png)

</details>

<details markdown="1">
  <summary>Pin & Peripherals Table</summary>

| STM32G431RB | Peripheral       | Config | Connection                | Notes |
|-------------|------------------|--------|---------------------------|-------|
| PB3         | `SYS_JTDO-SWO`   |        | TC2050 SWD Pin 6: `SWO`   |       |
| PA14        | `SYS_JTCK-SWCLK` |        | TC2050 SWD Pin 4: `SWCLK` |       |
| PA13        | `SYS_JTMS-SWDIO` |        | TC2050 SWD Pin 2: `SWDIO` |       |

</details>

### 1.4 Clock Configurations

```
```

---

## Third-Party Licenses

This project uses the following open-source software components:

- **STM32Cube HAL**, STMicroelectronics.
    - Licensed under the `3-Clause BSD License`.
        - See [`LICENSE.txt`](Drivers/STM32L4xx_HAL_Driver/LICENSE.txt).

> STMicroelectronics are trademarks of their respective owners. Use of these
> names does **not** imply any endorsement by the trademark holders.
