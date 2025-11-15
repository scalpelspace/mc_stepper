# mc_stepper

![arm_gcc_build](https://github.com/scalpelspace/mc_stepper/actions/workflows/arm_gcc_build.yaml/badge.svg)

STM32L432KC microcontroller firmware for `mc_stepper_pcb`.

- CAN bus and UART enabled stepper motor.
- Closed-loop position and speed control.

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

| Manufacturer Part Number | Manufacturer                         | Description             | Quantity | Notes |
|--------------------------|--------------------------------------|-------------------------|---------:|-------|
| STM32L432KC              | STMicroelectronics                   | 32-bit MCU              |        1 |       |
| TJA1057BTK               | NXP USA Inc.                         | CAN Bus Transceiver     |        1 |       |
| AS5147P                  | ams-OSRAM USA INC.                   | Magnetic Rotary Encoder |        1 |       |
| TMC2209                  | Analog Devices Inc./Maxim Integrated | Bipolar Motor Driver    |        1 |       |
| WS2812B                  | (Various)                            | PWM Addressable RGB LED |        1 |       |

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

| STM32L432KC | Peripheral       | Config                    | Connection                 | Notes                                 |
|-------------|------------------|---------------------------|----------------------------|---------------------------------------|
| PA14        | `SYS_JTCK-SWCLK` |                           | TC2050 SWD Pin 4: `SWCLK`  |                                       |
| PA13        | `SYS_JTMS-SWDIO` |                           | TC2050 SWD Pin 2: `SWDIO`  |                                       |
| PA10        | `USART1_RX`      | 115200 bps                | `TX`                       |                                       |
| PA9         | `USART1_TX`      | 115200 bps                | `RX`                       |                                       |
| PA11        | `CAN1_RX`        |                           | TJA1057BTK Pin 1: `TXD`    |                                       |
| PA12        | `CAN1_TX`        |                           | TJA1057BTK Pin 4: `RXD`    |                                       |
| PA1         | `TIM2_CH1`       |                           | AS5147P Pin 1: `CSn`       |                                       |
| PA5         | `TIM2_CH2`       |                           | AS5147P Pin 2: `CLK`       |                                       |
| PA4         | `GPIO_EXTI4`     |                           | AS5147P Pin 3: `MISO`      |                                       |
| PA4         | `GPIO_EXTI4`     |                           | AS5147P Pin 4: `MOSI`      |                                       |
| PA1         | `TIM2_CH1`       | TIM2 encoder mode         | AS5147P Pin 7: `A`         |                                       |
| PA5         | `TIM2_CH2`       | TIM2 encoder mode         | AS5147P Pin 6: `B`         |                                       |
| PA4         | `GPIO_EXTI4`     | Encoder index             | AS5147P Pin 14: `I`        |                                       |
| P?? TODO ?? | `GPIO_Output`    |                           | TMC2209 Pin 2: `ENN`       |                                       |
| PA2         | `USART2_TX`      | Single Wire (Half-Duplex) | TMC2209 Pin 14: `PDN_UART` |                                       |
| PA3         | `TIM15_CH2`      | PWM Generation CH2        | TMC2209 Pin 16: `STEP`     |                                       |
| P?? TODO ?? | `GPIO_Output`    |                           | TMC2209 Pin 19: `DIR`      |                                       |
| PA8         | `TIM1_CH1`       | PWM Generation CH1        | WS2812B Pin: `DIN`         | DIN pin number depends on IC variant. |

</details>

### 1.4 Clock Configurations

```
8 MHz High Speed External (HSE)
 -> Phase-Locked Loop Main (PLLM)
 -> 80 MHz SYSCLK
 -> 80 MHz HCLK
     -> 80 MHz APB1 (Maxed) -> 80 MHz APB1 Timer
     -> 80 MHz APB2 (Maxed) -> 80 MHz APB2 Timer
```

---

## Third-Party Licenses

This project uses the following open-source software components:

- **STM32Cube HAL**, STMicroelectronics.
    - Licensed under the `3-Clause BSD License`.
        - See [`LICENSE.txt`](Drivers/STM32L4xx_HAL_Driver/LICENSE.txt).

> STMicroelectronics are trademarks of their respective owners. Use of these
> names does **not** imply any endorsement by the trademark holders.
