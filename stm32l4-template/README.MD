# Project template for the **STM32L476RG** MCU.

**Used:**
- ARM CMSIS library for Cortex-M4 (v4.30)
- STM32L4xx startup, headers and vector table (v1.1.2)

**Only minimal system initialization.**

**An sample program (main.c) will do such things:**
- The main system clock initialization procedure expects an external 16MHz crystal and configures main CPU clock to 80MHz using PLL. If initialization of HSE fails (e.g. on Nucleo board by default external crystal is not populated), the procedure will try to configure main clock to use internal HSI oscillator at 16MHz, in that case the main CPU clock also will be 80MHz (by using PLL).
- Then, when the main system clocks has been configured, the USART2 port will be initialized and configured to transmit-only mode at 115200 baud rate (which is convenient for use on Nucleo board).
- After, the actual values of the clocks will be sent into USART2.
- And finally does the classics: flashing the green LED on Nucleo board (PA5).

The **periph** directory contains some libraries for the onboard peripherals and various helper functions.
