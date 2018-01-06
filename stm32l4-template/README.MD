# Project template for the **STM32L476RG** MCU.

**Used:**
- ARM CMSIS library for Cortex-M4 (v5.2.0)
- STM32L4xx startup, headers and vector table (v1.3.2)

**Only minimal system initialization.**

**A sample program (main.c), do the following things:**
- The procedure of main system clock initialization expects an external 16MHz crystal and configures main CPU clock to 80MHz using PLL. If HSE initialization fails (e.g. on the Nucleo board an external crystal is not populated out from the box), the procedure will try to configure the main clock to 80MHz using PLL clocked from internal HSI oscillator at 16MHz.
- Then, when the main system clocks were been configured, the USART2 port will be initialized and configured in transmit-only mode at 115200 baud rate (which is convenient for use on Nucleo board).
- After, the actual values of the clocks will be sent into USART2.
- And finally does the classics: flashing the green LED on Nucleo board (PA5).

The **periph** directory contains various libraries for the onboard peripherals and various helper functions.
