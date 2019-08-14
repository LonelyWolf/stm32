# Pure C implementation of device startup for STM32L476xx (based on CMSIS v5 templates)

### Trivia
The LTO doesn't work in the GCC versions since 7.x.x (and up to 9.2.0). It discards the IRQ functions due they are defined as 'weak' in the assembler startup.

As example: the TIM7 interrupt is used in application. Somewhere in it the interrupt handler it is implemented as follows:
```
void TIM7_IRQHandler(void) {
	bla bla bla
}
```
The prototype of this handler is defined in the startup_stm32l476xx.S with the `weak` attribute:
```
	.weak	TIM7_IRQHandler
	.thumb_set TIM7_IRQHandler,Default_Handler
```
At linking stage the LTO will discard the TIM7_IRQHandler() function from application. That issue can be solved by removing prototype definition from the startup_stm32l476xx.S, that looks inconvenient.

### Task
Create a device startup file in pure C instead of an assembler implementation provided by ST.

### Result
Far from perfect, but working. The firmware grows by about 400 bytes compared to assembler implementation (measured on this trivial example, on more decent projects the working LTO will noticeably reduce the firmware).

### How to use
- Remove the startup_stm32l476xx.S
- Add the startup_stm32l476xx.c and LinkerScript.ld from this project

### Project
The TIM7 is configured to trigger twice per second, its interrupt enabled. In the interrupt handler, the status of the Nucleo user LED is inverted. With the assembler startup file and broken LTO, the LED would not blink...
