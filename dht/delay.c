#include <stm32f10x_rcc.h>
#include <delay.h>


/** SysTick timer interrupt (overflow) frequency in Hz */
#define SYSTICK_INTERRUPT_FREQUENCY             100000
/** Setup SysTick Timer (24 bit) for 0.125 s interrupts (4 Hz blink signal)
 *  72 MHz / DIV = 8 Hz ==> DIV = 9000000 */
#define SYSTICK_RELOAD_VALUE                    (72000000 / SYSTICK_INTERRUPT_FREQUENCY)


static volatile uint32_t dSysTickCntr;


// SysTick interrupt handler
void SysTick_Handler() {
	dSysTickCntr++;
}

void dTimerInit(void) {
	SysTick->CTRL |= SysTick_CLKSource_HCLK; // AHB clock as SysTick clock source
//	SysTick_Config(SystemCoreClock / DELAY_TICK_FREQUENCY_US); // SysTick count microseconds
	SysTick_Config(SYSTICK_RELOAD_VALUE);
	dSysTickCntr = 0;
}

void dTimerReset(void) {
	uint32_t int_status;

	// Save status and disable global interrupts
	int_status = __get_PRIMASK();
    __set_PRIMASK(int_status | 0x01);

    // Reset counters
    SysTick->VAL = 0;
    dSysTickCntr = 0;

    // Restore global interrupts status
    __set_PRIMASK(int_status & 0x01);
}

uint32_t dTimerGet_ms(void) {
	return dSysTickCntr/(SYSTICK_INTERRUPT_FREQUENCY/1000);
}

uint32_t dTimerGet_us(void) {
	return dSysTickCntr*(1000000/SYSTICK_INTERRUPT_FREQUENCY);
}

void Delay_ms(uint32_t nTime) {
	uint32_t ms = dTimerGet_ms();

	while ((dTimerGet_ms() - ms) < nTime) __NOP();
}

void Delay_us(uint32_t nTime) {
	uint32_t us = dTimerGet_us();

	while ((dTimerGet_us() - us) < nTime) __NOP();
}
