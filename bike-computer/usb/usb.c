#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_exti.h>

#include <usb.h>


// USB wakeup interrupt handler
void USB_FS_WKUP_IRQHandler(void) {
	EXTI_ClearITPendingBit(EXTI_Line18);
}


// Initialize and configure the USB peripheral
void USB_Init() {
	GPIO_InitTypeDef PORT;
	EXTI_InitTypeDef EXTIInit;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // Enable the PORTA peripheral
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); // Enable the SYSCFG peripheral
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB,ENABLE); // Enable the USB peripheral

	// Set PA11,PA12 as In - USB_DM,USB_DP
	PORT.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12;
	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA,&PORT);

	// Set PA11,PA12 alternative function for USB
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_USB);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_USB);

	EXTI_ClearITPendingBit(EXTI_Line18);
	EXTIInit.EXTI_Line = EXTI_Line18;
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTIInit);
}
