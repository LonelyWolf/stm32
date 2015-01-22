///////////////////
// STM32L151RBT6 //
///////////////////


// SPL
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_syscfg.h>
//#include <stm32l1xx_tim.h>
#include <misc.h>

// USB related stuff
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"


#include "pipka.h"
#include "delay.h"
#include "uart.h"


GPIO_InitTypeDef PORT;
NVIC_InitTypeDef NVICInit;

// ---> usb_endp.c
extern uint8_t packet_send;
// <--- usb_endp.c


#define ADC_BUF_SIZE    1024 // ADC buffer size
#define OVS_COUNT       32   // Oversampling count


uint8_t ADC_buffer[ADC_BUF_SIZE];

uint32_t buf_pos;
uint32_t ADC_buf_pos;
uint32_t USB_buf_pos;
uint32_t i;
uint32_t ovs_accu;
uint32_t ovs_cntr;


void TIM9_IRQHandler(void) {
	union {
		int16_t PCM16;
		struct BPCM16 {
			uint8_t B1;
			uint8_t B0;
		} BPCM16;
	} PCM_val;
	uint16_t ADC_val;

	if (TIM9->SR & TIM_SR_UIF) {
		TIM9->SR &= ~TIM_SR_UIF; // Clear the TIMx interrupt pending bit

//		while (!(ADC1->SR & ADC_SR_EOC)); // Wait until ADC conversion end
		if (ADC1->SR & ADC_SR_EOC) ADC_val = ADC1->DR; else ADC_val = 0x0800; // Get ADC value

		ovs_accu += ADC_val;
		ovs_cntr++;
		if (ovs_cntr >= OVS_COUNT) {
			ADC_val = ovs_accu / ovs_cntr;
			ovs_accu = 0;
			ovs_cntr = 0;

//			PCM_val.PCM16 = (ADC_val * 16) - 32768; // Convert ADC value to PCM
			PCM_val.PCM16 = (ADC_val * 16) - 32768 - 60; // Convert ADC value to PCM
//			PCM_val.PCM16 = (ADC_val * 16) - 32768 - 340; // Convert ADC value to PCM (340 - dummy bias offset)
			ADC_buffer[ADC_buf_pos++] = PCM_val.BPCM16.B1;
			ADC_buffer[ADC_buf_pos++] = PCM_val.BPCM16.B0;
			if (ADC_buf_pos > ADC_BUF_SIZE - 1) ADC_buf_pos = 0;
		}

/*
		// Put test sample values
		ADC_buffer[ADC_buf_pos++] = pipka[buf_pos++];
		if (buf_pos > sizeof(pipka) - 1) buf_pos = 0;
		ADC_buffer[ADC_buf_pos++] = pipka[buf_pos++];
		if (buf_pos > sizeof(pipka) - 1) buf_pos = 0;
		if (ADC_buf_pos > ADC_BUF_SIZE - 1) ADC_buf_pos = 0;
*/

		ADC1->CR2 |= ADC_CR2_SWSTART; // Software ADC start
	}
}


int main(void) {
	// Initialize delay without callback
	Delay_Init(NULL);

	// Initialize UART2
	UARTx_Init(USART2,1382400);
	UART_SendStr(USART2,"--------------------------------\n");

	// Enable the PORTA peripheral
	RCC->AHBENR |= RCC_AHBPeriph_GPIOA;

	// Configure PA4 as ADC_IN4
	GPIOA->MODER   |=  GPIO_MODER_MODER4; // Analog mode
	GPIOA->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR4; // High speed
	GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPDR4; // No pull-up, pull-down
	GPIOA->OTYPER  &= ~GPIO_OTYPER_OT_4; // Output push-pull (reset state)

	// Initialize the HSI clock
	RCC->CR |= RCC_CR_HSION; // Enable HSI
	while (!(RCC->CR & RCC_CR_HSIRDY)); // Wait until HSI stable

	// Initialize the ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC1 peripheral clock
	ADC->CCR = 0; // Disable temperature sensor, Vrefint, ADC prescaler = HSI/1
	ADC1->SQR5 |= ADC_SQR5_SQ1_2; // 1st conversion in regular sequence will be from ADC_IN4
	ADC1->CR1 &= ~ADC_CR1_RES; // 12-bit resolution (Tconv = 12 ADCCLK cycles)
//	ADC1->CR1 |= ADC_CR1_RES_0; // 10-bit resolution (Tconv = 11 ADCCLK cycles)
//	ADC1->CR1 |= ADC_CR1_RES_1; // 8-bit resolution (Tconv =  9 ADCCLK cycles)
	ADC1->CR2 &= ~ADC_CR2_ALIGN; // Right alignment
//	ADC1->SMPR3 &= ~ADC_SMPR3_SMP4; // Channel4 sample rate: 4 cycles
	ADC1->SMPR3 |= ADC_SMPR3_SMP4; // Channel4 sample rate: 384 cycles
	ADC1->CR2 |= ADC_CR2_ADON; // Enable the ADC
	while (!(ADC1->SR & ADC_SR_ADONS)); // Wait until ADC is on

	// Configure TIM9 (trigger for ADC)
	RCC->APB2ENR |= RCC_APB2Periph_TIM9; // Enable the TIMx peripheral
	TIM9->ARR   =  SystemCoreClock / (8000 * OVS_COUNT); // Sample rate 8kHz
	TIM9->CR2  &= ~TIM_CR2_MMS; // Master mode selection reset
	TIM9->CR2  |=  TIM_CR2_MMS_1; // The update event is selected as trigger output (TRGO)
	TIM9->DIER |=  TIM_DIER_UIE; // ÒIMx update interrupt enable
	TIM9->EGR   =  TIM_EGR_UG; // Generate an update event to reload the prescaler value immediately

	// NVIC interrupt priority group 2
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	// Enable the TIM9 Interrupt
	NVICInit.NVIC_IRQChannel = TIM9_IRQn;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 2;
	NVICInit.NVIC_IRQChannelSubPriority = 0;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInit);

/*
	while (1) {
		ADC1->CR2 |= ADC_CR2_SWSTART; // Software ADC start
		while (!(ADC1->SR & ADC_SR_EOC)); // Wait until conversion end
		i = ADC1->DR;
		UART_SendHex16(USART2,i);
		UART_SendStr(USART2," ");
		UART_SendInt(USART2,i);
		UART_SendStr(USART2,"\n");
		Delay_ms(100);
	}

	while(1);
*/

	// Configure USB peripheral
	USB_HWConfig();

	// Initialize USB device
	USB_Init();

	buf_pos = 0;
	USB_buf_pos = 0;
	ADC_buf_pos = 0;
	ovs_accu = 0;
	ovs_cntr = 0;

	TIM9->CR1 |= TIM_CR1_CEN; // Enable TIM9

    while(1) {
    	if (packet_send) {
    		if (_GetENDPOINT(ENDP1) & EP_DTOG_RX) {
        		UserToPMABufferCopy(&ADC_buffer[USB_buf_pos],ENDP1_BUF0Addr,0x10);
        		SetEPDblBuf0Count(ENDP1,EP_DBUF_IN,0x10);
    		} else {
        		UserToPMABufferCopy(&ADC_buffer[USB_buf_pos],ENDP1_BUF1Addr,0x10);
        		SetEPDblBuf1Count(ENDP1,EP_DBUF_IN,0x10);
    		}
    		FreeUserBuffer(ENDP1,EP_DBUF_IN);
    		SetEPTxValid(ENDP1);
    		packet_send = 0;

    		USB_buf_pos += 0x10;
    		if (USB_buf_pos > ADC_BUF_SIZE - 0x10) USB_buf_pos = 0;
    	}
    }
}
