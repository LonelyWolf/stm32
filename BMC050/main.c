#include <stm32l1xx.h>
#include <misc.h>

#include "delay.h"
#include "i2c.h"
#include "uart.h"
#include "bmc050.h"


uint8_t i8;
int16_t temp;
int16_t X,Y,Z;
NVIC_InitTypeDef NVICInit;


#define INT1_EXTI_LINE (1 << 6) // INT1 connected to PA6



void EXTI9_5_IRQHandler(void) {
	if (EXTI->PR & INT1_EXTI_LINE) {
		i8 = BMC050_ACC_GetTSIRQ(); // Get IRQ source
		BMC050_ACC_GetXYZ(&X,&Y,&Z); // Get last accelerometer readings
		BMC050_ACC_SetIRQMode(ACC_IM_RESET); // Reset all latched interrupts

		UART_SendStr(USART2,"Slope=");
		UART_SendHex8(USART2,i8);
		UART_SendChar(USART2,' ');
		if (i8 & ACC_TS_SLOPEZ) UART_SendStr(USART2,"SLOPEZ ");
		if (i8 & ACC_TS_SLOPEY) UART_SendStr(USART2,"SLOPEY ");
		if (i8 & ACC_TS_SLOPEX) UART_SendStr(USART2,"SLOPEX ");

		UART_SendStr(USART2," X=");
		UART_SendInt(USART2,X);
		UART_SendStr(USART2," Y=");
		UART_SendInt(USART2,Y);
		UART_SendStr(USART2," Z=");
		UART_SendInt(USART2,Z);
		UART_SendChar(USART2,'\n');

		EXTI->PR = INT1_EXTI_LINE; // Clear IT bit for EXTI line
	}
}



int main(void) {
	Delay_Init((void *)0);

	UARTx_Init(USART2,1382400);

	UART_SendStr(USART2,"--------------------------------------\n");

	if (I2Cx_Init(BMC050_I2C_PORT,400000) != I2C_SUCCESS) {
		UART_SendStr(USART2,"I2C2 init fail\n");
		while(1);
	}
	UART_SendStr(USART2,"I2C2 init at 400kHz\n");


	// Enable PORTA peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA6 as external interrupt
	GPIOA->MODER &= ~GPIO_MODER_MODER6; // Input mode (reset state)
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR6; // No pull-up, pull-down
	GPIOA->PUPDR |=  GPIO_PUPDR_PUPDR6_1; // Pull-down

	// Configure priority group: 4 bits for preemption priority, 0 bits for subpriority.
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	// Enable the SYSCFG module clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// PA6 -> EXTI line 6 (INT1 from BMC050)
	EXTI->PR    =  INT1_EXTI_LINE; // Clear IT pending bit for EXTI line
	EXTI->IMR  |=  INT1_EXTI_LINE; // Enable interrupt request from EXTI line
	EXTI->EMR  &= ~INT1_EXTI_LINE; // Disable event on EXTI line
	EXTI->RTSR |=  INT1_EXTI_LINE; // Trigger rising edge enabled
	EXTI->FTSR &= ~INT1_EXTI_LINE; // Trigger falling edge disabled

	// Enable the USB interrupt
	NVICInit.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 2;
	NVICInit.NVIC_IRQChannelSubPriority = 0;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInit);


///*
	uint8_t buf[2];
	uint8_t reg;
	uint8_t val;
	uint32_t i;
//*/

///*
	reg = 0x00;
	val = 0;

	UART_SendStr(USART2,"\nAccelerometer\n");
	for (reg = 0; reg <= 0x3f; reg++) {
		I2Cx_Write(BMC050_I2C_PORT,&reg,1,0x18 << 1,I2C_NOSTOP);
		I2Cx_Read(BMC050_I2C_PORT,&val,1,0x18 << 1);
		UART_SendStr(USART2,"R");
		UART_SendHex8(USART2,reg);
		UART_SendStr(USART2," = ");
		UART_SendHex8(USART2,val);
		UART_SendStr(USART2,"\t\t");
	}
	UART_SendChar(USART2,'\n');
//*/

/*
	UART_SendStr(USART2,"\nMagnetometer\n");

	// Magnetometer power enable
	buf[0] = 0x4b;
	buf[1] = 0x01; // Set power control bit
	I2Cx_Write(BMC050_I2C_PORT,&buf[0],2,0x10 << 1,I2C_STOP);

	for (reg = 0x40; reg <= 0x52; reg++) {
		I2Cx_Write(BMC050_I2C_PORT,&reg,1,0x10 << 1,I2C_NOSTOP);
		I2Cx_Read(BMC050_I2C_PORT,&val,1,0x10 << 1);
		UART_SendStr(USART2,"R");
		UART_SendHex8(USART2,reg);
		UART_SendStr(USART2," = ");
		UART_SendHex8(USART2,val);
		UART_SendStr(USART2,"\t\t");
	}
	UART_SendStr(USART2,"\n========================================\n");
*/

	BMC050_ACC_SoftReset();
	Delay_ms(5); // must wait for start-up time of accelerometer (2ms)
	BMC050_Init();

	// Enable I2C watchdog timer with 50ms
	BMC050_ACC_InterfaceConfig(ACC_IF_WDT_50ms);

	UART_SendStr(USART2,"BMC050 ACC device ID: ");
	UART_SendHex8(USART2,BMC050_ACC_GetDeviceID());
	UART_SendChar(USART2,'\n');

	UART_SendStr(USART2,"BMC050 MAG device ID: ");
	UART_SendHex8(USART2,BMC050_MAG_GetDeviceID());
	UART_SendChar(USART2,'\n');

	UART_SendStr(USART2,"BMC050 temperature: ");
	temp = BMC050_ReadTemp();
	UART_SendInt(USART2,temp / 10);
	UART_SendChar(USART2,'.');
	UART_SendInt(USART2,temp % 10);
	UART_SendStr(USART2,"C\n");

	BMC050_ACC_SetBandwidth(ACC_BW8); // Accelerometer readings filtering (lower or higher better?)
	BMC050_ACC_SetIRQMode(ACC_IM_NOLATCH); // No IRQ latching
	BMC050_ACC_ConfigSlopeIRQ(0,16); // Motion detection sensitivity
	BMC050_ACC_IntPinMap(ACC_IM1_SLOPE); // Map slope interrupt to INT1 pin
	BMC050_ACC_SetIRQ(ACC_IE_SLOPEX | ACC_IE_SLOPEY | ACC_IE_SLOPEZ); // Detect motion by all axes
	BMC050_ACC_LowPower(ACC_SLEEP_100); // Low power with sleep duration 0.1s

//	BMC050_ACC_Suspend();

	while(1);

/*
	while(1) {
		while (!BMC050_ACC_GetIRQStatus()); // Wait for new data from accelerometer

		i8 = BMC050_ACC_GetTSIRQ();
		BMC050_ACC_GetXYZ(&X,&Y,&Z);

		UART_SendStr(USART2,"Slope=");
		UART_SendHex8(USART2,i8);
		UART_SendChar(USART2,' ');
		if (i8 & ACC_TS_SLOPEZ) UART_SendStr(USART2,"SLOPEZ ");
		if (i8 & ACC_TS_SLOPEY) UART_SendStr(USART2,"SLOPEY ");
		if (i8 & ACC_TS_SLOPEX) UART_SendStr(USART2,"SLOPEX ");

		UART_SendStr(USART2," X=");
		UART_SendInt(USART2,X);
		UART_SendStr(USART2," Y=");
		UART_SendInt(USART2,Y);
		UART_SendStr(USART2," Z=");
		UART_SendInt(USART2,Z);
		UART_SendChar(USART2,'\n');

		BMC050_ACC_SetIRQMode(ACC_IM_RESET);

		Delay_ms(100);
	}
*/
}
