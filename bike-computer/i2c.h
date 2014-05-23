// Define to prevent recursive inclusion -------------------------------------
#ifndef __I2C2_H
#define __I2C2_H

// I2C2 peripheral
#define I2C2_PERIPH      RCC_AHBPeriph_GPIOB

#define I2C2_PORT         I2C2
#define I2C2_SCL_PIN      GPIO_Pin_10    // PB10
#define I2C2_SDA_PIN      GPIO_Pin_11    // PB11
#define I2C2_GPIO_PORT    GPIOB
#define I2C2_CLOCK        RCC_APB1Periph_I2C2


typedef enum {
	I2C_STOP   = 0,
	I2C_NOSTOP = !I2C_STOP
} I2C_STOP_TypeDef;

typedef enum {
	I2C_START   = 0,
	I2C_NOSTART = !I2C_START
} I2C_START_TypeDef;


// Function prototypes
void I2C2_Init(uint32_t Clock);

uint8_t I2C2_Write(const uint8_t* buf, uint32_t nbytes, uint8_t SlaveAddress, I2C_STOP_TypeDef stop);
uint8_t I2C2_Read(uint8_t *buf, uint32_t nbytes, uint8_t SlaveAddress);

#endif // __I2C2_H
