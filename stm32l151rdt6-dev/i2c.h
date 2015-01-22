// Define to prevent recursive inclusion -------------------------------------
#ifndef __I2C2_H
#define __I2C2_H

// I2C2 peripheral

#define I2C1_SCL_PIN      GPIO_Pin_6  // PB6
#define I2C1_SDA_PIN      GPIO_Pin_7  // PB7
#define I2C1_SCL_PIN_SRC  GPIO_PinSource6
#define I2C1_SDA_PIN_SRC  GPIO_PinSource7
#define I2C1_GPIO_PORT    GPIOB
#define I2C1_CLOCK        RCC_APB1Periph_I2C1
#define I2C1_PERIPH       RCC_AHBPeriph_GPIOB

#define I2C2_SCL_PIN      GPIO_Pin_10 // PB10
#define I2C2_SDA_PIN      GPIO_Pin_11 // PB11
#define I2C2_SCL_PIN_SRC  GPIO_PinSource10
#define I2C2_SDA_PIN_SRC  GPIO_PinSource11
#define I2C2_GPIO_PORT    GPIOB
#define I2C2_CLOCK        RCC_APB1Periph_I2C2
#define I2C2_PERIPH       RCC_AHBPeriph_GPIOB


typedef enum {
	I2C_ERROR   = 0,
	I2C_SUCCESS = !I2C_ERROR
} I2C_Status;

typedef enum {
	I2C_STOP   = 0,
	I2C_NOSTOP = !I2C_STOP
} I2C_STOP_TypeDef;


#define I2C_WAIT_TIMEOUT  (uint16_t)0x5000     // Timeout for I2C operations


// Function prototypes
I2C_Status I2Cx_Init(I2C_TypeDef* I2Cx, uint32_t Clock);

I2C_Status I2Cx_Write(I2C_TypeDef* I2Cx, const uint8_t* buf, uint32_t nbytes,
		uint8_t SlaveAddress, I2C_STOP_TypeDef stop);
I2C_Status I2Cx_Read(I2C_TypeDef* I2Cx, uint8_t *buf, uint32_t nbytes,
		uint8_t SlaveAddress);

#endif // __I2C2_H
