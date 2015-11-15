// Define to prevent recursive inclusion -------------------------------------
#ifndef __I2C_H
#define __I2C_H


#include <stm32l1xx.h>


// I2C HAL

// I2C1
// SCL [PB6, PB8]
#define I2C1_SCL_GPIO_PERIPH    RCC_AHBENR_GPIOBEN
#define I2C1_SCL_GPIO_PORT      GPIOB
#define I2C1_SCL_GPIO_PIN       GPIO_Pin_6
#define I2C1_SCL_GPIO_SRC       GPIO_PinSource6
// SDA [PB7, PB9]
#define I2C1_SDA_GPIO_PERIPH    RCC_AHBENR_GPIOBEN
#define I2C1_SDA_GPIO_PORT      GPIOB
#define I2C1_SDA_GPIO_PIN       GPIO_Pin_7
#define I2C1_SDA_GPIO_SRC       GPIO_PinSource7

// I2C2
// SCL [PB10]
#define I2C2_SCL_GPIO_PERIPH    RCC_AHBENR_GPIOBEN
#define I2C2_SCL_GPIO_PORT      GPIOB
#define I2C2_SCL_GPIO_PIN       GPIO_Pin_10
#define I2C2_SCL_GPIO_SRC       GPIO_PinSource10
// SDA [PB11]
#define I2C2_SDA_GPIO_PERIPH    RCC_AHBENR_GPIOBEN
#define I2C2_SDA_GPIO_PORT      GPIOB
#define I2C2_SDA_GPIO_PIN       GPIO_Pin_11
#define I2C2_SDA_GPIO_SRC       GPIO_PinSource11


// Result of I2C procedures
typedef enum {
	I2C_ERROR   = 0,
	I2C_SUCCESS = !I2C_ERROR
} I2CStatus;

// Send or not STOP condition
typedef enum {
	I2C_STOP   = 0,
	I2C_NOSTOP = !I2C_STOP
} I2CStop;


// I2C defines
#define I2C_WAIT_TIMEOUT        ((uint32_t)0x00050000) // Timeout for I2C operations
#define I2C_FLAG_MASK           ((uint32_t)0x00FFFFFF) // I2C flags mask (clear PEC register)

// I2C flags (SR1 register)
#define I2C_F_TXE               ((uint32_t)0x80000080) // Data register empty (transmitters)
#define I2C_F_RXNE              ((uint32_t)0x80000040) // Data register not empty (receivers)
#define I2C_F_STOPF             ((uint32_t)0x80000010) // Stop detection
#define I2C_F_BTF               ((uint32_t)0x80000004) // Byte transfer finished
#define I2C_F_ADDR              ((uint32_t)0x80000002) // Address sent
#define I2C_F_SB                ((uint32_t)0x80000001) // Start condition generated

// I2C flags (SR2 register)
#define I2C_F_BUSY              ((uint32_t)0x00000002) // Bus busy


// Macros definitions
// Clear the ADDR flag
#define __I2C_CLEAR_ADDR(I2Cx) { \
	(void)(I2Cx)->SR1;         \
	(void)(I2Cx)->SR2;         \
}


// Function prototypes
I2CStatus I2Cx_Init(I2C_TypeDef* I2Cx, uint32_t Clock);
I2CStatus I2Cx_Transmit(I2C_TypeDef* I2Cx, const uint8_t *buf, uint32_t nbytes, uint8_t devAddr, I2CStop stop);
I2CStatus I2Cx_Receive(I2C_TypeDef* I2Cx, uint8_t *buf, uint32_t nbytes, uint8_t devAddr);
I2CStatus I2Cx_IsDeviceReady(I2C_TypeDef* I2Cx, uint8_t devAddr, uint32_t Trials);

#endif // __I2C_H
