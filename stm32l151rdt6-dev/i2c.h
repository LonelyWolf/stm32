// Define to prevent recursive inclusion -------------------------------------
#ifndef __I2C_H
#define __I2C_H


#include <stm32l1xx.h>


// I2C HAL

// I2C1
#define I2C1_GPIO_AHB      RCC_AHBENR_GPIOBEN
#define I2C1_GPIO_PORT     GPIOB
#define I2C1_GPIO_SCL      GPIO_Pin_6 // PB6
#define I2C1_GPIO_SDA      GPIO_Pin_7 // PB7
#define I2C1_GPIO_SCL_SRC  GPIO_PinSource6
#define I2C1_GPIO_SDA_SRC  GPIO_PinSource7

// I2C2
#define I2C2_GPIO_AHB      RCC_AHBENR_GPIOBEN
#define I2C2_GPIO_PORT     GPIOB
#define I2C2_GPIO_SCL      GPIO_Pin_10 // PB10
#define I2C2_GPIO_SDA      GPIO_Pin_11 // PB11
#define I2C2_GPIO_SCL_SRC  GPIO_PinSource10
#define I2C2_GPIO_SDA_SRC  GPIO_PinSource11


// Result of I2C procedures
typedef enum {
	I2C_ERROR   = 0,
	I2C_SUCCESS = !I2C_ERROR
} I2C_Status;

// Send or not STOP condition
typedef enum {
	I2C_STOP   = 0,
	I2C_NOSTOP = !I2C_STOP
} I2C_STOP_TypeDef;


// I2C defines
#define I2C_WAIT_TIMEOUT  (uint32_t)0x00005000 // Timeout for I2C operations
#define I2C_FLAG_MASK     (uint32_t)0x00FFFFFF // I2C flags mask (clear PEC register)

// I2C events
//#define I2C_EVENT_EV5     (uint32_t)(((I2C_SR2_BUSY | I2C_SR2_MSL) << 16) | I2C_SR1_SB)
//#define I2C_EVENT_EV6     ((uint32_t)((I2C_SR2_BUSY | I2C_SR2_MSL | I2C_SR2_TRA) << 16) | I2C_SR1_TXE | I2C_SR1_ADDR)
//#define I2C_EVENT_EV7     ((uint32_t)((I2C_SR2_BUSY | I2C_SR2_MSL) << 16) | I2C_SR1_RXNE)
#define I2C_EVENT_EV5     ((uint32_t)0x00030001) // BUSY, MSL, SB (MASTER_MODE_SELECT)
#define I2C_EVENT_EV6     ((uint32_t)0x00070082) // BUSY, MSL, TRA, TXE, ADDR (MASTER_TRANSMITTER_MODE_SELECTED)
#define I2C_EVENT_EV7     ((uint32_t)0x00030040) // BUSY, MSL, RXNE (MASTER_BYTE_RECEIVED)

// I2C flags (SR1 register)
#define I2C_F_RXNE        ((uint32_t)0x80000040) // Data register not empty
#define I2C_F_STOPF       ((uint32_t)0x80000010) // Stop detection
#define I2C_F_BTF         ((uint32_t)0x80000004) // Byte transfer finished
#define I2C_F_ADDR        ((uint32_t)0x80000002) // Address sent
#define I2C_F_SB          ((uint32_t)0x80000001) // Start condition generated

// I2C flags (SR2 register)
#define I2C_F_BUSY        ((uint32_t)0x00000002) // Bus busy


// Function prototypes
I2C_Status I2Cx_Init(I2C_TypeDef* I2Cx, uint32_t Clock);
I2C_Status I2Cx_Write(I2C_TypeDef* I2Cx, const uint8_t* buf, uint32_t nbytes,
		uint8_t SlaveAddress, I2C_STOP_TypeDef stop);
I2C_Status I2Cx_Read(I2C_TypeDef* I2Cx, uint8_t *buf, uint32_t nbytes,
		uint8_t SlaveAddress);
I2C_Status I2Cx_IsDeviceReady(I2C_TypeDef* I2Cx, uint8_t SlaveAddress, uint32_t Trials);

#endif // __I2C_H
