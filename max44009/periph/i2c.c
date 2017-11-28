// Inter-integrated circuit (I2C) management


#include "i2c.h"


// I2C timeout, about 2ms
#define I2C_TIMEOUT             100U

// Maximum NBYTES value
#define I2C_NBYTES_MAX          255U


// Count rough delay for timeouts
static uint32_t I2C_CalcDelay(uint32_t delay) {
	uint32_t cnt;

	if (SystemCoreClock > 1000000U) {
		cnt = (delay * ((SystemCoreClock / 1000000U) + 1U));
	} else {
		cnt = (((delay / 100U) + 1U) * ((SystemCoreClock / 10000U) + 1U));
	}

	return cnt;
}

// Initialize the specified I2C peripheral and its GPIO lines
// input:
//   I2Cx - pointer to the I2C peripheral (I2C1, etc.)
void I2C_Init(I2C_TypeDef* I2Cx) {
	// Disable the I2C peripheral
	I2C_Disable(I2Cx);

	if (I2Cx == I2C1) {
		// Reset the I2C1 peripheral to initial state
		RCC->APB1RSTR1 |=  RCC_APB1RSTR1_I2C1RST;
		RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;

		// Enable the I2C1 peripheral clock
		RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

		// Enable the I2C1 GPIO peripherals
		RCC->AHB2ENR |= I2C1_SCL_GPIO_PERIPH | I2C1_SDA_GPIO_PERIPH;

		// Initialize the I2C1 GPIO
		// SCL
		GPIO_set_mode(I2C1_SCL_GPIO_PORT,GPIO_Mode_AF,GPIO_PUPD_NONE,I2C1_SCL_GPIO_PIN);
		GPIO_out_cfg(I2C1_SCL_GPIO_PORT,GPIO_OT_OD,GPIO_SPD_LOW,I2C1_SCL_GPIO_PIN);
		GPIO_af_cfg(I2C1_SCL_GPIO_PORT,I2C1_SCL_GPIO_SRC,GPIO_AF4); // AF4 = I2C1
		// SDA
		GPIO_set_mode(I2C1_SDA_GPIO_PORT,GPIO_Mode_AF,GPIO_PUPD_NONE,I2C1_SDA_GPIO_PIN);
		GPIO_out_cfg(I2C1_SDA_GPIO_PORT,GPIO_OT_OD,GPIO_SPD_LOW,I2C1_SDA_GPIO_PIN);
		GPIO_af_cfg(I2C1_SDA_GPIO_PORT,I2C1_SDA_GPIO_SRC,GPIO_AF4); // AF4 = I2C1
	} else if (I2Cx == I2C2) {
		// Reset the I2C2 peripheral to initial state
		RCC->APB1RSTR1 |=  RCC_APB1RSTR1_I2C2RST;
		RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C2RST;

		// Enable the I2C2 peripheral clock
		RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

		// Enable the I2C2 GPIO peripherals
		RCC->AHB2ENR |= I2C2_SCL_GPIO_PERIPH | I2C2_SDA_GPIO_PERIPH;

		// Initialize the I2C2 GPIO
		// SCL
		GPIO_set_mode(I2C2_SCL_GPIO_PORT,GPIO_Mode_AF,GPIO_PUPD_NONE,I2C2_SCL_GPIO_PIN);
		GPIO_out_cfg(I2C2_SCL_GPIO_PORT,GPIO_OT_OD,GPIO_SPD_LOW,I2C2_SCL_GPIO_PIN);
		GPIO_af_cfg(I2C2_SCL_GPIO_PORT,I2C2_SCL_GPIO_SRC,GPIO_AF4); // AF4 = I2C2
		// SDA
		GPIO_set_mode(I2C2_SDA_GPIO_PORT,GPIO_Mode_AF,GPIO_PUPD_NONE,I2C2_SDA_GPIO_PIN);
		GPIO_out_cfg(I2C2_SDA_GPIO_PORT,GPIO_OT_OD,GPIO_SPD_LOW,I2C2_SDA_GPIO_PIN);
		GPIO_af_cfg(I2C2_SDA_GPIO_PORT,I2C2_SDA_GPIO_SRC,GPIO_AF4); // AF4 = I2C2
	} else if (I2Cx == I2C3) {
		// Reset the I2C3 peripheral to initial state
		RCC->APB1RSTR1 |=  RCC_APB1RSTR1_I2C3RST;
		RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C3RST;

		// Enable the I2C3 peripheral clock
		RCC->APB1ENR1 |= RCC_APB1ENR1_I2C3EN;

		// Enable the I2C3 GPIO peripherals
		RCC->AHB2ENR |= I2C3_SCL_GPIO_PERIPH | I2C3_SDA_GPIO_PERIPH;

		// Initialize the I2C3 GPIO
		// SCL
		GPIO_set_mode(I2C3_SCL_GPIO_PORT,GPIO_Mode_AF,GPIO_PUPD_NONE,I2C3_SCL_GPIO_PIN);
		GPIO_out_cfg(I2C3_SCL_GPIO_PORT,GPIO_OT_OD,GPIO_SPD_LOW,I2C3_SCL_GPIO_PIN);
		GPIO_af_cfg(I2C3_SCL_GPIO_PORT,I2C3_SCL_GPIO_SRC,GPIO_AF4); // AF4 = I2C3
		// SDA
		GPIO_set_mode(I2C3_SDA_GPIO_PORT,GPIO_Mode_AF,GPIO_PUPD_NONE,I2C3_SDA_GPIO_PIN);
		GPIO_out_cfg(I2C3_SDA_GPIO_PORT,GPIO_OT_OD,GPIO_SPD_LOW,I2C3_SDA_GPIO_PIN);
		GPIO_af_cfg(I2C3_SDA_GPIO_PORT,I2C3_SDA_GPIO_SRC,GPIO_AF4); // AF4 = I2C3
	} else {
		// Unknown I2C port specified
	}
}

// Check if target device is ready for communication
// input:
//   I2Cx - pointer to the I2C peripheral (I2C1, etc.)
//   devAddr - target device address
//   trials - number of trials (must not be zero)
// return:
//   I2C_ERROR if there was a timeout during I2C operations, I2C_SUCCESS otherwise
I2CSTATUS I2C_IsDeviceReady(I2C_TypeDef* I2Cx, uint8_t devAddr, uint32_t trials) {
	volatile uint32_t wait;
	uint32_t delay_val = I2C_CalcDelay(I2C_TIMEOUT);
	uint32_t reg;

	while (trials--) {
		// Generate START
		I2C_GenStart(I2Cx,devAddr);

		// Wait for STOP, NACK or BERR
		wait = delay_val;
		while (!((reg = I2Cx->ISR) & (I2C_ISR_STOPF | I2C_ISR_NACKF | I2C_ISR_BERR)) && --wait);
		if (wait == 0) return I2C_ERROR;

		// Wait while STOP flag is reset
		wait = delay_val;
		while (!(I2Cx->ISR & I2C_ISR_STOPF) && --wait);
		if (wait == 0) return I2C_ERROR;

		// Clear the NACK, STOP and BERR flags
		I2Cx->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF;

		// Check for BERR flag
		if (reg & I2C_ISR_BERR) {
			// TODO: I2C: how to handle the misplaced START/STOP?
		} else {
			// Device responded if NACK flag is not set
			if (!(reg & I2C_ISR_NACKF)) {
				return I2C_SUCCESS;
			}
		}
	}

	return I2C_ERROR;
}

// Transmit an amount of data in master mode
// input:
//   I2Cx - pointer to the I2C peripheral (I2C1, etc.)
//   pBbuf - pointer to the data buffer
//   nbytes - number of bytes to transmit
//   devAddr - address of target device
//   stop - generate or not the STOP condition (I2C_GENSTOP_YES or I2C_GENSTOP_NO)
// return:
//   I2C_ERROR if there was a timeout during I2C operations, I2C_SUCCESS otherwise
I2CSTATUS I2C_Transmit(I2C_TypeDef* I2Cx, const uint8_t *pBuf, uint32_t nbytes, uint8_t devAddr, uint32_t stop) {
	uint32_t reg;
	uint32_t tx_count;
	uint32_t delay_val = I2C_CalcDelay(I2C_TIMEOUT);
	volatile uint32_t wait;

	// Configure the slave device address, transfer length and generate START condition
	reg  = I2Cx->CR2;
	reg &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);
	reg |= (devAddr & I2C_CR2_SADD) | I2C_CR2_START;
	if (nbytes > I2C_NBYTES_MAX) {
		// The number of bytes to transmit is greater than the maximum value for
		// one transfer, in this case the RELOAD bit must be set
		reg |= (I2C_NBYTES_MAX << 16) | I2C_CR2_RELOAD;
		tx_count = I2C_NBYTES_MAX;
	} else {
		reg |= (nbytes << 16) | (stop & I2C_CR2_AUTOEND);
		tx_count = nbytes;
	}
	I2Cx->CR2 = reg;

	// Transmit data
	while (nbytes) {
		// Wait until either TXIS or NACK flag is set
		wait = delay_val;
		while (!((reg = I2Cx->ISR) & (I2C_ISR_TXIS | I2C_ISR_NACKF)) && --wait);
		if ((reg & I2C_ISR_NACKF) || (wait == 0))  {
			// Clear the NACK flag
			I2Cx->ICR = I2C_ICR_NACKCF;

			return I2C_ERROR;
		}

		// Transmit byte
		I2Cx->TXDR = *pBuf++;
		nbytes--;
		tx_count--;

		if ((tx_count == 0) && (nbytes != 0)) {
			// Wait until TCR flag is set (Transfer Complete Reload)
			wait = delay_val;
			while (!(I2Cx->ISR & I2C_ISR_TCR) && --wait);
			if (wait == 0) return I2C_ERROR;

			// Is this last part of buffer?
			reg = I2Cx->CR2;
			if (nbytes > I2C_NBYTES_MAX) {
				reg |= (I2C_NBYTES_MAX << 16) | I2C_CR2_RELOAD;
				tx_count = I2C_NBYTES_MAX;
			} else {
				reg &= ~(I2C_CR2_RELOAD | I2C_CR2_NBYTES);
				reg |= (nbytes << 16) | (stop & I2C_CR2_AUTOEND);
				tx_count = nbytes;
			}
			I2Cx->CR2 = reg;
		}
	}

	// End of transmit
	wait = delay_val;
	if (stop == I2C_GENSTOP_YES) {
		// With AUTOEND bit set the STOP condition generated automatically
		// Wait for STOP flag is set
		while (!(I2Cx->ISR & I2C_ISR_STOPF) && --wait);

		// Clear the STOP flag
		I2Cx->ICR = I2C_ICR_STOPCF;
	} else {
		// The STOP condition is not generated, wait for TC (Transfer Complete) flag
		while (!(I2Cx->ISR & I2C_ISR_TC) && --wait);
	}

	// Reset configuration register
	I2Cx->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN);

	return I2C_SUCCESS;
}

// Receive an amount of data in master mode
// input:
//   I2Cx - pointer to the I2C peripheral (I2C1, etc.)
//   buf - pointer to the data buffer
//   nbytes - number of bytes to receive
//   devAddr - address of target device
// return:
//   I2C_ERROR if there was a timeout during I2C operations, I2C_SUCCESS otherwise
I2CSTATUS I2C_Receive(I2C_TypeDef* I2Cx, uint8_t *pBuf, uint32_t nbytes, uint8_t devAddr) {
	uint32_t reg;
	uint32_t tx_count;
	uint32_t delay_val = I2C_CalcDelay(I2C_TIMEOUT);
	volatile uint32_t wait;

	// Configure the slave device address, transfer length and generate START condition
	reg  = I2Cx->CR2;
	reg &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);
	reg |= (devAddr & I2C_CR2_SADD) | I2C_CR2_START | I2C_CR2_RD_WRN;
	if (nbytes > I2C_NBYTES_MAX) {
		// The number of bytes to transmit is greater than the maximum value for
		// one transfer, in this case the RELOAD bit must be set
		reg |= (I2C_NBYTES_MAX << 16) | I2C_CR2_RELOAD;
		tx_count = I2C_NBYTES_MAX;
	} else {
		reg |= (nbytes << 16) | I2C_CR2_AUTOEND;
		tx_count = nbytes;
	}
	I2Cx->CR2 = reg;

	// Receive data
	while (nbytes) {
		// Wait until either RXNE or NACK flag is set
		wait = delay_val;
		while (!((reg = I2Cx->ISR) & (I2C_ISR_RXNE | I2C_ISR_NACKF)) && --wait);
		if ((reg & I2C_ISR_NACKF) || (wait == 0)) {
			I2Cx->ICR = I2C_ICR_NACKCF;

			return I2C_ERROR;
		}

		// Read received data
		*pBuf++ = I2Cx->RXDR;
		nbytes--;
		tx_count--;

		if ((tx_count == 0) && (nbytes != 0)) {
			// Wait until TCR flag is set (Transfer Complete Reload)
			wait = delay_val;
			while (!(I2Cx->ISR & I2C_ISR_TCR) && --wait);
			if (wait == 0) {
				return I2C_ERROR;
			}

			// Is this last part of buffer?
			reg = I2Cx->CR2;
			if (nbytes > I2C_NBYTES_MAX) {
				reg |= (I2C_NBYTES_MAX << 16) | I2C_CR2_RELOAD;
				tx_count = I2C_NBYTES_MAX;
			} else {
				reg &= ~(I2C_CR2_RELOAD | I2C_CR2_NBYTES);
				reg |= (nbytes << 16) | I2C_CR2_AUTOEND;
				tx_count = nbytes;
			}
			I2Cx->CR2 = reg;
		}
	}

	// With AUTOEND bit set the STOP condition generated automatically
	// Wait for STOP flag is set
	wait = delay_val;
	while (!(I2Cx->ISR & I2C_ISR_STOPF) && --wait);

	// Clear the STOP flag
	I2Cx->ICR = I2C_ICR_STOPCF;

	// Reset configuration register
	I2Cx->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN);

	return I2C_SUCCESS;
}
