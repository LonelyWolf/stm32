// Inter-integrated circuit (I2C) management


#include "i2c.h"


// I2C timeout, about 2ms
#define I2C_TIMEOUT             200U

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
		GPIO_af_cfg(I2C1_SCL_GPIO_PORT, I2C1_SCL_GPIO_SRC, GPIO_AF4); // AF4 = I2C1
		GPIO_set_mode(I2C1_SCL_GPIO_PORT, GPIO_Mode_AF, GPIO_PUPD_NONE, I2C1_SCL_GPIO_PIN);
		GPIO_out_cfg(I2C1_SCL_GPIO_PORT, GPIO_OT_OD, GPIO_SPD_LOW, I2C1_SCL_GPIO_PIN);
		// SDA
		GPIO_af_cfg(I2C1_SDA_GPIO_PORT, I2C1_SDA_GPIO_SRC, GPIO_AF4); // AF4 = I2C1
		GPIO_set_mode(I2C1_SDA_GPIO_PORT, GPIO_Mode_AF, GPIO_PUPD_NONE, I2C1_SDA_GPIO_PIN);
		GPIO_out_cfg(I2C1_SDA_GPIO_PORT, GPIO_OT_OD, GPIO_SPD_LOW, I2C1_SDA_GPIO_PIN);
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
		GPIO_af_cfg(I2C2_SCL_GPIO_PORT, I2C2_SCL_GPIO_SRC, GPIO_AF4); // AF4 = I2C2
		GPIO_set_mode(I2C2_SCL_GPIO_PORT, GPIO_Mode_AF, GPIO_PUPD_NONE, I2C2_SCL_GPIO_PIN);
		GPIO_out_cfg(I2C2_SCL_GPIO_PORT, GPIO_OT_OD, GPIO_SPD_LOW, I2C2_SCL_GPIO_PIN);
		// SDA
		GPIO_af_cfg(I2C2_SDA_GPIO_PORT, I2C2_SDA_GPIO_SRC, GPIO_AF4); // AF4 = I2C2
		GPIO_set_mode(I2C2_SDA_GPIO_PORT, GPIO_Mode_AF, GPIO_PUPD_NONE, I2C2_SDA_GPIO_PIN);
		GPIO_out_cfg(I2C2_SDA_GPIO_PORT, GPIO_OT_OD, GPIO_SPD_LOW, I2C2_SDA_GPIO_PIN);
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
		GPIO_af_cfg(I2C3_SCL_GPIO_PORT, I2C3_SCL_GPIO_SRC, GPIO_AF4); // AF4 = I2C3
		GPIO_set_mode(I2C3_SCL_GPIO_PORT, GPIO_Mode_AF, GPIO_PUPD_NONE, I2C3_SCL_GPIO_PIN);
		GPIO_out_cfg(I2C3_SCL_GPIO_PORT, GPIO_OT_OD, GPIO_SPD_LOW, I2C3_SCL_GPIO_PIN);
		// SDA
		GPIO_af_cfg(I2C3_SDA_GPIO_PORT, I2C3_SDA_GPIO_SRC, GPIO_AF4); // AF4 = I2C3
		GPIO_set_mode(I2C3_SDA_GPIO_PORT, GPIO_Mode_AF, GPIO_PUPD_NONE, I2C3_SDA_GPIO_PIN);
		GPIO_out_cfg(I2C3_SDA_GPIO_PORT, GPIO_OT_OD, GPIO_SPD_LOW, I2C3_SDA_GPIO_PIN);
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
		// Clear all flags
		I2Cx->ICR = I2C_ICR_ALL;

		// Generate START
		I2C_GenStart(I2Cx, devAddr);

		// Wait for STOP, NACK or BERR
		wait = delay_val;
		while (!((reg = I2Cx->ISR) & (I2C_ISR_STOPF | I2C_ISR_NACKF | I2C_ISR_BERR)) && --wait);
		if (wait == 0) { return I2C_ERROR; }

		// Wait while STOP flag is reset
		wait = delay_val;
		while (!(I2Cx->ISR & I2C_ISR_STOPF) && --wait);
		if (wait == 0) { return I2C_ERROR; }

		// Clear the NACK, STOP and BERR flags
		I2Cx->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF;

		// Check for BERR flag
		if (reg & I2C_ISR_BERR) {
			// Misplaced START/STOP? Perform a software reset of I2C
			I2C_Disable(I2Cx);
			I2C_Enable(I2Cx);
		} else {
			// Device responded if NACK flag is not set
			if (!(reg & I2C_ISR_NACKF)) { return I2C_SUCCESS; }
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
//   flags - options for transmission, combination of I2C_TX_xx values:
//     I2C_TX_NOSTART - don't generate START condition
//     I2C_TX_NOSTOP - don't generate STOP condition
//     I2C_TX_CONT - this flag indicates that transmission will be continued
//                   e.g. by calling this function again with NOSTART flag
//     zero value - generate both START and STOP conditions
// return:
//   I2C_ERROR if there was a timeout during I2C operations, I2C_SUCCESS otherwise
I2CSTATUS I2C_Transmit(I2C_TypeDef* I2Cx, const uint8_t *pBuf, uint32_t nbytes, uint8_t devAddr, uint32_t flags) {
	uint32_t reg;
	uint32_t tx_count;
	uint32_t delay_val = I2C_CalcDelay(I2C_TIMEOUT);
	volatile uint32_t wait;

	// Clear all flags
	I2Cx->ICR = I2C_ICR_ALL;

	// Everything regarding to the transmission is in the CR2 register
	reg = I2Cx->CR2;
	reg &= ~I2C_CR2_ALL;

	// Slave device address
	reg |= (devAddr & I2C_CR2_SADD);

	// Whether it need to generate START condition
	if (!(flags & I2C_TX_NOSTART)) { reg |= I2C_CR2_START; }

	// Whether it need to generate STOP condition
	if ((flags & I2C_TX_CONT) || (nbytes > I2C_NBYTES_MAX)) {
		reg |= I2C_CR2_RELOAD;
	} else {
		if (!(flags & I2C_TX_NOSTOP)) { reg |= I2C_CR2_AUTOEND; }
	}

	// Transfer length
	tx_count = (nbytes > I2C_NBYTES_MAX) ? I2C_NBYTES_MAX : nbytes;
	nbytes -= tx_count;
	reg |= tx_count << I2C_CR2_NBYTES_Pos;

	// Write a composed value to the I2C register
	I2Cx->CR2 = reg;

	// Transmit data
	while (tx_count) {
		// Wait until either TXIS or NACK flag is set
		wait = delay_val;
		while (!((reg = I2Cx->ISR) & (I2C_ISR_TXIS | I2C_ISR_NACKF)) && --wait);
		if ((reg & I2C_ISR_NACKF) || (wait == 0))  { return I2C_ERROR; }

		// Transmit byte
		I2Cx->TXDR = *pBuf++;
		tx_count--;

		if ((tx_count == 0) && (nbytes != 0)) {
			// Wait until TCR flag is set (Transfer Complete Reload)
			wait = delay_val;
			while (!(I2Cx->ISR & I2C_ISR_TCR) && --wait);
			if (wait == 0) { return I2C_ERROR; }

			// Configure next (or last) portion transfer
			reg = I2Cx->CR2;
			reg &= ~(I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND);
			if ((flags & I2C_TX_CONT) || (nbytes > I2C_NBYTES_MAX)) {
				reg |= I2C_CR2_RELOAD;
			} else {
				if (!(flags & I2C_TX_NOSTOP)) { reg |= I2C_CR2_AUTOEND; }
			}
			tx_count = (nbytes > I2C_NBYTES_MAX) ? I2C_NBYTES_MAX : nbytes;
			nbytes -= tx_count;
			reg |= tx_count << I2C_CR2_NBYTES_Pos;
			I2Cx->CR2 = reg;
		}
	}

	// End of transmission
	wait = delay_val;
	while (!(I2Cx->ISR & (I2C_ISR_TC | I2C_ISR_TCR | I2C_ISR_STOPF)) && --wait);

	return (wait) ? I2C_SUCCESS : I2C_ERROR;
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
	uint32_t rx_count;
	uint32_t delay_val = I2C_CalcDelay(I2C_TIMEOUT);
	volatile uint32_t wait;

	// Clear all flags
	I2Cx->ICR = I2C_ICR_ALL;

	// Everything regarding to the transmission is in the CR2 register
	reg = I2Cx->CR2;
	reg &= ~I2C_CR2_ALL;

	// Configure slave device address, enable START condition and set direction to READ
	reg |= (devAddr & I2C_CR2_SADD) | I2C_CR2_START | I2C_CR2_RD_WRN;

	// Transfer length
	if (nbytes > I2C_NBYTES_MAX) {
		rx_count = I2C_NBYTES_MAX;
		reg |= I2C_CR2_RELOAD;
	} else {
		rx_count = nbytes;
		reg |= I2C_CR2_AUTOEND;
	}
	reg |= rx_count << I2C_CR2_NBYTES_Pos;
	nbytes -= rx_count;

	// Write a composed value to the I2C register
	I2Cx->CR2 = reg;

	// Receive data
	while (rx_count) {
		// Wait until either RXNE or NACK flag is set
		wait = delay_val;
		while (!((reg = I2Cx->ISR) & (I2C_ISR_RXNE | I2C_ISR_NACKF)) && --wait);
		if ((reg & I2C_ISR_NACKF) || (wait == 0)) { return I2C_ERROR; }

		// Read received data
		*pBuf++ = I2Cx->RXDR;
		rx_count--;

		if ((rx_count == 0) && (nbytes != 0)) {
			// Wait until TCR flag is set (Transfer Complete Reload)
			wait = delay_val;
			while (!(I2Cx->ISR & I2C_ISR_TCR) && --wait);
			if (wait == 0) { return I2C_ERROR; }

			// Configure next (or last) portion transfer
			reg = I2Cx->CR2;
			reg &= ~(I2C_CR2_NBYTES | I2C_CR2_AUTOEND | I2C_CR2_RELOAD);
			if (nbytes > I2C_NBYTES_MAX) {
				rx_count = I2C_NBYTES_MAX;
				reg |= I2C_CR2_RELOAD;
			} else {
				rx_count = nbytes;
				reg |= I2C_CR2_AUTOEND;
			}
			reg |= rx_count << I2C_CR2_NBYTES_Pos;
			nbytes -= rx_count;
			I2Cx->CR2 = reg;
		}
	}

	// Wait for the STOP flag
	wait = delay_val;
	while (!(I2Cx->ISR & I2C_ISR_STOPF) && --wait);

	return (wait) ? I2C_SUCCESS : I2C_ERROR;
}
