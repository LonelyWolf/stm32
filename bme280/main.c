#include "main.h"




// Ignore 'format' compiler warnings (for printf)
#pragma GCC diagnostic ignored "-Wformat"




uint32_t i,j;

// RAW temperature, pressure and humidity values
int32_t UT,UP,UH;

// Pressure in Q24.8 format
uint32_t press_q24_8;

// Humidity in Q22.10 format
uint32_t hum_q22_10;

// Human-readable temperature, pressure and humidity value
int32_t temperature;
uint32_t pressure;
uint32_t humidity;

// Human-readable altitude value
int32_t altitude;



int main(void) {
	UARTx_Init(USART2,USART_TX,1382400);

	printf("\r\n\r\nSTM32L151RBT6\r\n");

    for (pressure = 0; pressure <= 120000; pressure += 10000) {
        BME280_Pa_to_Alt(pressure);
    }
//    while(1);

    // Initialize the I2C peripheral
	if (I2Cx_Init(I2C2,400000) != I2C_SUCCESS) {
		printf("I2C initialization failed\r\n");
	} else {
		// Check if the I2C device responding
		printf("BME280: ");
		if (I2Cx_IsDeviceReady(I2C2,BME280_ADDR,10) != I2C_SUCCESS) {
			printf("NO RESPOND\r\n");
			while(1);
		} else {
			printf("PRESENT\r\n");
		}
	}

	// Reset the BME280 chip
	BME280_Reset();

	// Chip start-up time is 2ms, must wait until it becomes accessible
	// Instead of using delay function, wait while chip loads the NVM data to image register
	// It loads the NVM data during start-up or before each measurement starts, so
	// wait until chip becomes accessible through the I2C interface and loads the NVM data
	i = 0xfff;
	while (i-- && !(BME280_GetStatus() & BME280_STATUS_IM_UPDATE));
	if (i == 0) {
		// Some banana happens (no respond from the chip after reset or NVM bit stuck forever)
		printf("BME280 timeout\r\n");
		while(1);
	}

	// Get version of the chip (BME280 = 0x60)
	i = BME280_GetVersion();
	printf("ChipID: %02X\r\n",i);

	// Get current status of the chip
	i = BME280_GetStatus();
	printf("Status: [%02X] %s %s\r\n",
			i,
			(i & BME280_STATUS_MEASURING) ? "MEASURING" : "READY",
			(i & BME280_STATUS_IM_UPDATE) ? "NVM_UPDATE" : "NVM_READY"
		);

	// Get current working mode (must be SLEEP after reset)
	i = BME280_GetMode();
	printf("Mode: [%02X] ",i);
	switch (i) {
		case BME280_MODE_SLEEP:
			printf("SLEEP\r\n");
			break;
		case BME280_MODE_FORCED:
			printf("FORCED\r\n");
			break;
		default:
			printf("NORMAL\r\n");
			break;
	}

	// Read calibration values
	i = BME280_Read_Calibration();
	printf("Read calibration: [%02X]\r\n",i);
	printf("\tdig_T1:%u\tdig_T2:%i\tdig_T3:%i\r\n",cal_param.dig_T1,cal_param.dig_T2,cal_param.dig_T3);
	printf("\tdig_P1:%u\tdig_P2:%i\tdig_P3:%i\r\n",cal_param.dig_P1,cal_param.dig_P2,cal_param.dig_P3);
	printf("\tdig_P4:%i\tdig_P5:%i\tdig_P6:%i\r\n",cal_param.dig_P4,cal_param.dig_P5,cal_param.dig_P6);
	printf("\tdig_P7:%i\tdig_P8:%i\tdig_P9:%i\r\n",cal_param.dig_P7,cal_param.dig_P8,cal_param.dig_P9);
	printf("\tdig_H1:%u\tdig_H2:%i\tdig_H3:%u\r\n",cal_param.dig_H1,cal_param.dig_H2,cal_param.dig_H3);
	printf("\tdig_H4:%i\tdig_H5:%i\tdig_H6:%i\r\n",cal_param.dig_H4,cal_param.dig_H5,cal_param.dig_H6);

	// Set normal mode inactive duration (standby time)
	BME280_SetStandby(BME280_STBY_1s);

	// Set IIR filter constant
	BME280_SetFilter(BME280_FILTER_4);

	// Set oversampling for temperature
	BME280_SetOSRST(BME280_OSRS_T_x4);

	// Set oversampling for pressure
	BME280_SetOSRSP(BME280_OSRS_P_x2);

	// Set oversampling for humidity
	BME280_SetOSRSH(BME280_OSRS_H_x1);

	// Get status of measurements
	i  = BME280_ReadReg(BME280_REG_CTRL_MEAS);
	i |= BME280_ReadReg(BME280_REG_CTRL_HUM) << 8;
	printf("Measurements status: %04X\r\n\tTemperature: %s\r\n\tPressure   : %s\r\n\tHumidity   : %s\r\n",
			i,
			(i & BME280_OSRS_T_MSK) ? "ON" : "OFF",
			(i & BME280_OSRS_P_MSK) ? "ON" : "OFF",
			((i >> 8) & BME280_OSRS_H_MSK) ? "ON" : "OFF"
		);

	// Set normal mode (perpetual periodic conversion)
	BME280_SetMode(BME280_MODE_NORMAL);

	// Main loop
	while(1) {
		// Check current status of chip
		i = BME280_GetStatus();
		printf("Status: [%02X] %s %s\r\n",
				i,
				(i & BME280_STATUS_MEASURING) ? "MEASURING" : "READY",
				(i & BME280_STATUS_IM_UPDATE) ? "NVM_UPDATE" : "NVM_READY"
			);

		// Get all raw readings from the chip
		i = BME280_Read_UTPH(&UT,&UP,&UH);
		printf("Raw TPH: %05X %05X %04X [%02X]\r\n",UT,UP,UH,i);

		// Calculate compensated values

		if (UT == 0x80000) {
			// Either temperature measurement is configured as 'skip' or first conversion is not completed yet
			printf("Temperature: no data\r\n");
		} else {
			// Temperature (must be calculated first)
//			UT = 0x84d3c; // test raw value: 25.90C
			temperature = BME280_CalcT(UT);
			printf("Temperature: %i.%02uC\r\n",
					temperature / 100,
					temperature % 100
				);
		}

		if (UH == 0x8000) {
			// Either humidity measurement is configured as 'skip' or first conversion is not completed yet
			printf("Humidity: no data\r\n");
		} else {
			// Humidity
//			UH = 0x7e47; // test raw value: 61.313%RH
			hum_q22_10 = BME280_CalcH(UH);
//			hum_q22_10 = 47445; // test Q22.10 value, output must be 46.333
			// Convert Q22.10 value to integer
			// e.g. Q22.10 value '47445' will be converted to '46333' what represents 46.333
			// Fractional part computed as (frac / 2^10)
			humidity = ((hum_q22_10 >> 10) * 1000) + (((hum_q22_10 & 0x3ff) * 976562) / 1000000);
			printf("Humidity: %u.%03u%%RH\r\n",
					humidity / 1000,
					humidity % 1000
				);
		}

		if (UT == 0x80000) {
			// Either pressure measurement is configured as 'skip' or first conversion is not completed yet
			printf("Pressure: no data\r\n");
		} else {
			// Pressure
//			UP = 0x554d8; // test raw value: 99488.136Pa = 994.881hPa = 746.224mmHg
			press_q24_8 = BME280_CalcP(UP);
//			press_q24_8 = 24674867; // test Q24.8 value, output must be 96386.199
			// Convert Q24.8 value to integer
			// e.g. Q24.8 value '24674867' will be converted to '96386199' what represents 96386.199
			// Fractional part computed as (frac / 2^8)
			pressure = ((press_q24_8 >> 8) * 1000) + (((press_q24_8 & 0xff) * 390625) / 100000);
			// Convert pascals to millimeters of mercury
			j = BME280_Pa_to_mmHg(press_q24_8);
			printf("Pressure: %u.%03uPa = %u.%03uhPa = %u.%03ummHg (FPM: %u.%03ummHg)\r\n",
					pressure / 1000,
					pressure % 1000,
					pressure / 100000,
					(pressure % 100000) / 100,
					(uint32_t)(pressure * 0.00750061683) / 1000, // Floating point calculations to
					(uint32_t)(pressure * 0.00750061683) % 1000, // to compare with fixed point calculations
					j / 1000,
					j % 1000
				);

			altitude = BME280_Pa_to_Alt(pressure / 1000);
			printf("Altitude: %u.%03u\r\n",
					altitude / 1000,
					altitude % 1000
				);
		}

		// Start measurement (if FORCED mode enabled)
//		BME280_SetMode(BME280_MODE_FORCED);

		printf("------------------------\r\n");

		// Dummy delay
		for (i = 0x003FFFFF; i--; ) asm volatile ("nop");
	}
}
