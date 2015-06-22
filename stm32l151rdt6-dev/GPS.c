#include <stm32l1xx_rcc.h>

#include "uart.h"
#include "NMEA.h"
#include "GPS.h"




// FIXME: remove this after debugging
// Debug output
#include <stdio.h>
#include "VCP.h"
#pragma GCC diagnostic ignored "-Wformat"




uint16_t GPS_buf_cntr;                      // Number of actual bytes in GPS buffer
uint8_t GPS_buf[GPS_BUFFER_SIZE];           // Buffer with data from GPS
bool GPS_new_data;                          // TRUE if a new GPS packet received
bool GPS_parsed;                            // TRUE if GPS data was parsed




// Send NMEA sentence
// input:
//   cmd - pointer to the buffer containing sentence
// note: sentence must begin with '$' char and end with '*'
void GPS_Send(char *cmd) {
	uint8_t cmd_CRC;

	cmd_CRC = NMEA_CalcCRC(cmd);
	UART_SendStr(GPS_USART_PORT,cmd);
	UART_SendChar(GPS_USART_PORT,HEX_CHARS[cmd_CRC >> 4]);
	UART_SendChar(GPS_USART_PORT,HEX_CHARS[cmd_CRC & 0x0f]);
	UART_SendChar(GPS_USART_PORT,'\r');
	UART_SendChar(GPS_USART_PORT,'\n');

	// Wait for an USART transmit complete
	while (!(GPS_USART_PORT->SR & USART_SR_TC));
}

// Initialize the GPS module
void GPS_Init(void) {
	uint32_t wait;
	uint32_t baud = 9600;
	uint32_t BC; // FIXME: debug remove this
	uint32_t trials = 5;

	// Reset all GPS related variables
	NMEA_InitData();

	// After first power-on with no backup the Quectel L80 baud rate will be 9600bps
	// After power-on with backup the baud rate remains same as it was before power-off
	// What the hell to do with this shit?

	// Set USART baud rate to 9600pbs and wait some time for a NMEA sentence
	// It must be "$PMTK011,MTKGPS" followed by "$PMTK010,001"
	// In case of timeout we decide what there are no GPS receiver?
	UARTx_SetSpeed(GPS_USART_PORT,baud);

	while (trials--) {
		wait = 0x00300000; // Magic number, about 1.5s on 32MHz CPU
		while (!GPS_new_data && --wait);
		if (wait) {
			// No timeout, USART IDLE frame detected

			// FIXME: Output data for debug purposes
			BC = GPS_buf_cntr;
			VCP_SendBuf(GPS_buf,GPS_buf_cntr);

			// Parse data from GPS receiver
			NMEA_ParseBuf(GPS_buf,&GPS_buf_cntr);
			GPS_new_data = FALSE; // Reset the new GPS data flag (data were parsed)
			GPS_parsed = TRUE; // Set flag indicating what GPS data was parsed

			// FIXME: Output data for debug purposes
			printf("\r\nPMTK: BOOT=%s PMTK010=%u CMD=%u FLAG=%u | B=%u SC=%u/%u/%u [BR=%u] %X\r\n",
					(PMTKData.PMTK_BOOT) ? "TRUE" : "FALSE",
					PMTKData.PMTK010,
					PMTKData.PMTK001_CMD,
					PMTKData.PMTK001_FLAG,
					BC,
					NMEA_sentences_parsed,
					NMEA_sentences_unknown,
					NMEA_sentences_invalid,
					baud,
					wait
					);

			if (baud == 9600) {
				if (NMEA_sentences_parsed) {
					// Known NMEA sentences were detected on 9600 baud rate
					// Send command to the GPS receiver to switch to higher baud rate
					// And do this only after GPS receiver finish the boot sequence
					if ((!PMTKData.PMTK_BOOT) && (PMTKData.PMTK010 != 2)) {
						GPS_Send(PMTK_SET_NMEA_BAUDRATE_38400);
						baud = 38400;
						UARTx_SetSpeed(GPS_USART_PORT,baud);
					}
				} else {
					// Known NMEA sentences were not detected, set USART baud rate to 38400 and try again
					baud = 38400;
					UARTx_SetSpeed(GPS_USART_PORT,baud);
				}
			}

			if (NMEA_sentences_parsed && (baud == 38400)) {
				// Known NMEA sentences were detected on 38400 baud rate, thats's all
				break;
			}
		} else {
			// GPS timeout
			baud = 0;
			break;
		}

		printf("---------------------------------------------\r\n");
	}

	// There is no result after several trials, count this as no GPS present
	if (!trials) baud = 0;

	if (baud) {
		// FIXME: here must be a little delay, before sending PMTK commands!
		printf("--->>> It's time to configure <<<---\r\n");

		// Looks like an initialization completed, configure the GPS receiver
		GPS_Send(PMTK_SET_NMEA_OUTPUT_EFFICIENT); // Efficient sentences only
		GPS_Send(PMTK_SET_AIC_ENABLED); // Enable AIC (enabled by default)
		GPS_Send(PMTK_API_SET_STATIC_NAV_THD_OFF); // Disable speed threshold
		GPS_Send(PMTK_EASY_ENABLE); // Enable EASY (for MT3339)
		GPS_Send(PMTK_SET_PERIODIC_MODE_NORMAL); // Disable periodic mode

		// FIXME: just for debug, remove this
		trials = 4;
		while (trials--) {
			wait = 0x00300000; // Magic number, about 1.5s on 32MHz CPU
			while (!GPS_new_data && --wait);
			if (wait) {
				// No timeout, USART IDLE frame detected

				// Output data for debug purposes
				BC = GPS_buf_cntr;
				VCP_SendBuf(GPS_buf,GPS_buf_cntr);

				// Parse data from GPS receiver
				NMEA_ParseBuf(GPS_buf,&GPS_buf_cntr);
				GPS_new_data = FALSE; // Reset the new GPS data flag (data were parsed)
				GPS_parsed = TRUE; // Set flag indicating what GPS data was parsed

				// Output data for debug purposes
				printf("\r\nPMTK: BOOT=%s PMTK010=%u CMD=%u FLAG=%u | B=%u SC=%u/%u/%u [BR=%u] %X\r\n",
						(PMTKData.PMTK_BOOT) ? "TRUE" : "FALSE",
						PMTKData.PMTK010,
						PMTKData.PMTK001_CMD,
						PMTKData.PMTK001_FLAG,
						BC,
						NMEA_sentences_parsed,
						NMEA_sentences_unknown,
						NMEA_sentences_invalid,
						baud,
						wait
						);
			}
		}

	} else {
		// No proper communication with GPS receiver
		printf("GPS_Init timeout\r\n");
	}

	// FIXME: return some value here, no VOID
}
