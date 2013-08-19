/* Port and pin with DHT22 sensor*/
#define DHT22_GPIO_PORT            GPIOB
#define DHT22_GPIO_CLOCK           RCC_APB2Periph_GPIOB
#define DHT22_GPIO_PIN             GPIO_Pin_0

/* DHT22_GetReadings response codes */
#define DHT22_RCV_OK               0 // Return with no error
#define DHT22_RCV_NO_RESPONSE      1 // No response from sensor
#define DHT22_RCV_BAD_ACK1         2 // Bad first half length of ACK impulse
#define DHT22_RCV_BAD_ACK2         3 // Bad second half length of ACK impulse
#define DHT22_RCV_RCV_TIMEOUT      4 // It was timeout while receiving bits


void DHT22_Init(void);
uint32_t DHT22_GetReadings(void);
uint16_t DHT22_DecodeReadings(void);
uint16_t DHT22_GetHumidity(void);
uint16_t DHT22_GetTemperature(void);
