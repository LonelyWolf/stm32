# nRF24L01+ Transceiver Library

This "library" is aimed to deal with Nordic nRF24L01+ transceiver.

## Structure
The library basically consists four files:
* nrf24.c
* nrf24.h
* nrf24_hal.c
* nrf24_hal.h

`nrf24.c` and `nrf24.h` - hardware independent part, does all things with the transceiver.

`nrf24_hal.c` and `nrf24_hal.h` - hardware dependent part, provides functions to work with SPI port and GPIO pins.

The user must implement the following functions in the `*_hal.*` part:
* nRF24_LL_RW() - send a byte via SPI and return a received one.
* nRF24_CSN_L() - set CSN pin to LOW state.
* nRF24_CSN_H() - set CSN ping to HIGH state.

Other things such as initialization of SPI or GPIO can also be placed to HAL part. All this at the user discretion. One thing must be done: all hardware must be properly initialized prior to use nRF24 functions.

## Description of some functions
- `nRF24_Init()` - configures nRF24 to its initial state (as after reset), flushes RX/TX FIFOs and clears all pending interrupts. With one exception: the addresses of pipes remains untouched.
- `nRF24_Check()` - check presence of the nRF24. Function configures address length to 5 bytes and writes a predefined TX address (`"nRF24"`), then tries to read it back and compare. If the received value is equal to predefined address, the function returns `1` (nRF24 is present), `0` otherwise. So if software uses address length different from 5 bytes, it must re-program its value after calling this function.
- `nRF24_SetAddr()` - program address for the specified pipe. The first argument - pipe number, it can be from `0` to `5` for RX pipe (one of `nRF24_PIPEx` values) and `6` (or `nRF24_PIPETX`). Second argument must be a pointer to the buffer with address. For TX and RX pipes #0 and #1, the length of the buffer must be equal to a current address length programmed in the nRF24. For RX pipes 2..5 the buffer length can be one byte since these pipe addresses differs from the address of pipe #1 by one last byte (read the datasheet, though).
- `nRF24_DisableAA()` - disable auto-retransmit (a.k.a. ShockBurst) for the specified RX pipe or all pipes at once. Function accepts one parameter, if it is from 0 to 5, then AA will be disabled for the pipe with the specified number. Any other value disable AA for **ALL** RX pipes.

## Sample transceiver configuration
### Receiver, one logic address (pipe#1), without ShockBurst
```tcc
uint8_t ADDR[] = { 'n', 'R', 'F', '2', '4' }; // the address for RX pipe
nRF24_DisableAA(0xFF); // disable ShockBurst
nRF24_SetRFChannel(90); // set RF channel to 2490MHz
nRF24_SetDataRate(nRF24_DR_2Mbps); // 2Mbit/s data rate
nRF24_SetCRCScheme(nRF24_CRC_1byte); // 1-byte CRC scheme
nRF24_SetAddrWidth(5); // address width is 5 bytes
nRF24_SetAddr(nRF24_PIPE1, ADDR); // program pipe address
nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 10); // enable RX pipe#1 with Auto-ACK: disabled, payload length: 10 bytes
nRF24_SetOperationalMode(nRF24_MODE_RX); // switch transceiver to the RX mode
nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
// then pull CE pin to HIGH, and the nRF24 will start a receive...
```

### Receiver, one logic address (pipe#1), with ShockBurst
```tcc
uint8_t ADDR[] = { 'n', 'R', 'F', '2', '4' }; // the address for RX pipe
nRF24_SetRFChannel(90); // set RF channel to 2490MHz
nRF24_SetDataRate(nRF24_DR_2Mbps); // 2Mbit/s data rate
nRF24_SetCRCScheme(nRF24_CRC_1byte); // 1-byte CRC scheme
nRF24_SetAddrWidth(5); // address width is 5 bytes
nRF24_SetAddr(nRF24_PIPE1, ADDR); // program pipe address
nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 10); // enable RX pipe#1 with Auto-ACK: enabled, payload length: 10 bytes
nRF24_SetTXPower(nRF24_TXPWR_0dBm); // configure TX power for Auto-ACK, good choice - same power level as on transmitter
nRF24_SetOperationalMode(nRF24_MODE_RX); // switch transceiver to the RX mode
nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
// then pull CE pin to HIGH, and the nRF24 will start a receive...
```

### Transmiter, without ShockBurst
```tcc
uint8_t ADDR[] = { 'n', 'R', 'F', '2', '4' }; // the TX address
nRF24_DisableAA(0xFF); // disable ShockBurst
nRF24_SetRFChannel(90); // set RF channel to 2490MHz
nRF24_SetDataRate(nRF24_DR_2Mbps); // 2Mbit/s data rate
nRF24_SetCRCScheme(nRF24_CRC_1byte); // 1-byte CRC scheme
nRF24_SetAddrWidth(5); // address width is 5 bytes
nRF24_SetTXPower(nRF24_TXPWR_0dBm); // configure TX power
nRF24_SetAddr(nRF24_PIPETX, ADDR); // program TX address
nRF24_SetOperationalMode(nRF24_MODE_TX); // switch transceiver to the TX mode
nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
// the nRF24 is ready for transmission, upload a payload, then pull CE pin to HIGH and it will transmit a packet...
```

### Transmiter, with ShockBurst
```tcc
uint8_t ADDR[] = { 'n', 'R', 'F', '2', '4' }; // the TX address
nRF24_SetRFChannel(90); // set RF channel to 2490MHz
nRF24_SetDataRate(nRF24_DR_2Mbps); // 2Mbit/s data rate
nRF24_SetCRCScheme(nRF24_CRC_1byte); // 1-byte CRC scheme
nRF24_SetAddrWidth(5); // address width is 5 bytes
nRF24_SetAddr(nRF24_PIPETX, ADDR); // program TX address
nRF24_SetAddr(nRF24_PIPE0, ADDR); // program pipe#0 RX address, must be same as TX (for ACK packets)
nRF24_SetTXPower(nRF24_TXPWR_0dBm); // configure TX power
nRF24_SetAutoRetr(nRF24_ARD_2500us, 10); // configure auto retransmit: 10 retransmissions with pause of 2500s in between
nRF24_EnableAA(nRF24_PIPE0); // enable Auto-ACK for pipe#0 (for ACK packets)
nRF24_SetOperationalMode(nRF24_MODE_TX); // switch transceiver to the TX mode
nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
// the nRF24 is ready for transmission, upload a payload, then pull CE pin to HIGH and it will transmit a packet...
```

## Receive
Configure nRF24 for data receive, pull the CE pin HIGH. After this the transceiver begins to listen ether waiting for data packet. The main software can poll the nRF24 status or for an IRQ. The polling method is far from optimal and if it possible, it is better to use the dedicated IRQ pin.
Simple example code of receiver in polling mode:
```tcc
uint8_t nRF24_payload[32]; // buffer for payload
uint8_t payload_length; // variable to store a length of received payload
uint8_t pipe; // pipe number
nRF24_CE_H(); // start receiving
while (1) {
    // constantly poll the status of RX FIFO...
    if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
        // the RX FIFO have some data, take a note what nRF24 can hold up to three payloads of 32 bytes...
        pipe = nRF24_ReadPayload(nRF24_payload, &payload_length); // read a payload to buffer
        nRF24_ClearIRQFlags(); // clear any pending IRQ bits
        // now the nRF24_payload buffer holds received data
        // payload_length variable holds a length of received data
        // pipe variable holds a number of the pipe which has received the data
        // ... do something with received data ...
    }
}
```

## Transmit
Configure the nRF24 for data transmission, upload a payload, then pull the CE pin HIGH. The transceiver will transmit data packet. The software can wait for interrupt from nRF24 or call `nRF24_GetStatus()` in cycle to get a value of the `STATUS` register and check for `TX_DS` or `MAX_RT` bits.
The `TX_DS` bit means the transmission has been success.
The `MAX_RT` bit means that the maximum number of retransmissions reached and the receiver did not respond with ACK packet.
Then software de-asserts CE pin (pull it LOW), thus disabling transmission mode.
The polling method is far from optimal and if it possible, it is better to use the dedicated IRQ pin.
### Simple example code of transmitter in polling mode:
```tcc
uint8_t status;
nRF24_WritePayload(pBuf, length); // transfer payload data to transceiver
nRF24_CE_H(); // assert CE pin (transmission starts)
while (1) {
    status = nRF24_GetStatus();
    if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
        // transmission ended, exit loop
        break;
    }
}
nRF24_CE_H(); // de-assert CE pin (nRF24 goes to StandBy-I mode)
nRF24_ClearIRQFlags(); // clear any pending IRQ flags
if (status & nRF24_FLAG_MAX_RT) {
    // Auto retransmit counter exceeds the programmed maximum limit (payload in FIFO is not removed)
    // Also the software can flush the TX FIFO here...
    return TX_MAXRT_ERROR;
}
if (status & nRF24_FLAG_TX_DS) {
    // Successful transmission
    return TX_SUCCESS;
}
// In fact that should not happen
return TX_UNKNOWN_ERROR;
```

### Example of handling the transmit with ShockBurst enabled
```tcc
// Here was a transmit procedure... 
status = nRF24_GetStatus(); // get status of nRF24
nRF24_ClearIRQFlags(); // clear any pending IRQ flags
uint8_t otx = nRF24_GetRetransmitCounters(); // get some stats, this variable will contain two counters
uint8_t otx_plos_cnt = (otx & nRF24_MASK_PLOS_CNT) >> 4; // packets lost counter
uint8_t otx_arc_cnt  = (otx & nR24_MASK_ARC_CNT); // auto retransmissions counter
if (otx_arc_cnt > 0) {
    // ARC_CNT is non zero, has been few retransmissions
    // ... do something here with this knowlege ...
    // This counter will be reset when a new transmission starts
}
if (status & nRF24_FLAG_TX_DS) { // success }
if (status & nRF24_FLAG_MAX_RT) {
    // Maximum number of retransmits happened, the lost packets counter must be non zero
    // This counter counts up to 15 and then stops (protection from overflow as datasheet says)
    // For example the software can have some variable as counter for lost packets (e.g. uint32_t lost_pckts)...
    lost_pckts += otx_plos_cnt; // update global counter of lost packets
    nRF24_ResetPLOS(); // reset PLOS_CNT value
}
```

## Demo code
`main.c` contains some demo code. It written for **STM32F103** MCU using the old **SPL**. Just for fun, quite inefficient manner, you've been warned :)
The `main.c` contains five defines which allow user to select a piece of demo which will be compiled:

- `#define DEMO_RX_SINGLE` - receiver with single logic address (one pipe)
Waits for data receive in infinite loop. When a packet is received, dumps it contents to UART.
- `#define DEMO_RX_SINGLE_ESB` - receiver similar to `DEMO_RX_SINGLE` but with Auto-ACK.
- `#define DEMO_RX_MULTI` - receiver with multiple logic addresses (three pipes, each with different payload length)
Waits for data receive in infinite loop. When a packet is received, dumps it contents to UART along with pipe number. Thus software can listen for packets from different transmitters on same frequency channel.
- `#define DEMO_TX_SINGLE` - transmitter with single TX address
Stuffs the payload by the sequence of bytes, calls the transmit function and prints result of transmission to UART. Then waits for a half of second and repeats the transmission.
- `#define DEMO_TX_SINGLE_ESB` - transmitter similar to `DEMO_TX_SINGLE` but with Auto-ACK.
- `#define DEMO_TX_MULTI` - transmitter with multiple TX addresses (three addresses, each with different payload length)
Stuffs the payload by the sequence of bytes, calls the transmit function and prints result of transmission to UART. Then waits for a half of second and repeats the transmission. With one difference: the TX address and length of the payload changed on every transmission.
- `#define DEMO_RX_SOLAR` - dull receiver for the solar powered temperature sensor, useless without sensor but shows receiver with Auto-ACK enabled.

## Note
- Doesn't support a dynamic payload length, maybe this will be added in future.
- Maybe some settings of the nRF24 has been missed, feel free to add them :)
- Any notes, suggestions and swearing are welcome.
