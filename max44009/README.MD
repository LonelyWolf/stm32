# Library for the **MAX44009** - the ambient light sensor

## Configuration

Library have a signle configuration definition - `MAX44009_ACCURACY`. In case when it defined as `0`, library will be compiled with the reduced accuracy. That means for Lux readings will be used single I2C transaction and a little less calculations will be made in cost of losing some precision in results. Eny other (non-zero) value will compile normal accuracy. 

## Usage

Include library, initialize I2C and then call `MAX44009_GetLux()` function to get Lux readings. The return of this function is the LUX value, measured in `millilux` (e.g. the value of `147520` represents `147.520` Lux).

## Porting

This library uses only two hardware-dependent functions: `MAX44009_WriteReg()` and `MAX44009_ReadReg()` which communicates with the IC via I2C bus.

`MAX44009_WriteReg()` - writes a value to register of MAX44009. It should be able to transmit two bytes via I2C and then generate a STOP condition.

`MAX44009_ReadReg()` - reads a value of register of MAX44009. It should be able transmit a single byte via I2C, reads a single byte via I2C and then generate a STOP condition.
