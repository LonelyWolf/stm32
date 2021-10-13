# TMP116 library

The TMP116 family (TMP116, TMP116N) consist of low-power, high-precision digital temperature sensors with integrated EEPROM memory. It provides a 16-bit temperature result with a resolution of 0.0078°C and an accuracy of up to ±0.2°C with no calibration and has programmable alert functionality. And can be used for devices that need to meet the NIST traceability.

## Features
* TMP116 accuracy without calibration:
  * ±0.2°C (maximum) from –10°C to +85°C
  * ±0.25°C (maximum) from –40°C to +105°C
  * ±0.3°C (maximum) from +105°C to +125°C
* TMP116N accuracy, no calibration needed:
  * ±0.3°C (maximum) from –25°C to +85°C
  * ±0.4°C (maximum) from –40°C to +125°C
* Supply range: 1.9V to 5.5V
* Low Power Consumption: 3.5-µA (1Hz Conversion Cycle), 250nA shutdown current
* Programmable temperature alert limits
* Selectable averaging
* NIST traceability

[More information can be found in the datasheet](https://www.ti.com/lit/gpn/tmp116)

## Functionality

* [x] Sensor configuration
* [x] Read measured temperature
* [x] Read/write from/to internal EEPROM
* [x] Store current configuration and limits to interanl EEPROM

## Porting

The library has the following hardware-dependent functions:
* `__reg_write` - writes value to a specified register
* `__reg_read` - reads value of a specified register
* `__general_call` - transmits an I2C general-call command (not used when `TMP116_RESET` defined as zero)

In order to port the library, in these functions should be implemented hardware layer of the reception and transmission of data on the I2C bus.

## Configuring library

The I2C address of the sensor depends of how the pin `ADD0` connected. This library (in order not to overcomplicate it) can deal with only one sensor on the bus, so it is necessary to specify state of the `ADD0` pin by defining `TMP116_ADDR` with one of the following values:
- `TMP116_ADDR_GND` when `ADD0` is connected to `GND`
- `TMP116_ADDR_VCC` when `ADD0` is connected to `VCC`
- `TMP116_ADDR_SDA` when `ADD0` is connected to `SDA`
- `TMP116_ADDR_SCL` when `ADD0` is connected to `SCL`

## Features

Chip configuration and alert limits can be saved to its EEPROM. In order to do this, configure the chip to a desired state and then call one of the `TMP116_Write*` functions. After soft-reset or power cycle, the chip will read the saved configuration from the EEPROM.

After the supply voltage reaches within the operating range, the chip requires 1.5ms before conversions begin. The user software in such situation can poll `TMP116_IsEEPROMBusy()` waiting while chip becomes ready.

## Notes

* Before using the library, call the `TMP116_Init()` function once. Also call this function if the sensor has been powered-on or soft-reset.
* Since all the bit flags of the chip are located in same register and cleared on reading it, the function `TMP116_GetStatus()` was implemented. It reads the value of this register and stores it in an internal variable. After calling this function, state of the flags can be checked by calling any of the `TMP116_Is*` functions.
* Before calling any function what writes data to the sensor EEPROM, user software must ensure that the EEPROM busy flag is cleared
* After calling any function what writes data to the sensor EEPROM, user software should not work with sensor while the EEPROM busy flag is set
* A software reset of the chip is possible through the I2C general call function. Before using it, make sure that this will not interfere with other devices on the same bus, which also can respond to a general-call command. In order to enable this functionality define `TMP116_RESET` to non-zero value and implement transmission of one byte to the special I2C address `0x00` in `__general_call()` function.
