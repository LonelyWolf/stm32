# Library for CRC peripheral of **STM32L4** MCU (with some tests).

The "library": two files `crc.c` and `crc.h` in the *periph* directory.

The `main.c` contains buffer with test data and provides CRC calculations for the different standards (CRC size and initial values). The results will be printed in two values, the first - is the calculated CRC for the data buffer using the CRC unit, the second - test value calculated with CRC different utilities. Both values must be identical :)
Also note: in word and half-word calculations the input bit reversing is still configured as per-byte, this is done due to features of function `CRC_CalcBuffer()`.

An output of demo code in `main.c` must be something like this:
```
CRC-8 tests:
CCITT    : 0x34 . 0x34
CDMA2000 : 0xCE . 0xCE
WCDMA    : 0xEA . 0xEA
I-CODE   : 0x87 . 0x87
ROHC     : 0x1A . 0x1A
CRC-16 tests:
MODBUS   : 0x3BD1 . 0x3BD1
CDMA2000 : 0x52F7 . 0x52F7
XMODEM   : 0x047B . 0x047B
USB      : 0xC42E . 0xC42E
CRC-32 tests:
MPEG-2   : 0xF5C62CB4 . 0xF5C62CB4
POSIX    : 0xD338A790 . 0xD338A790
CRC-32D  : 0xD2178F40 . 0xD2178F40
XFER     : 0x5D4143CF . 0x5D4143CF
```
