# ST7528

Library for graphical LCD based on **ST7528** controller (16 grayscale levels).

Contains functions for display initialization and configuration.
Uses 8Kbytes of RAM for video buffer (128x128x4-bit). All drawing occurs in this buffer and then its contents transmitted to the display.

Display can be rotated to any of the following positions: normal, 180 degrees, clockwise and counter-clockwise. Image will be adopted to display rotation by software (coordinates swapping) and hardware tricks (COM/SEG lines inverting).

Communication through SPI.

**ST7528** supports two modes:
- 132x128 pixels (mode 0)
- 160x100 pixels (mode 1)
This library supports only mode 0, but easely can be modified to mode 1.

In main.c presented some stupid demo code.
