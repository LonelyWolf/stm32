# SSD1306

Library for monochrome OLED based on SSD1306 controller.

Contains functions for display initialization and configuration.
Uses 1Kbyte of RAM for video buffer (128x64 resolution). All drawing occurs in this buffer and then its contents transmitted to the display.

Display can be rotated to any of the following positions: normal, 180 degrees, clockwise and counter-clockwise. Image will be adopted to display rotation by software (coordinates swapping) and hardware tricks (COM/SEG lines inverting).

Communication through SPI but easily can be modified for I2C.

In main.c presented some stupid demo code.
