// Code by Blaise Jarrett @ OSEPP http://blaisejarrett.com
// Released to the public domain! Enjoy!

#include "HMC5883Llib.h"
#include <stdio.h>
#include <math.h>
#include <Wire.h>

#define WIRE Wire

#if (ARDUINO >= 100)
 #include <Arduino.h> // capital A so it is error prone on case-sensitive filesystems
#else
 #include <WProgram.h>
#endif

Magnetometer::Magnetometer() : i2cAddr_(0) {}

int8_t Magnetometer::begin()
{
    uint8_t buff[3];
    i2cAddr_ = HMC5833L_I2CADD;

    // Join the I2C bus as master
    WIRE.begin();

    // read the ID registers
    if (i2cReadBytes(HMC5833L_REG_IDA, buff, 3) != 0) 
        return HMC5833L_ERROR_I2CREAD;
    
    // compare the ID registers
    if (buff[0] != HMC5833L_REG_IDA_ID || buff[1] != HMC5833L_REG_IDB_ID
        || buff[2] != HMC5833L_REG_IDC_ID)
        return HMC5833L_ERROR_WRONG_ID;

    // set data rate speed to 30hz
    if (i2cWriteByte(HMC5833L_REG_CFGA, 0x14) != 0)
        return HMC5833L_ERROR_I2CWRITE;

    // set to continuous mode
    // single mode not supported by lib
    if (i2cWriteByte(HMC5833L_REG_MODE, 0) != 0)
        return HMC5833L_ERROR_I2CWRITE;

    // set default gain
    if (setGain(HMC5833L_GAIN_1090) != 0)
        return HMC5833L_ERROR_I2CWRITE;

    return 0;
}

int8_t Magnetometer::setGain(uint8_t gain)
{
    uint8_t data = 0;

    if (gain > 7)
        return HMC5833L_ERROR_GAINOUTOFRANGE;

    gain_ = gain;

    data = gain_ << 5;
    data = data & 0xE0;

    if (i2cWriteByte(HMC5833L_REG_CFGB, data) != 0)
        return HMC5833L_ERROR_I2CWRITE;

    // do a read to apply the new gain
    int16_t x, y, z;
    readRaw(&x, &y, &z);

    return 0;
}

int8_t Magnetometer::readRaw(int16_t * x, int16_t * y, int16_t * z)
{
    if (i2cAddr_ == 0)
        return HMC5833L_ERROR_NOT_INITIALIZED;

    uint8_t data[6];

    if (i2cReadBytes(HMC5833L_REG_DATAXYZ, data, 6) != 0)
        return HMC5833L_ERROR_I2CREAD;

    conv2Byte2Signed16(data[1], data[0], x);
    conv2Byte2Signed16(data[3], data[2], z);
    conv2Byte2Signed16(data[5], data[4], y);

    if (*x == -4096 || *y == -4096 || *z == -4096)
        return HMC5833L_ERROR_GAINOVERFLOW;

    return 0;
}

int8_t Magnetometer::readGauss(double * x, double * y, double * z)
{
    int16_t x_r, y_r, z_r;

    int8_t ret = readRaw(&x_r, &y_r, &z_r);

    // pass the error
    if (ret != 0)
        return ret;

    // convert reading to gauss
    int16_t divisor;
    switch (gain_)
    {
        case HMC5833L_GAIN_1370:
            divisor = 1370;
            break;
        case HMC5833L_GAIN_1090:
            divisor = 1090;
            break;
        case HMC5833L_GAIN_820:
            divisor = 820;
            break;
        case HMC5833L_GAIN_660:
            divisor = 660;
            break;
        case HMC5833L_GAIN_440:
            divisor = 440;
            break;
        case HMC5833L_GAIN_390:
            divisor = 390;
            break;
        case HMC5833L_GAIN_330:
            divisor = 330;
            break;
        case HMC5833L_GAIN_230:
            divisor = 230;
            break;

        default:
            return HMC5833L_ERROR_GAINOUTOFRANGE;
    }

    *x = (double)x_r / divisor;
    *y = (double)y_r / divisor;
    *z = (double)z_r / divisor;

    return 0;
}

int8_t Magnetometer::readHeadingDeg(double * heading)
{
    int16_t x, y, z;

    int8_t ret = readRaw(&x, &y, &z);

    double tan_heading = atan2(y, x);
    // Correct for when signs are reversed.
    if(tan_heading < 0)
        tan_heading += 2 * M_PI;

    // convert to degrees
    *heading = tan_heading * 180 / M_PI; 

    return 0;
}

void Magnetometer::conv2Byte2Signed16(uint8_t LSB, uint8_t MSB, int16_t * dest)
{
    *dest = 0;

    *dest = (int16_t)LSB;

    *dest |= ((int16_t)MSB << 8);
}

int8_t Magnetometer::i2cReadBytes(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (i2cAddr_ == 0)
        return HMC5833L_ERROR_NOT_INITIALIZED;

    WIRE.beginTransmission(i2cAddr_);
    // request a read from this address
    WIRE.write(reg);
    // end to allow the response
    WIRE.endTransmission();

    // Read a byte from the device
    WIRE.requestFrom(i2cAddr_, len); 

    uint8_t i = 0;

    while (WIRE.available())
    {
        data[i] = WIRE.read();
        ++i;

        if (i == len)
            break;
    }

    if (i != len)
        return -1;

    return 0;
}

int8_t Magnetometer::i2cWriteByte(uint8_t reg, uint8_t data)
{
    if (i2cAddr_ == 0)
        return HMC5833L_ERROR_NOT_INITIALIZED;

    // Begin the write sequence 
    WIRE.beginTransmission(i2cAddr_);

    // First byte is to set the register pointer
    WIRE.write(reg);

    // Write the data byte
    WIRE.write(data);

    // End the write sequence; bytes are actually transmitted now 
    WIRE.endTransmission();

    return 0;
}
