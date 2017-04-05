/**
 * @file
 * @author  Blaise Jarrett <me@blaisejarrett.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * Released to the public domain! Enjoy!
 *
 * @section DESCRIPTION
 *
 * The Magnetometer header for the HMC5883L Magnetometer class.
 */

#ifndef _HMC5833LLIB_H_
#define _HMC5833LLIB_H_

#include <stdint.h>

/** The I2C address of the device */
#define HMC5833L_I2CADD 0x1E

/** Forgot to call begin() */
#define HMC5833L_ERROR_NOT_INITIALIZED -2
/** Device did not respond with the correct ID */
#define HMC5833L_ERROR_WRONG_ID -3
/** I2C read error */
#define HMC5833L_ERROR_I2CREAD -4
/** I2C write error */
#define HMC5833L_ERROR_I2CWRITE -5
/** gain argument must be one of the definitions */
#define HMC5833L_ERROR_GAINOUTOFRANGE -6
/** Reading is out of range, reduce the gain */
#define HMC5833L_ERROR_GAINOVERFLOW -7

// Register Map
#define HMC5833L_REG_CFGA 0
#define HMC5833L_REG_CFGB 1
#define HMC5833L_REG_MODE 2
#define HMC5833L_REG_DATAXYZ 3
#define HMC5833L_REG_IDA 10
#define HMC5833L_REG_IDB 11
#define HMC5833L_REG_IDC 12

#define HMC5833L_REG_IDA_ID 0x48
#define HMC5833L_REG_IDB_ID 0x34
#define HMC5833L_REG_IDC_ID 0x33

/** Gain: +-0.88 Ga or 1370LSB/Ga */
#define HMC5833L_GAIN_1370 0
/** Gain: +-1.3 Ga or 1090LSB/Ga */
#define HMC5833L_GAIN_1090 1
/** Gain: +-1.9 Ga or 820LSB/Ga */
#define HMC5833L_GAIN_820 2
/** Gain: +-2.5 Ga or 660LSB/Ga */
#define HMC5833L_GAIN_660 3
/** Gain: +-4.0 Ga or 440LSB/Ga */
#define HMC5833L_GAIN_440 4
/** Gain: +-4.7 Ga or 390LSB/Ga */
#define HMC5833L_GAIN_390 5
/** Gain: +-5.6 Ga or 330LSB/Ga */
#define HMC5833L_GAIN_330 6
/** Gain: +-8.1 Ga or 230LSB/Ga */
#define HMC5833L_GAIN_230 7

// 1370     +- 0.88 Ga
// 1090     +- 1.3 Ga
// 820      +- 1.9 Ga
// 660      +- 2.5 Ga
// 440      +- 4.0 Ga
// 390      +- 4.7 Ga
// 330      +- 5.6 Ga
// 230      +- 8.1 Ga

/**
 * Represents a HMC5883L bassed Magnetometer sensor.
 * You must call begin before calling any other methods.
 */
class Magnetometer {
    public:
        Magnetometer();

        /**
         * Attenpts to connect to the device. You must call begin before calling any other method.
         *
         * @return Returns 0 if it successfully connects to the Gyroscope sensor. 
         *         Returns any other non-zero number if it fails.
         *         Check HMC5883Llib.h for the various ERROR definitions.
         */
        int8_t begin();

        /**
         * Reads the RAW data from the magnetometer as a signed 16 bit integer.
         * This integer does not represent Gauss. You'll probably 
         * want to use readGauss instead.
         *
         * @param x A pointer to the variable that will get the x raw data recorded to.
         * @param y A pointer to the variable that will get the y raw data recorded to.
         * @param z A pointer to the variable that will get the z raw data recorded to.
         *
         * @return Returns 0 on success.
         *         Returns any other non-zero number if it fails. 
         *         Check HMC5883Llib.h for the various ERROR definitions.
         */
        int8_t readRaw(int16_t * x, int16_t * y, int16_t * z);

        /**
         * Reads the Gauss experienced by the Magnetometer.
         *
         * @param x A pointer to the variable that will get the x data recorded to (in Ga).
         * @param y A pointer to the variable that will get the y data recorded to (in Ga).
         * @param z A pointer to the variable that will get the z data recorded to (in Ga).
         *
         * @return Returns 0 on success.
         *         Returns any other non-zero number if it fails. 
         *         Check HMC5883Llib.h for the various ERROR definitions.
         */
        int8_t readGauss(double * x, double * y, double * z);

        /**
         * The HMC5883L has an adjustable gain to help you fine tune the sensitivity.
         *
         * @param gain The gain definition. Check HMC5883Llib.h for possible arugments.
         *             Defaults range is +-1.3Ga.
         *
         * @return Returns 0 on success.
         *         Returns any other non-zero number if it fails. 
         *         Check HMC5883Llib.h for the various ERROR definitions.
         */
        int8_t setGain(uint8_t gain);

        /**
         * Compass function. Reads the heading in degrees (0-360).
         * Uses the X and Y axes readings to calculate the heading. The sensor must be
         * as level as possible for accurate results.
         *
         * @param heading A pointer to the variable that will get the heading recorded to (in degrees).
         *
         * @return Returns 0 on success.
         *         Returns any other non-zero number if it fails. 
         *         Check HMC5883Llib.h for the various ERROR definitions.
         */
        int8_t readHeadingDeg(double * heading);
    protected:

        /**
         * I2C interface function for reading bytes. Uses the Arduino Wire library.
         *
         * @param reg The first register to read from.
         * @param data The array where the data will be read into.
         * @param len The number of bytes to read.
         *
         * @return Returns 0 on success.
         *         Returns any other non-zero number if it fails. 
         *         Check HMC5883Llib.h for the various ERROR definitions.
         */
        int8_t i2cReadBytes(uint8_t reg, uint8_t * data, uint8_t len);

        /**
         * I2C interface function for writing a single byte. Uses the Arduino Wire library.
         *
         * @param reg The register address to write to.
         * @param data The byte to write.
         *
         * @return Returns 0 on success.
         *         Returns any other non-zero number if it fails. 
         *         Check HMC5883Llib.h for the various ERROR definitions.
         */
        int8_t i2cWriteByte(uint8_t reg, uint8_t data);

        /** The I2C address of the device. */
        uint8_t i2cAddr_;

        /** The gain definition enum of the device. */
        uint8_t gain_;
    private:
        
        /**
         * Combines two bytes into a 16 bit 2s compliment number.
         *
         * @param LSB The LSB 2s compliment data.
         * @param MSB The MSB 2s compliment data.
         * @param dest The signed 16 bit number result.
         *
         * @return Returns 0 on success.
         *         Returns any other non-zero number if it fails. 
         *         Check ADXL345lib.h for the various ERROR definitions.
         */
        void conv2Byte2Signed16(uint8_t LSB, uint8_t MSB, int16_t * dest);
};


#endif