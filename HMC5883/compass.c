/*
 *  Reads the direction your compass sensor is pointing in degrees
 *  The sensor is pointing North when the X axis marking is pointing North
 *
 *  Place the sensor on a flat surface and avoid anything magnetic that
 *  may skew the results.
 */


#include <Wire.h>
#include "HMC5883Llib.h"


Magnetometer mag;
bool fail;

void setup()
{
    Serial.begin(57600);
    
    if (mag.begin() != 0)
    {
        Serial.println("Error connecting to Magnetometer");
        fail = true;
        return;
    }

    // set the amount of gain - Use the most sensitive
    // for reading the earths magnetic field
    // 
    // MSB/Gauss   Field Range
    // 1370     +- 0.88 Ga
    // 1090     +- 1.3 Ga
    // 820      +- 1.9 Ga
    // 660      +- 2.5 Ga
    // 440      +- 4.0 Ga
    // 390      +- 4.7 Ga
    // 330      +- 5.6 Ga
    // 230      +- 8.1 Ga
    mag.setGain(HMC5833L_GAIN_1370);
}

void loop()
{
    // don't bother reading if we failed to connect
    if (fail)
        return;

    double heading;

    // reads the heading in degrees using the X and Y axis
    int8_t ret = mag.readHeadingDeg(&heading);

    switch (ret)
    {
        case HMC5833L_ERROR_GAINOVERFLOW:
            Serial.println("Gain Overflow");
            return;
        case 0:
            // success
            break;
        default:
            Serial.println("Failed to read Magnetometer");
            return;
    }
    
    // print them out
    Serial.print("Heading: ");
    Serial.print(heading);
    Serial.println(" degrees");

    delay(50);
}