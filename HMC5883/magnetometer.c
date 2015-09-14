/*
 * Reads the magnetic field in Gauss around the sensor
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

    // set the amount of gain
    // 
    // LSB/Gauss   Field Range
    // 1370     +- 0.88 Ga
    // 1090     +- 1.3 Ga
    // 820      +- 1.9 Ga
    // 660      +- 2.5 Ga
    // 440      +- 4.0 Ga
    // 390      +- 4.7 Ga
    // 330      +- 5.6 Ga
    // 230      +- 8.1 Ga
    mag.setGain(HMC5833L_GAIN_440);
}

void loop()
{
    // don't bother reading if we failed to connect
    if (fail)
        return;

    double x, y, z;

    int8_t ret = mag.readGauss(&x, &y, &z);
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
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Z: ");
    Serial.println(z);

    delay(50);
}