#ifndef SENSOR_H_INCLUDED
#define SENSOR_H_INCLUDED

#include <inttypes.h>


class Sensor
{
    public:
    Sensor(){};
    Sensor(uint8_t sensorType, uint8_t sensorPin, int16_t currentTemperature, float currentTemperatureC);
    ~Sensor(){}
    uint8_t sensorType; ///< Type of temperature sensor.
    uint8_t sensorPin; ///< Pin to read extruder temperature.
    int16_t currentTemperature; ///< Currenttemperature value read from sensor.
    float currentTemperatureC; ///< Current temperature in degC.


    int16_t temp2unit(float target);
    void updateCurrentTemperature();

};











#endif // SENSOR_H_INCLUDED

