/*
    This file is part of Polybox.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef __HEATER_H__
#define __HEATER_H__

#include <inttypes.h>
#include "Repetier.h"


/**
 * State value:
 *  0: stopDecouple();
 *  1: startHoldDecouple(time);
 *  2: 1startFullDecouple(time);
 *
 * */
#define STOP_DECOUPLE 0
#define HOLD_DECOUPLE 1
#define FULL_DECOUPLE 2
#define FOR_CONTINUE  3


#define HTR_OFF 0
#define HTR_PID 1
#define HTR_SLOWBANG 2
#define HTR_DEADTIME 3

class Heater
{
    public:

    int16_t targetTemperature; ///< Target temperature value in units of sensor.
    float targetTemperatureC; ///< Target temperature in degC.
    uint32_t lastTemperatureUpdate; ///< Time in millis of the last temperature update.
    int8_t heatManager; ///< How is temperature controled. 0 = on/off, 1 = PID-Control, 3 = deat time control
#ifdef TEMP_PID
    float tempIState; ///< Temp. var. for PID computation.
    uint8_t pidDriveMax; ///< Used for windup in PID calculation.
    uint8_t pidDriveMin; ///< Used for windup in PID calculation.
#define deadTime pidPGain
    float pidPGain; ///< Pgain (proportional gain) for PID temperature control [0,01 Units].
    float pidIGain; ///< Igain (integral) for PID temperature control [0,01 Units].
    float pidDGain;  ///< Dgain (damping) for PID temperature control [0,01 Units].
    uint8_t pidMax; ///< Maximum PWM value, the heater should be set.
    float tempIStateLimitMax;
    float tempIStateLimitMin;
    uint8_t tempPointer;
    float tempArray[4];
    //uint8_t pin;
#endif

    Heater( ){};
    Heater( float targetTemperatureC,
            int16_t targetTemperature,
            uint32_t lastTemperatureUpdate,
            int8_t heatManager
            #ifdef TEMP_PID
            ,float tempIState,
            uint8_t pidDriveMax,
            uint8_t pidDriveMin,
            float pidPGain,
            float pidIGain,
            float pidDGain,
            uint8_t pidMax,
            float tempIStateLimitMax,
            float tempIStateLimitMin,
            uint8_t tempPointer
            #endif
        ):targetTemperatureC(targetTemperatureC),
            targetTemperature(targetTemperature),
            lastTemperatureUpdate(lastTemperatureUpdate),
            heatManager(heatManager)
            #ifdef TEMP_PID
            ,tempIState(tempIState),
            pidDriveMax(pidDriveMax),
            pidDriveMin(pidDriveMin),
            pidPGain(pidPGain),
            pidIGain(pidIGain),
            pidDGain(pidDGain),
            pidMax(pidMax),
            tempIStateLimitMax(tempIStateLimitMax),
            tempIStateLimitMin(tempIStateLimitMin),
            tempPointer(tempPointer)
            #endif

    {
        #ifdef TEMP_PID
        this->tempArray[0] = 0;
        this->tempArray[1] = 0;
        this->tempArray[2] = 0;
        this->tempArray[3] = 0;
        #endif
       // this->pin = -1;
    };

    ~Heater(){}


    uint8_t computeHeaterOutput(float currentTemperatureC, uint8_t& state, millis_t& t )
    {
       // uint8_t on = currentTemperatureC >= targetTemperatureC ? LOW : HIGH;

        uint8_t on = currentTemperatureC >= targetTemperatureC ? LOW : HIGH;
#if TEMP_PID
        tempArray[tempPointer++] = currentTemperatureC;
        tempPointer &= 3;
        uint8_t output = 0;
        float error = targetTemperatureC - currentTemperatureC;
        if(targetTemperatureC < 20.0f) // heating is off
        {
            output = 0; // off is off, even if damping term wants a heat peak!
            state = STOP_DECOUPLE;
        }
        else if(error > PID_CONTROL_RANGE) // Phase 1: full heating until control range reached
        {
            output = pidMax;
            state = FULL_DECOUPLE;
        }
        else if(error < -PID_CONTROL_RANGE) // control range left upper side!
            output = 0;
        else // control range handle by heat manager
        {
            if(heatManager == HTR_PID)
            {
                state = HOLD_DECOUPLE;
                float pidTerm = pidPGain * error;
                tempIState = constrain(tempIState + error, tempIStateLimitMin, tempIStateLimitMax);
                pidTerm += pidIGain * tempIState * 0.1; // 0.1 = 10Hz
                float dgain = pidDGain * (tempArray[tempPointer] - currentTemperatureC) * 3.333f;
                pidTerm += dgain;
#if SCALE_PID_TO_MAX == 1
                pidTerm = (pidTerm * pidMax) * 0.0039215;
#endif
                output = constrain((int)pidTerm, 0, pidMax);
            }
            else if(heatManager == HTR_DEADTIME)     // dead-time control
            {
                state = HOLD_DECOUPLE;
                float raising = 3.333 * (currentTemperatureC - tempArray[tempPointer]); // raising dT/dt, 3.33 = reciproke of time interval (300 ms)
                tempIState = 0.25 * (3.0 * tempIState + raising); // damp raising
                output = (currentTemperatureC + tempIState * deadTime > targetTemperatureC ? 0 : pidDriveMax);
            }
            else // bang bang and slow bang bang
#endif
                if(heatManager == HTR_SLOWBANG)    // Bang-bang with reduced change frequency to save relais life
                {
                    if (t - lastTemperatureUpdate > HEATED_BED_SET_INTERVAL)
                    {
                        output = (on ? pidMax : 0);
                        lastTemperatureUpdate = t;
                        if(on) state = FULL_DECOUPLE;
                        else state = STOP_DECOUPLE;
                    }
                    else
                    {
                        state = FOR_CONTINUE;
                        return 0;
                    }
                }
                else     // Fast Bang-Bang fallback
                {
                    output = (on ? pidMax : 0);
                    if(on) state = FULL_DECOUPLE;
                    else state = STOP_DECOUPLE;
                }
        } // Temperatur control
#ifdef MAXTEMP
        if(currentTemperatureC > MAXTEMP) // Force heater off if MAXTEMP is exceeded
            output = 0;
#endif

    return output;
}


};

#endif // __HEATER_H__
