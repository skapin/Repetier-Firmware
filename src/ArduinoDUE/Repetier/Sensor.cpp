#include "Sensor.h"

#include "Repetier.h"



Sensor::Sensor(uint8_t sensorType, uint8_t sensorPin, int16_t currentTemperature, float currentTemperatureC):
                        sensorType(sensorType),
                        sensorPin(sensorPin),
                        currentTemperature(currentTemperature),
                        currentTemperatureC(currentTemperatureC)
{
}


void Sensor::updateCurrentTemperature()
{
       uint8_t type = sensorType;
    // get raw temperature
    switch(type)
    {
    case 0:
        currentTemperature = 25;
        break;
#if ANALOG_INPUTS>0
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 97:
    case 98:
    case 99:
        currentTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - (osAnalogInputValues[sensorPin] >> (ANALOG_REDUCE_BITS)); // Convert to 10 bit result
        break;
    case 50: // User defined PTC table
    case 51:
    case 52:
    case 60: // HEATER_USES_AD8495 (Delivers 5mV/degC)
    case 61: // HEATER_USES_AD8495 (Delivers 5mV/degC) 1.25v offset
    case 100: // AD595
        currentTemperature = (osAnalogInputValues[sensorPin] >> (ANALOG_REDUCE_BITS));
        break;
#endif
#ifdef SUPPORT_MAX6675
    case 101: // MAX6675
        currentTemperature = read_max6675(sensorPin);
        break;
#endif
#ifdef SUPPORT_MAX31855
    case 102: // MAX31855
        currentTemperature = read_max31855(sensorPin);
        break;
#endif
    default:
        currentTemperature = 4095; // unknown method, return high value to switch heater off for safety
    }
    int currentTemperature = this->currentTemperature;
    //OUT_P_I_LN("OC for raw ",raw_temp);
    switch(type)
    {
    case 0:
        currentTemperatureC = 25;
        break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    {
        type--;
        uint8_t num = pgm_read_byte(&temptables_num[type])<<1;
        uint8_t i=2;
        const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word_near(&temptables[type]);
        short oldraw = pgm_read_word(&temptable[0]);
        short oldtemp = pgm_read_word(&temptable[1]);
        short newraw,newtemp;
        currentTemperature = (1023<<(2-ANALOG_REDUCE_BITS))-currentTemperature;
        while(i<num)
        {
            newraw = pgm_read_word(&temptable[i++]);
            newtemp = pgm_read_word(&temptable[i++]);
            if (newraw > currentTemperature)
            {
                //OUT_P_I("RC O:",oldtemp);OUT_P_I_LN(" OR:",oldraw);
                //OUT_P_I("RC N:",newtemp);OUT_P_I_LN(" NR:",newraw);
                currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature-oldraw)*(float)(newtemp-oldtemp)/(newraw-oldraw));
                return;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
    }
    break;
    case 50: // User defined PTC thermistor
    case 51:
    case 52:
    {
        type-=46;
        uint8_t num = pgm_read_byte(&temptables_num[type])<<1;
        uint8_t i=2;
        const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word_near(&temptables[type]);
        short oldraw = pgm_read_word(&temptable[0]);
        short oldtemp = pgm_read_word(&temptable[1]);
        short newraw,newtemp;
        while(i<num)
        {
            newraw = pgm_read_word(&temptable[i++]);
            newtemp = pgm_read_word(&temptable[i++]);
            if (newraw > currentTemperature)
            {
                currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature-oldraw)*(float)(newtemp-oldtemp)/(newraw-oldraw));
                return;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
        break;
    }
    case 60: // AD8495 (Delivers 5mV/degC vs the AD595's 10mV)
#if CPU_ARCH == ARCH_AVR
        currentTemperatureC = ((float)currentTemperature * 1000.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));
#else
        currentTemperatureC = ((float)currentTemperature * 660.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));
#endif
        break;
    case 61: // AD8495 1.25V Vref offset (like Adafruit 8495 breakout board)
#if CPU_ARCH == ARCH_AVR
        currentTemperatureC = ((float)currentTemperature * 1000.0f / (1024 << (2 - ANALOG_REDUCE_BITS))) - 250.0f;
#else
        currentTemperatureC = ((float)currentTemperature * 660.0f / (1024 << (2 - ANALOG_REDUCE_BITS))) - 250.0f;
#endif
        break;
    case 100: // AD595
        //return (int)((long)raw_temp * 500/(1024<<(2-ANALOG_REDUCE_BITS)));
#if CPU_ARCH == ARCH_AVR
        currentTemperatureC = ((float)currentTemperature * 500.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));
#else
        currentTemperatureC = ((float)currentTemperature * 330.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));
#endif
        break;
#ifdef SUPPORT_MAX6675
    case 101: // MAX6675
        currentTemperatureC = (float)currentTemperature / 4.0;
        break;
#endif
#ifdef SUPPORT_MAX31855
    case 102: // MAX31855
        currentTemperatureC = (float)currentTemperature / 4.0;
        break;
#endif
#if defined(USE_GENERIC_THERMISTORTABLE_1) || defined(USE_GENERIC_THERMISTORTABLE_2) || defined(USE_GENERIC_THERMISTORTABLE_3)
    case 97:
    case 98:
    case 99:
    {
        uint8_t i=2;
        const short *temptable;
#ifdef USE_GENERIC_THERMISTORTABLE_1
        if(type == 97)
            temptable = (const short *)temptable_generic1;
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_2
        if(type == 98)
            temptable = (const short *)temptable_generic2;
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_3
        if(type == 99)
            temptable = (const short *)temptable_generic3;
#endif
        short oldraw = temptable[0];
        short oldtemp = temptable[1];
        short newraw,newtemp;
        currentTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - currentTemperature;
        while(i<GENERIC_THERM_NUM_ENTRIES*2)
        {
            newraw = temptable[i++];
            newtemp = temptable[i++];
            if (newraw > currentTemperature)
            {
                //OUT_P_I("RC O:",oldtemp);OUT_P_I_LN(" OR:",oldraw);
                //OUT_P_I("RC N:",newtemp);OUT_P_I_LN(" NR:",newraw);
                currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature-oldraw)*(float)(newtemp-oldtemp)/(newraw-oldraw));
                return;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
        break;
    }
#endif
    }
}


int16_t Sensor::temp2unit(float target)
{
    int temp = TEMP_FLOAT_TO_INT(target);
    int16_t targetTemperature = 0;
    uint8_t type = sensorType;
    switch(sensorType)
    {
    case 0:
        targetTemperature = 0;
        break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    {
        type--;
        uint8_t num = pgm_read_byte(&temptables_num[type]) << 1;
        uint8_t i = 2;
        const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word(&temptables[type]);
        short oldraw = pgm_read_word(&temptable[0]);
        short oldtemp = pgm_read_word(&temptable[1]);
        short newraw = 0,newtemp;
        while(i<num)
        {
            newraw = pgm_read_word(&temptable[i++]);
            newtemp = pgm_read_word(&temptable[i++]);
            if (newtemp < temp)
            {
                targetTemperature = (1023 << (2 - ANALOG_REDUCE_BITS))- oldraw + (int32_t)(oldtemp - temp) * (int32_t)(oldraw - newraw) / (oldtemp - newtemp);
                return targetTemperature;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        targetTemperature = (1023<<(2-ANALOG_REDUCE_BITS))-newraw;
        break;
    }
    case 50: // user defined PTC thermistor
    case 51:
    case 52:
    {
        type -= 46;
        uint8_t num = pgm_read_byte(&temptables_num[type]) << 1;
        uint8_t i = 2;
        const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word(&temptables[type]);
        short oldraw = pgm_read_word(&temptable[0]);
        short oldtemp = pgm_read_word(&temptable[1]);
        short newraw = 0,newtemp;
        while(i<num)
        {
            newraw = pgm_read_word(&temptable[i++]);
            newtemp = pgm_read_word(&temptable[i++]);
            if (newtemp > temp)
            {
                targetTemperature = oldraw + (int32_t)(oldtemp-temp) * (int32_t)(oldraw-newraw) / (oldtemp-newtemp);
                return targetTemperature;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        targetTemperature = newraw;
        break;
    }
    case 60: // HEATER_USES_AD8495 (Delivers 5mV/degC)
        targetTemperature = (int)((int32_t)temp * (1024 << (2 - ANALOG_REDUCE_BITS))/ 1000);
        break;
    case 100: // HEATER_USES_AD595
        targetTemperature = (int)((int32_t)temp * (1024 << (2 - ANALOG_REDUCE_BITS))/ 500);
        break;
#ifdef SUPPORT_MAX6675
    case 101:  // defined HEATER_USES_MAX6675
        targetTemperature = temp * 4;
        break;
#endif
#ifdef SUPPORT_MAX31855
    case 102:  // defined HEATER_USES_MAX31855
        targetTemperature = temp * 4;
        break;
#endif
#if defined(USE_GENERIC_THERMISTORTABLE_1) || defined(USE_GENERIC_THERMISTORTABLE_2) || defined(USE_GENERIC_THERMISTORTABLE_3)
    case 97:
    case 98:
    case 99:
    {
        uint8_t i = 2;
        const short *temptable;
#ifdef USE_GENERIC_THERMISTORTABLE_1
        if(type == 97)
            temptable = (const short *)temptable_generic1;
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_2
        if(type == 98)
            temptable = (const short *)temptable_generic2;
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_3
        if(type == 99)
            temptable = (const short *)temptable_generic3;
#endif
        short oldraw = temptable[0];
        short oldtemp = temptable[1];
        short newraw,newtemp;
        while(i<GENERIC_THERM_NUM_ENTRIES*2)
        {
            newraw = temptable[i++];
            newtemp = temptable[i++];
            if (newtemp < temp)
            {
                targetTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - oldraw + (int32_t)(oldtemp-temp) * (int32_t)(oldraw-newraw) / (oldtemp-newtemp);
                return targetTemperature;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        targetTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - newraw;
        break;
    }
#endif
    }
    return targetTemperature;
}
