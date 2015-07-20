/*
    This file is part of Repetier-Firmware.

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

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#include "Repetier.h"
#include "Heater.h"
#include "Sensor.h"

uint8_t manageMonitor = 0; ///< Temp. we want to monitor with our host. 1+NUM_EXTRUDER is heated bed
unsigned int counterPeriodical = 0;
volatile uint8_t executePeriodical = 0;
uint8_t counter250ms = 25;
#if FEATURE_DITTO_PRINTING
uint8_t Extruder::dittoMode = 0;
#endif
#if MIXING_EXTRUDER > 0
int Extruder::mixingS;
uint8_t Extruder::mixingDir = 10;
uint8_t Extruder::activeMixingExtruder = 0;
#endif // MIXING_EXTRUDER
#ifdef SUPPORT_MAX6675
extern int16_t read_max6675(uint8_t ss_pin);
#endif
#ifdef SUPPORT_MAX31855
extern int16_t read_max31855(uint8_t ss_pin);
#endif

#if ANALOG_INPUTS > 0
const uint8 osAnalogInputChannels[] PROGMEM = ANALOG_INPUT_CHANNELS;
volatile uint osAnalogInputValues[ANALOG_INPUTS];
#endif

#ifdef USE_GENERIC_THERMISTORTABLE_1
short temptable_generic1[GENERIC_THERM_NUM_ENTRIES][2];
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_2
short temptable_generic2[GENERIC_THERM_NUM_ENTRIES][2];
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_3
short temptable_generic3[GENERIC_THERM_NUM_ENTRIES][2];
#endif
/** Makes updates to temperatures and heater state every call.

Is called every 100ms.
*/
static uint8_t extruderTempErrors = 0;

void Extruder::manageTemperatures()
{
#if FEATURE_WATCHDOG
    HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG
    uint8_t errorDetected = 0;
    millis_t d_time = HAL::timeInMilliseconds(); // compare time for decouple tests
    for(uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; controller++)
    {
        TemperatureController *act = tempController[controller];
        // Handle automatic cooling of extruders
        if(controller < NUM_EXTRUDER)
        {
#if ((SHARED_COOLER && NUM_EXTRUDER >= 2 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN) || SHARED_COOLER_BOARD_EXT) && EXT0_EXTRUDER_COOLER_PIN > -1
            if(controller == 0)
            {
                bool enable = false;
                for(uint8_t j = 0; j < NUM_EXTRUDER; j++)
                {
                    if(tempController[j]->currentTemperatureC >= EXTRUDER_FAN_COOL_TEMP || tempController[j]->targetTemperatureC >= EXTRUDER_FAN_COOL_TEMP)
                    {
                        enable = true;
                        break;
                    }
                }
#if SHARED_COOLER_BOARD_EXT
                if(pwm_pos[NUM_EXTRUDER + 1]) enable = true;
#endif
                extruder[0].coolerPWM = (enable ? extruder[0].coolerSpeed : 0);
            }
#else
            if(act->currentTemperatureC < EXTRUDER_FAN_COOL_TEMP && act->targetTemperatureC < EXTRUDER_FAN_COOL_TEMP)
                extruder[controller].coolerPWM = 0;
            else
                extruder[controller].coolerPWM = extruder[controller].coolerSpeed;
#endif // NUM_EXTRUDER
        }
        // do skip temperature control while auto tuning is in progress
        if(controller == autotuneIndex) continue;
#if MIXING_EXTRUDER
        if(controller > 0 && controller < NUM_EXTRUDER) continue; // Mixing extruder only test for ext 0
#endif

        // Get Temperature
        act->updateCurrentTemperature();


        // Check for obvious sensor errors
        if(act->currentTemperatureC < MIN_DEFECT_TEMPERATURE || act->currentTemperatureC > MAX_DEFECT_TEMPERATURE)   // no temp sensor or short in sensor, disable heater
        {
            errorDetected = 1;
            if(extruderTempErrors < 10)    // Ignore short temporary failures
                extruderTempErrors++;
            else
            {
                act->flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
                if(!Printer::isAnyTempsensorDefect())
                {
                    Printer::setAnyTempsensorDefect();
                    reportTempsensorError();
                }
                EVENT_HEATER_DEFECT(controller);
            }
        }
        if(Printer::isAnyTempsensorDefect()) continue;
        uint8_t on = act->currentTemperatureC >= act->targetTemperatureC ? LOW : HIGH;
        // Make a sound if alarm was set on reaching target temperature
        if(!on && act->isAlarm())
        {
            beep(50 * (controller + 1), 3);
            act->setAlarm(false);  //reset alarm
        }

        // Run test if heater and sensor are decoupled
        bool decoupleTestRequired = act->decoupleTestPeriod > 0 && (d_time - act->lastDecoupleTest) > act->decoupleTestPeriod; // time enough for temperature change?
        if(decoupleTestRequired && act->isDecoupleFullOrHold() && Printer::isPowerOn()) // Only test when powered
        {
            if(act->isDecoupleFull()) // Phase 1: Heating fully until target range is reached
            {
                if(act->currentTemperatureC - act->lastDecoupleTemp < DECOUPLING_TEST_MIN_TEMP_RISE)   // failed test
                {
                    extruderTempErrors++;
                    errorDetected = 1;
                    if(extruderTempErrors > 10)   // Ignore short temporary failures
                    {
                        act->flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED;
                        Printer::setAnyTempsensorDefect();
                        UI_ERROR_P(Com::tHeaterDecoupled);
                        Com::printErrorFLN(Com::tHeaterDecoupledWarning);
                        Com::printF(PSTR("Error:Temp. raised to slow. Rise = "),act->currentTemperatureC - act->lastDecoupleTemp);
                        Com::printF(PSTR(" after "),(int32_t)(d_time-act->lastDecoupleTest));
                        Com::printFLN(PSTR(" ms"));
                        EVENT_HEATER_DECOUPLED(controller);
                    }
                }
                else
                {
                    act->stopDecouple();
                    act->startFullDecouple(d_time);
                }
            }
            else     // Phase 2: Holding temperature inside a target corridor
            {
                if(fabs(act->currentTemperatureC - act->targetTemperatureC) > DECOUPLING_TEST_MAX_HOLD_VARIANCE)   // failed test
                {
                    extruderTempErrors++;
                    errorDetected = 1;
                    if(extruderTempErrors > 10)   // Ignore short temporary failures
                    {
                        act->flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED;
                        Printer::setAnyTempsensorDefect();
                        UI_ERROR_P(Com::tHeaterDecoupled);
                        Com::printErrorFLN(Com::tHeaterDecoupledWarning);
                        Com::printF(PSTR("Error:Could not hold temperature "),act->lastDecoupleTemp);
                        Com::printF(PSTR(" measured "),act->currentTemperatureC);
                        Com::printFLN(PSTR(" deg. C"));
                        EVENT_HEATER_DECOUPLED(controller);
                    }
                }
                else
                {
                    act->lastDecoupleTest = d_time - act->decoupleTestPeriod + 1000; // once running test every second
                }
            }
        }
//
        pwm_pos[act->pwmIndex] = act->computeHeaterOutputDecoupled(pwm_pos[act->pwmIndex], d_time); // set pwm signal
#if LED_PIN > -1
        if(act == &Extruder::current->tempControl)
            WRITE(LED_PIN,on);
#endif
    } // for controller

#if EXTRUDER_JAM_CONTROL
    /*    for(fast8_t i = 0; i < NUM_EXTRUDER; i++) {
            if(extruder[i].jamStepsSinceLastSignal > JAM_ERROR_STEPS) {
                if(!extruder[i].isWaitJamStartcount())
                    extruder[i].tempControl.setJammed(true);
                else {
                    extruder[i].jamStepsSinceLastSignal = 0;
                    extruder[i].setWaitJamStartcount(false);
                }
            }
        }*/
#endif // EXTRUDER_JAM_CONTROL


    if(errorDetected == 0 && extruderTempErrors > 0)
        extruderTempErrors--;
    if(Printer::isAnyTempsensorDefect()
#if HAVE_HEATED_BED
            || Extruder::getHeatedBedTemperature() > HEATED_BED_MAX_TEMP + 5
#endif
      )
    {
        for(uint8_t i = 0; i < NUM_TEMPERATURE_LOOPS; i++)
        {
            pwm_pos[tempController[i]->pwmIndex] = 0;
        }
#if defined(KILL_IF_SENSOR_DEFECT) && KILL_IF_SENSOR_DEFECT > 0
        if(!Printer::debugDryrun() && PrintLine::linesCount > 0)    // kill printer if actually printing
        {
#if SDSUPPORT
            sd.stopPrint();
#endif
            Printer::kill(0);
        }
#endif
        Printer::debugLevel |= 8; // Go into dry mode
    }

}

void TemperatureController::waitForTargetTemperature()
{
    if(targetTemperatureC < 30) return;
    if(Printer::debugDryrun()) return;
    millis_t d_time = HAL::timeInMilliseconds();
    while(true)
    {
        if( (HAL::timeInMilliseconds() - d_time) > 1000 )   //Print Temp Reading every 1 second while heating up.
        {
            Commands::printTemperatures();
            d_time = HAL::timeInMilliseconds();
        }
        Commands::checkForPeriodicalActions(true);
        if(fabs(targetTemperatureC - currentTemperatureC) <= 1)
            return;
    }
}

/* For pausing we negate target temperature, so negative value means paused extruder.
Since temp. is negative no heating will occur. */
void Extruder::pauseExtruders()
{
    disableAllExtruderMotors();
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++)
    {
        if(extruder[i].tempControl.targetTemperatureC > 0)
            extruder[i].tempControl.targetTemperatureC = -extruder[i].tempControl.targetTemperatureC;
    }
}

void Extruder::unpauseExtruders()
{
    // activate temperatures
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++)
    {
        if(extruder[i].tempControl.targetTemperatureC < 0)
            extruder[i].tempControl.targetTemperatureC = -extruder[i].tempControl.targetTemperatureC;
    }
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++)
        extruder[i].tempControl.waitForTargetTemperature();
}

#if EXTRUDER_JAM_CONTROL
void TemperatureController::setJammed(bool on)
{
    if(on)
    {
        flags |= TEMPERATURE_CONTROLLER_FLAG_JAM;
        Printer::setInterruptEvent(PRINTER_INTERRUPT_EVENT_JAM_DETECTED, true);
    }
    else flags &= ~(TEMPERATURE_CONTROLLER_FLAG_JAM);
}

void Extruder::markAllUnjammed()
{
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++)
    {
        extruder[i].tempControl.setJammed(false);
        extruder[i].tempControl.setSlowedDown(false);
        extruder[i].resetJamSteps();
    }
    if(Printer::feedrateMultiply == JAM_SLOWDOWN_TO)
        Commands::changeFeedrateMultiply(100);
    Printer::unsetAnyTempsensorDefect(); // stop alarm
    Com::printInfoFLN(PSTR("Marked all extruders as unjammed."));
    Printer::setUIErrorMessage(false);
}

void Extruder::resetJamSteps()
{
    jamStepsOnSignal = jamStepsSinceLastSignal;
    jamStepsSinceLastSignal = 0;
    Printer::setInterruptEvent(PRINTER_INTERRUPT_EVENT_JAM_SIGNAL0 + id, false);
}
#endif

void Extruder::initHeatedBed()
{
#if HAVE_HEATED_BED
 #if HEATER_BED_0 > -1
    VPIN_MODE(HEATER_BED_0, OUTPUT |PIN_TYPE_ANALOG);
 #endif
 #if HEATER_BED_1 > -1
    VPIN_MODE(HEATER_BED_1, OUTPUT |PIN_TYPE_ANALOG);
 #endif
 #if HEATER_BED_2 > -1
    VPIN_MODE(HEATER_BED_2, OUTPUT |PIN_TYPE_ANALOG);
 #endif
 #if HEATER_BED_3 > -1
    VPIN_MODE(HEATER_BED_3, OUTPUT |PIN_TYPE_ANALOG);
 #endif

 #ifdef TEMP_PID
    for( byte i = 0; i< HEATED_BED_NUM ; ++i )
    {
        heatedBedController[i].updateTempControlVars();
    }
 #endif
#endif
}

#if defined(USE_GENERIC_THERMISTORTABLE_1) || defined(USE_GENERIC_THERMISTORTABLE_2) || defined(USE_GENERIC_THERMISTORTABLE_3)
void createGenericTable(short table[GENERIC_THERM_NUM_ENTRIES][2],short minTemp,short maxTemp,float beta,float r0,float t0,float r1,float r2)
{
    t0 += 273.15f;
    float rs, vs;
    if(r1==0)
    {
        rs = r2;
        vs = GENERIC_THERM_VREF;
    }
    else
    {
        vs =static_cast<float>((GENERIC_THERM_VREF * r1) / (r1 + r2));
        rs = (r2 * r1) / (r1 + r2);
    }
    float k = r0 * exp(-beta / t0);
    float delta = (maxTemp-minTemp) / (GENERIC_THERM_NUM_ENTRIES - 1.0f);
    for(uint8_t i = 0; i < GENERIC_THERM_NUM_ENTRIES; i++)
    {
#if FEATURE_WATCHDOG
        HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG
        float t = maxTemp - i * delta;
        float r = exp(beta / (t + 272.65)) * k;
        float v = 4092 * r * vs / ((rs + r) * GENERIC_THERM_VREF);
        int adc = static_cast<int>(v);
        t *= 8;
        if(adc > 4092) adc = 4092;
        table[i][0] = (adc >> (ANALOG_REDUCE_BITS));
        table[i][1] = static_cast<int>(t);
#ifdef DEBUG_GENERIC
        Com::printF(Com::tGenTemp,table[i][0]);
        Com::printFLN(Com::tComma,table[i][1]);
#endif
    }
}
#endif

/** \brief Initalizes all extruder.

Updates the pin configuration needed for the extruder and activates extruder 0.
Starts a interrupt based analog input reader, which is used by simple thermistors
for temperature reading.
*/
void Extruder::initExtruder()
{
    uint8_t i;
    Extruder::current = &extruder[0];
#ifdef USE_GENERIC_THERMISTORTABLE_1
    createGenericTable(temptable_generic1,GENERIC_THERM1_MIN_TEMP,GENERIC_THERM1_MAX_TEMP,GENERIC_THERM1_BETA,GENERIC_THERM1_R0,GENERIC_THERM1_T0,GENERIC_THERM1_R1,GENERIC_THERM1_R2);
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_2
    createGenericTable(temptable_generic2,GENERIC_THERM2_MIN_TEMP,GENERIC_THERM2_MAX_TEMP,GENERIC_THERM2_BETA,GENERIC_THERM2_R0,GENERIC_THERM2_T0,GENERIC_THERM2_R1,GENERIC_THERM2_R2);
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_3
    createGenericTable(temptable_generic3,GENERIC_THERM3_MIN_TEMP,GENERIC_THERM3_MAX_TEMP,GENERIC_THERM3_BETA,GENERIC_THERM3_R0,GENERIC_THERM3_T0,GENERIC_THERM3_R1,GENERIC_THERM3_R2);
#endif
#if defined(EXT0_STEP_PIN) && EXT0_STEP_PIN > -1 && NUM_EXTRUDER > 0
    SET_OUTPUT(EXT0_DIR_PIN);
    SET_OUTPUT(EXT0_STEP_PIN);
#endif
#if defined(EXT1_STEP_PIN) && EXT1_STEP_PIN > -1 && NUM_EXTRUDER > 1
    SET_OUTPUT(EXT1_DIR_PIN);
    SET_OUTPUT(EXT1_STEP_PIN);
#endif
#if defined(EXT2_STEP_PIN) && EXT2_STEP_PIN > -1 && NUM_EXTRUDER > 2
    SET_OUTPUT(EXT2_DIR_PIN);
    SET_OUTPUT(EXT2_STEP_PIN);
#endif
#if defined(EXT3_STEP_PIN) && EXT3_STEP_PIN > -1 && NUM_EXTRUDER > 3
    SET_OUTPUT(EXT3_DIR_PIN);
    SET_OUTPUT(EXT3_STEP_PIN);
#endif
#if defined(EXT4_STEP_PIN) && EXT4_STEP_PIN > -1 && NUM_EXTRUDER > 4
    SET_OUTPUT(EXT4_DIR_PIN);
    SET_OUTPUT(EXT4_STEP_PIN);
#endif
#if defined(EXT5_STEP_PIN) && EXT5_STEP_PIN > -1 && NUM_EXTRUDER > 5
    SET_OUTPUT(EXT5_DIR_PIN);
    SET_OUTPUT(EXT5_STEP_PIN);
#endif

    for(i = 0; i < NUM_EXTRUDER; ++i)
    {
        Extruder *act = &extruder[i];
        if(act->enablePin > -1)
        {
            HAL::pinMode(act->enablePin, OUTPUT);
            HAL::digitalWrite(act->enablePin,!act->enableOn);
        }
        act->tempControl.lastTemperatureUpdate = HAL::timeInMilliseconds();
#if defined(SUPPORT_MAX6675) || defined(SUPPORT_MAX31855)
        if(act->tempControl.sensorType == 101 || act->tempControl.sensorType == 102)
        {
            WRITE(SCK_PIN, 0);
            SET_OUTPUT(SCK_PIN);
            WRITE(MOSI_PIN, 1);
            SET_OUTPUT(MOSI_PIN);
            WRITE(MISO_PIN, 1);
            SET_INPUT(MISO_PIN);
            SET_OUTPUT(SS);
            WRITE(SS, HIGH);
            HAL::digitalWrite(act->tempControl.sensorPin, 1);
            HAL::pinMode(act->tempControl.sensorPin, OUTPUT);
        }
#endif
    }
#if HEATED_BED_HEATER_PIN > -1
    SET_OUTPUT(HEATED_BED_HEATER_PIN);
    WRITE(HEATED_BED_HEATER_PIN, HEATER_PINS_INVERTED);
    Extruder::initHeatedBed();
#endif
    HAL::analogStart();

}

void TemperatureController::updateTempControlVars()
{
#if TEMP_PID
    if(heatManager == HTR_PID && pidIGain != 0)   // prevent division by zero
    {
        tempIStateLimitMax = (float)pidDriveMax * 10.0f / pidIGain;
        tempIStateLimitMin = (float)pidDriveMin * 10.0f / pidIGain;
    }
#endif
}

/** \brief Select extruder ext_num.

This function changes and initalizes a new extruder. This is also called, after the eeprom values are changed.
*/
void Extruder::selectExtruderById(uint8_t extruderId)
{
#if NUM_EXTRUDER > 0
#if MIXING_EXTRUDER
    if(extruderId >= VIRTUAL_EXTRUDER)
        extruderId = 0;
    activeMixingExtruder = extruderId;
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
        Extruder::setMixingWeight(i, extruder[i].virtualWeights[extruderId]);
    extruderId = 0;
#endif
    if(extruderId >= NUM_EXTRUDER)
        extruderId = 0;
#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
    bool executeSelect = false;
    if(extruderId != Extruder::current->id)
    {
        GCode::executeFString(Extruder::current->deselectCommands);
        executeSelect = true;
    }
    Commands::waitUntilEndOfAllMoves();
#endif
    Extruder::current->extrudePosition = Printer::currentPositionSteps[E_AXIS];
    Extruder::current = &extruder[extruderId];
#ifdef SEPERATE_EXTRUDER_POSITIONS
    // Use seperate extruder positions only if beeing told. Slic3r e.g. creates a continuous extruder position increment
    Printer::currentPositionSteps[E_AXIS] = Extruder::current->extrudePosition;
#endif
    Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS];
    Printer::axisStepsPerMM[E_AXIS] = Extruder::current->stepsPerMM;
    Printer::invAxisStepsPerMM[E_AXIS] = 1.0f / Printer::axisStepsPerMM[E_AXIS];
    Printer::maxFeedrate[E_AXIS] = Extruder::current->maxFeedrate;
//   max_start_speed_units_per_second[E_AXIS] = Extruder::current->maxStartFeedrate;
    Printer::maxAccelerationMMPerSquareSecond[E_AXIS] = Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS] = Extruder::current->maxAcceleration;
    Printer::maxTravelAccelerationStepsPerSquareSecond[E_AXIS] =
        Printer::maxPrintAccelerationStepsPerSquareSecond[E_AXIS] = Printer::maxAccelerationMMPerSquareSecond[E_AXIS] * Printer::axisStepsPerMM[E_AXIS];
#if USE_ADVANCE
    Printer::maxExtruderSpeed = (uint8_t)floor(HAL::maxExtruderTimerFrequency() / (Extruder::current->maxFeedrate*Extruder::current->stepsPerMM));
    if(Printer::maxExtruderSpeed > 15) Printer::maxExtruderSpeed = 15;
    float maxdist = Extruder::current->maxFeedrate * Extruder::current->maxFeedrate * 0.00013888 / Extruder::current->maxAcceleration;
    maxdist -= Extruder::current->maxStartFeedrate * Extruder::current->maxStartFeedrate * 0.5 / Extruder::current->maxAcceleration;
    float fmax = ((float)HAL::maxExtruderTimerFrequency() / ((float)Printer::maxExtruderSpeed * Printer::axisStepsPerMM[E_AXIS])); // Limit feedrate to interrupt speed
    if(fmax < Printer::maxFeedrate[E_AXIS]) Printer::maxFeedrate[E_AXIS] = fmax;
#endif
    Extruder::current->tempControl.updateTempControlVars();
    float cx, cy, cz;
    Printer::realPosition(cx, cy, cz);
    float oldfeedrate = Printer::feedrate;
    Printer::offsetX = -Extruder::current->xOffset * Printer::invAxisStepsPerMM[X_AXIS];
    Printer::offsetY = -Extruder::current->yOffset * Printer::invAxisStepsPerMM[Y_AXIS];
    Printer::offsetZ = -Extruder::current->zOffset * Printer::invAxisStepsPerMM[Z_AXIS];
    Commands::changeFlowrateMultiply(Printer::extrudeMultiply); // needed to adjust extrusionFactor to possibly different diameter
    if(Printer::isHomed())
        Printer::moveToReal(cx, cy, cz, IGNORE_COORDINATE, Printer::homingFeedrate[X_AXIS]);
    Printer::feedrate = oldfeedrate;
    Printer::updateCurrentPosition();
#if USE_ADVANCE
    HAL::resetExtruderDirection();
#endif

#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
    if(executeSelect) // Run only when changing
        GCode::executeFString(Extruder::current->selectCommands);
#endif
#endif
}

void Extruder::setTemperatureForExtruder(float temperatureInCelsius, uint8_t extr, bool beep, bool wait)
{
#if MIXING_EXTRUDER
    extr = 0; // map any virtual extruder number to 0
#endif // MIXING_EXTRUDER
    bool alloffs = true;
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
        if(tempController[i]->targetTemperatureC > 15) alloffs = false;
#ifdef MAXTEMP
    if(temperatureInCelsius > MAXTEMP) temperatureInCelsius = MAXTEMP;
#endif
    if(temperatureInCelsius < 0) temperatureInCelsius = 0;
    TemperatureController *tc = tempController[extr];
    if(tc->sensorType == 0) temperatureInCelsius = 0;
    //if(temperatureInCelsius==tc->targetTemperatureC) return;
    tc->setTargetTemperature(temperatureInCelsius);
    tc->updateTempControlVars();
    if(beep && temperatureInCelsius > MAX_ROOM_TEMPERATURE)
        tc->setAlarm(true);
    if(temperatureInCelsius >= EXTRUDER_FAN_COOL_TEMP) extruder[extr].coolerPWM = extruder[extr].coolerSpeed;
    Com::printF(Com::tTargetExtr,extr,0);
    Com::printFLN(Com::tColon,temperatureInCelsius,0);
#if FEATURE_DITTO_PRINTING
    if(Extruder::dittoMode && extr == 0)
    {
        TemperatureController *tc2 = tempController[1];
        tc2->setTargetTemperature(temperatureInCelsius);
        if(temperatureInCelsius >= EXTRUDER_FAN_COOL_TEMP) extruder[1].coolerPWM = extruder[1].coolerSpeed;
#if NUM_EXTRUDER > 2
        if(Extruder::dittoMode > 1 && extr == 0)
        {
            TemperatureController *tc2 = tempController[2];
            tc2->setTargetTemperature(temperatureInCelsius);
            if(temperatureInCelsius >= EXTRUDER_FAN_COOL_TEMP) extruder[2].coolerPWM = extruder[2].coolerSpeed;
        }
#endif
#if NUM_EXTRUDER > 3
        if(Extruder::dittoMode > 2 && extr == 0)
        {
            TemperatureController *tc2 = tempController[3];
            tc2->setTargetTemperature(temperatureInCelsius);
            if(temperatureInCelsius >= EXTRUDER_FAN_COOL_TEMP) extruder[3].coolerPWM = extruder[3].coolerSpeed;
        }
#endif
    }
#endif // FEATURE_DITTO_PRINTING
    if(wait && temperatureInCelsius > MAX_ROOM_TEMPERATURE
#if defined(SKIP_M109_IF_WITHIN) && SKIP_M109_IF_WITHIN > 0
            && !(abs(tc->currentTemperatureC - tc->targetTemperatureC) < (SKIP_M109_IF_WITHIN))// Already in range
#endif
      )
    {
        Extruder *actExtruder = &extruder[extr];
        UI_STATUS_UPD(UI_TEXT_HEATING_EXTRUDER);
        EVENT_WAITING_HEATER(actExtruder->id);
        bool dirRising = actExtruder->tempControl.targetTemperature > actExtruder->tempControl.currentTemperature;
        millis_t printedTime = HAL::timeInMilliseconds();
        millis_t waituntil = 0;
#if RETRACT_DURING_HEATUP
        uint8_t retracted = 0;
#endif
        millis_t currentTime;
        do
        {
            previousMillisCmd = currentTime = HAL::timeInMilliseconds();
            if( (currentTime - printedTime) > 1000 )   //Print Temp Reading every 1 second while heating up.
            {
                Commands::printTemperatures();
                printedTime = currentTime;
            }
            Commands::checkForPeriodicalActions(true);
            //gcode_read_serial();
#if RETRACT_DURING_HEATUP
            if (actExtruder == Extruder::current && actExtruder->waitRetractUnits > 0 && !retracted && dirRising && actExtruder->tempControl.currentTemperatureC > actExtruder->waitRetractTemperature)
            {
                PrintLine::moveRelativeDistanceInSteps(0, 0, 0, -actExtruder->waitRetractUnits * Printer::axisStepsPerMM[E_AXIS], actExtruder->maxFeedrate / 4, false, false);
                retracted = 1;
            }
#endif
            if((waituntil == 0 &&
                    (dirRising ? actExtruder->tempControl.currentTemperatureC >= actExtruder->tempControl.targetTemperatureC - 1
                     : actExtruder->tempControl.currentTemperatureC <= actExtruder->tempControl.targetTemperatureC + 1))
#if defined(TEMP_HYSTERESIS) && TEMP_HYSTERESIS>=1
                    || (waituntil != 0 && (abs(actExtruder->tempControl.currentTemperatureC - actExtruder->tempControl.targetTemperatureC)) > TEMP_HYSTERESIS)
#endif
              )
            {
                waituntil = currentTime + 1000UL*(millis_t)actExtruder->watchPeriod; // now wait for temp. to stabalize
            }
        }
        while(waituntil == 0 || (waituntil != 0 && (millis_t)(waituntil - currentTime) < 2000000000UL));
#if RETRACT_DURING_HEATUP
        if (retracted && actExtruder == Extruder::current)
        {
            PrintLine::moveRelativeDistanceInSteps(0, 0, 0, actExtruder->waitRetractUnits * Printer::axisStepsPerMM[E_AXIS], actExtruder->maxFeedrate / 4, false, false);
        }
#endif
        EVENT_HEATING_FINISHED(actExtruder->id);
    }
    UI_CLEAR_STATUS;

    bool alloff = true;
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
        if(tempController[i]->targetTemperatureC > 15) alloff = false;
#if EEPROM_MODE != 0
    if(alloff && !alloffs) // All heaters are now switched off?
        EEPROM::updatePrinterUsage();
#endif
    if(alloffs && !alloff)   // heaters are turned on, start measuring printing time
    {
        Printer::msecondsPrinting = HAL::timeInMilliseconds();
        Printer::filamentPrinted = 0;  // new print, new counter
        Printer::flag2 &= ~PRINTER_FLAG2_RESET_FILAMENT_USAGE;
    }
}
void Extruder::setHeatedBedTemperatureById(float temperatureInCelsius, uint8_t bed_id, bool beep)
{
#if HAVE_HEATED_BED
    if ( bed_id >= HEATED_BED_NUM || bed_id < 0 )  // out of range/index i.e bad bed-id
        return;
    /* @todo
        if ( !bed_detected( bed_id ) ) // bed not detected, useless to heat.
        return;
        * */
    if(temperatureInCelsius>HEATED_BED_MAX_TEMP) temperatureInCelsius = HEATED_BED_MAX_TEMP;
    if(temperatureInCelsius<0) temperatureInCelsius = 0;
    if(heatedBedController[bed_id].targetTemperatureC==temperatureInCelsius) return; // don't flood log with messages if killed
    heatedBedController[bed_id].setTargetTemperature(temperatureInCelsius);
    if(beep && temperatureInCelsius>30) heatedBedController[bed_id].setAlarm(true);
    Com::printFLN(Com::tTargetBedColon,heatedBedController[bed_id].targetTemperatureC,0);
    if(temperatureInCelsius > 15)
        pwm_pos[NUM_EXTRUDER + 1] = 255;    // turn on the mainboard cooling fan
    else if(Printer::areAllSteppersDisabled())
        pwm_pos[NUM_EXTRUDER + 1] = 0;      // turn off the mainboard cooling fan only if steppers disabled
#endif

}

void Extruder::setHeatedBedTemperature(float temperatureInCelsius, bool beep)
{
    #if HAVE_HEATED_BED
    for(uint8_t i=0; i<HEATED_BED_NUM; ++i)
    {
        Extruder::setHeatedBedTemperatureById( temperatureInCelsius, i, beep );
    }
#endif
}

float Extruder::getHeatedBedTemperature()
{
#if HAVE_HEATED_BED
    TemperatureController *c = tempController[NUM_TEMPERATURE_LOOPS-1];
    return c->currentTemperatureC;
#else
    return -1;
#endif
}

#if MIXING_EXTRUDER > 0
void Extruder::setMixingWeight(uint8_t extr,int weight)
{
    uint8_t i;
    mixingS = 0;
    extruder[extr].mixingW = weight;
    for(i=0; i<NUM_EXTRUDER; i++)
    {
        extruder[i].mixingE = extruder[i].mixingW;
        mixingS += extruder[i].mixingW;
    }
}
void Extruder::step()
{
    uint8_t best = 255,i;
    int bestError;
    if(mixingDir)
    {
        bestError = -10000;
        for(i = 0; i < NUM_EXTRUDER; i++)
        {
            if(extruder[i].mixingW == 0) continue;
            if(extruder[i].mixingE > bestError)
            {
                bestError = extruder[i].mixingE;
                best = i;
            }
            extruder[i].mixingE += extruder[i].mixingW;
        }
        if(best == 255) return; // no extruder has weight!
        extruder[best].mixingE -= mixingS;
    }
    else
    {
        bestError = 10000;
        for(i = 0; i < NUM_EXTRUDER; i++)
        {
            if(extruder[i].mixingW == 0) continue;
            if(extruder[i].mixingE < bestError)
            {
                bestError = extruder[i].mixingE;
                best = i;
            }
            extruder[i].mixingE -= extruder[i].mixingW;
        }
        if(best == 255) return; // no extruder has weight!
        extruder[best].mixingE += mixingS;
    }
#if NUM_EXTRUDER > 0
    if(best == 0)
    {
        WRITE(EXT0_STEP_PIN, HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT0_JAM_PIN) && EXT0_JAM_PIN > -1
        TEST_EXTRUDER_JAM(0)
#endif
    }
#endif
#if NUM_EXTRUDER > 1
    if(best == 1)
    {
        WRITE(EXT1_STEP_PIN, HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT1_JAM_PIN) && EXT1_JAM_PIN > -1
        TEST_EXTRUDER_JAM(1)
#endif
    }
#endif
#if NUM_EXTRUDER > 2
    if(best == 2)
    {
        WRITE(EXT2_STEP_PIN, HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT2_JAM_PIN) && EXT2_JAM_PIN > -1
        TEST_EXTRUDER_JAM(2)
#endif
    }
#endif
#if NUM_EXTRUDER > 3
    if(best == 3)
    {
        WRITE(EXT3_STEP_PIN, HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT3_JAM_PIN) && EXT3_JAM_PIN > -1
        TEST_EXTRUDER_JAM(3)
#endif
    }
#endif
#if NUM_EXTRUDER > 4
    if(best == 4)
    {
        WRITE(EXT4_STEP_PIN, HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT4_JAM_PIN) && EXT4_JAM_PIN > -1
        TEST_EXTRUDER_JAM(4)
#endif
    }
#endif
#if NUM_EXTRUDER > 5
    if(best == 5)
    {
        WRITE(EXT5_STEP_PIN, HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT5_JAM_PIN) && EXT5_JAM_PIN > -1
        TEST_EXTRUDER_JAM(5)
#endif
    }
#endif
}

void Extruder::unstep()
{
#if NUM_EXTRUDER > 0
    WRITE(EXT0_STEP_PIN, LOW);
#endif
#if NUM_EXTRUDER > 1
    WRITE(EXT1_STEP_PIN, LOW);
#endif
#if NUM_EXTRUDER > 2
    WRITE(EXT2_STEP_PIN, LOW);
#endif
#if NUM_EXTRUDER > 3
    WRITE(EXT3_STEP_PIN, LOW);
#endif
#if NUM_EXTRUDER > 4
    WRITE(EXT4_STEP_PIN, LOW);
#endif
#if NUM_EXTRUDER > 5
    WRITE(EXT5_STEP_PIN, LOW);
#endif
}

void Extruder::setDirection(uint8_t dir)
{
    mixingDir = dir;
#if NUM_EXTRUDER > 0
    if(dir)
        WRITE(EXT0_DIR_PIN,!EXT0_INVERSE);
    else
        WRITE(EXT0_DIR_PIN,EXT0_INVERSE);
    RESET_EXTRUDER_JAM(0, dir)
#endif
#if defined(EXT1_DIR_PIN) && NUM_EXTRUDER > 1
    if(dir)
        WRITE(EXT1_DIR_PIN,!EXT1_INVERSE);
    else
        WRITE(EXT1_DIR_PIN,EXT1_INVERSE);
    RESET_EXTRUDER_JAM(1, dir)
#endif
#if defined(EXT2_DIR_PIN) && NUM_EXTRUDER > 2
    if(dir)
        WRITE(EXT2_DIR_PIN,!EXT2_INVERSE);
    else
        WRITE(EXT2_DIR_PIN,EXT2_INVERSE);
    RESET_EXTRUDER_JAM(2, dir)
#endif
#if defined(EXT3_DIR_PIN) && NUM_EXTRUDER > 3
    if(dir)
        WRITE(EXT3_DIR_PIN,!EXT3_INVERSE);
    else
        WRITE(EXT3_DIR_PIN,EXT3_INVERSE);
    RESET_EXTRUDER_JAM(3, dir)
#endif
#if defined(EXT4_DIR_PIN) && NUM_EXTRUDER > 4
    if(dir)
        WRITE(EXT4_DIR_PIN,!EXT4_INVERSE);
    else
        WRITE(EXT4_DIR_PIN,EXT4_INVERSE);
    RESET_EXTRUDER_JAM(4, dir)
#endif
#if defined(EXT5_DIR_PIN) && NUM_EXTRUDER > 5
    if(dir)
        WRITE(EXT5_DIR_PIN,!EXT5_INVERSE);
    else
        WRITE(EXT5_DIR_PIN,EXT5_INVERSE);
    RESET_EXTRUDER_JAM(5, dir)
#endif
}

void Extruder::enable()
{
#if NUM_EXTRUDER > 0 && defined(EXT0_ENABLE_PIN) && EXT0_ENABLE_PIN > -1
    WRITE(EXT0_ENABLE_PIN, EXT0_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 1 && defined(EXT1_ENABLE_PIN) && EXT1_ENABLE_PIN > -1
    WRITE(EXT1_ENABLE_PIN, EXT1_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 2 && defined(EXT2_ENABLE_PIN) && EXT2_ENABLE_PIN > -1
    WRITE(EXT2_ENABLE_PIN, EXT2_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 3 && defined(EXT3_ENABLE_PIN) && EXT3_ENABLE_PIN > -1
    WRITE(EXT3_ENABLE_PIN, EXT3_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 4 && defined(EXT4_ENABLE_PIN) && EXT4_ENABLE_PIN > -1
    WRITE(EXT4_ENABLE_PIN, EXT4_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 5 && defined(EXT5_ENABLE_PIN) && EXT5_ENABLE_PIN > -1
    WRITE(EXT5_ENABLE_PIN, EXT5_ENABLE_ON );
#endif
}
#else // Normal extruder
/** \brief Sends the high-signal to the stepper for next extruder step.
Call this function only, if interrupts are disabled.
*/
void Extruder::step()
{
#if NUM_EXTRUDER == 1
    WRITE(EXT0_STEP_PIN, HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT0_JAM_PIN) && EXT0_JAM_PIN > -1
    TEST_EXTRUDER_JAM(0)
#endif
#else
    switch(Extruder::current->id)
    {
    case 0:
#if NUM_EXTRUDER > 0
        WRITE(EXT0_STEP_PIN,HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT0_JAM_PIN) && EXT0_JAM_PIN > -1
        TEST_EXTRUDER_JAM(0)
#endif
#if FEATURE_DITTO_PRINTING
        if(Extruder::dittoMode)
        {
            WRITE(EXT1_STEP_PIN,HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT1_JAM_PIN) && EXT1_JAM_PIN > -1
            TEST_EXTRUDER_JAM(1)
#endif
#if NUM_EXTRUDER > 2
            if(Extruder::dittoMode > 1)
            {
                WRITE(EXT2_STEP_PIN,HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT2_JAM_PIN) && EXT2_JAM_PIN > -1
                TEST_EXTRUDER_JAM(2)
#endif
            }
#endif
#if NUM_EXTRUDER > 3
            if(Extruder::dittoMode > 2)
            {
                WRITE(EXT3_STEP_PIN,HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT3_JAM_PIN) && EXT3_JAM_PIN > -1
                TEST_EXTRUDER_JAM(3)
#endif
            }
#endif
        }
#endif
#endif
        break;
#if defined(EXT1_STEP_PIN) && NUM_EXTRUDER > 1
    case 1:
        WRITE(EXT1_STEP_PIN,HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT1_JAM_PIN) && EXT1_JAM_PIN > -1
        TEST_EXTRUDER_JAM(1)
#endif
        break;
#endif
#if defined(EXT2_STEP_PIN) && NUM_EXTRUDER > 2
    case 2:
        WRITE(EXT2_STEP_PIN,HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT2_JAM_PIN) && EXT2_JAM_PIN > -1
        TEST_EXTRUDER_JAM(2)
#endif
        break;
#endif
#if defined(EXT3_STEP_PIN) && NUM_EXTRUDER > 3
    case 3:
        WRITE(EXT3_STEP_PIN,HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT3_JAM_PIN) && EXT3_JAM_PIN > -1
        TEST_EXTRUDER_JAM(3)
#endif
        break;
#endif
#if defined(EXT4_STEP_PIN) && NUM_EXTRUDER > 4
    case 4:
        WRITE(EXT4_STEP_PIN,HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT4_JAM_PIN) && EXT4_JAM_PIN > -1
        TEST_EXTRUDER_JAM(4)
#endif
        break;
#endif
#if defined(EXT5_STEP_PIN) && NUM_EXTRUDER > 5
    case 5:
        WRITE(EXT5_STEP_PIN,HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT5_JAM_PIN) && EXT5_JAM_PIN > -1
        TEST_EXTRUDER_JAM(5)
#endif
        break;
#endif
    }
#endif
}
/** \brief Sets stepper signal to low for current extruder.

Call this function only, if interrupts are disabled.
*/


void Extruder::unstep()
{
#if NUM_EXTRUDER == 1
    WRITE(EXT0_STEP_PIN,LOW);
#else
    switch(Extruder::current->id)
    {
    case 0:
#if NUM_EXTRUDER > 0
        WRITE(EXT0_STEP_PIN,LOW);
#if FEATURE_DITTO_PRINTING
        if(Extruder::dittoMode)
        {
            WRITE(EXT1_STEP_PIN,LOW);
#if NUM_EXTRUDER > 2
            if(Extruder::dittoMode > 1)
            {
                WRITE(EXT2_STEP_PIN,LOW);
            }
#endif
#if NUM_EXTRUDER > 3
            if(Extruder::dittoMode > 2)
            {
                WRITE(EXT3_STEP_PIN,LOW);
            }
#endif // NUM_EXTRUDER > 3
        }
#endif // FEATURE_DITTO_PRINTING
#endif // NUM_EXTRUDER > 0
        break;
#if defined(EXT1_STEP_PIN) && NUM_EXTRUDER > 1
    case 1:
        WRITE(EXT1_STEP_PIN,LOW);
        break;
#endif
#if defined(EXT2_STEP_PIN) && NUM_EXTRUDER > 2
    case 2:
        WRITE(EXT2_STEP_PIN,LOW);
        break;
#endif
#if defined(EXT3_STEP_PIN) && NUM_EXTRUDER > 3
    case 3:
        WRITE(EXT3_STEP_PIN,LOW);
        break;
#endif
#if defined(EXT4_STEP_PIN) && NUM_EXTRUDER > 4
    case 4:
        WRITE(EXT4_STEP_PIN,LOW);
        break;
#endif
#if defined(EXT5_STEP_PIN) && NUM_EXTRUDER > 5
    case 5:
        WRITE(EXT5_STEP_PIN,LOW);
        break;
#endif
    }
#endif
}
/** \brief Activates the extruder stepper and sets the direction. */
void Extruder::setDirection(uint8_t dir)
{
#if NUM_EXTRUDER == 1
    if(dir)
        WRITE(EXT0_DIR_PIN, !EXT0_INVERSE);
    else
        WRITE(EXT0_DIR_PIN, EXT0_INVERSE);
    RESET_EXTRUDER_JAM(0, dir)
#else
    switch(Extruder::current->id)
    {
#if NUM_EXTRUDER > 0
    case 0:
        if(dir)
            WRITE(EXT0_DIR_PIN,!EXT0_INVERSE);
        else
            WRITE(EXT0_DIR_PIN,EXT0_INVERSE);
        RESET_EXTRUDER_JAM(0, dir)
#if FEATURE_DITTO_PRINTING
        if(Extruder::dittoMode)
        {
            if(dir)
                WRITE(EXT1_DIR_PIN,!EXT1_INVERSE);
            else
                WRITE(EXT1_DIR_PIN,EXT1_INVERSE);
            RESET_EXTRUDER_JAM(1, dir)
#if NUM_EXTRUDER > 2
            if(Extruder::dittoMode > 1)
            {
                if(dir)
                    WRITE(EXT2_DIR_PIN,!EXT2_INVERSE);
                else
                    WRITE(EXT2_DIR_PIN,EXT2_INVERSE);
                RESET_EXTRUDER_JAM(2, dir)
            }
#endif
#if NUM_EXTRUDER > 3
            if(Extruder::dittoMode > 2)
            {
                if(dir)
                    WRITE(EXT3_DIR_PIN,!EXT3_INVERSE);
                else
                    WRITE(EXT3_DIR_PIN,EXT3_INVERSE);
                RESET_EXTRUDER_JAM(3, dir)
            }
#endif
        }
#endif
        break;
#endif
#if defined(EXT1_DIR_PIN) && NUM_EXTRUDER > 1
    case 1:
        if(dir)
            WRITE(EXT1_DIR_PIN,!EXT1_INVERSE);
        else
            WRITE(EXT1_DIR_PIN,EXT1_INVERSE);
        RESET_EXTRUDER_JAM(1, dir)
        break;
#endif
#if defined(EXT2_DIR_PIN) && NUM_EXTRUDER > 2
    case 2:
        if(dir)
            WRITE(EXT2_DIR_PIN,!EXT2_INVERSE);
        else
            WRITE(EXT2_DIR_PIN,EXT2_INVERSE);
        RESET_EXTRUDER_JAM(2, dir)
        break;
#endif
#if defined(EXT3_DIR_PIN) && NUM_EXTRUDER > 3
    case 3:
        if(dir)
            WRITE(EXT3_DIR_PIN,!EXT3_INVERSE);
        else
            WRITE(EXT3_DIR_PIN,EXT3_INVERSE);
        RESET_EXTRUDER_JAM(3, dir)
        break;
#endif
#if defined(EXT4_DIR_PIN) && NUM_EXTRUDER > 4
    case 4:
        if(dir)
            WRITE(EXT4_DIR_PIN,!EXT4_INVERSE);
        else
            WRITE(EXT4_DIR_PIN,EXT4_INVERSE);
        RESET_EXTRUDER_JAM(4, dir)
        break;
#endif
#if defined(EXT5_DIR_PIN) && NUM_EXTRUDER > 5
    case 5:
        if(dir)
            WRITE(EXT5_DIR_PIN,!EXT5_INVERSE);
        else
            WRITE(EXT5_DIR_PIN,EXT5_INVERSE);
        RESET_EXTRUDER_JAM(5, dir)
        break;
#endif
    }
#endif
}

void Extruder::enable()
{
#if NUM_EXTRUDER == 1
#if EXT0_ENABLE_PIN > -1
    WRITE(EXT0_ENABLE_PIN,EXT0_ENABLE_ON );
#endif
#else
    if(Extruder::current->enablePin > -1)
        digitalWrite(Extruder::current->enablePin,Extruder::current->enableOn);
#if FEATURE_DITTO_PRINTING
    if(Extruder::dittoMode)
    {
        if(extruder[1].enablePin > -1)
            digitalWrite(extruder[1].enablePin,extruder[1].enableOn);
#if NUM_EXTRUDER > 2
        if(Extruder::dittoMode > 1 && extruder[2].enablePin > -1)
            digitalWrite(extruder[2].enablePin,extruder[2].enableOn);
#endif
#if NUM_EXTRUDER > 3
        if(Extruder::dittoMode > 2 && extruder[3].enablePin > -1)
            digitalWrite(extruder[3].enablePin,extruder[3].enableOn);
#endif
    }
#endif
#endif
}

#endif  // MIXING_EXTRUDER > 0

void Extruder::disableCurrentExtruderMotor()
{
#if MIXING_EXTRUDER
#if NUM_EXTRUDER > 0 && defined(EXT0_ENABLE_PIN) && EXT0_ENABLE_PIN > -1
    WRITE(EXT0_ENABLE_PIN, !EXT0_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 1 && defined(EXT1_ENABLE_PIN) && EXT1_ENABLE_PIN > -1
    WRITE(EXT1_ENABLE_PIN, !EXT1_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 2 && defined(EXT2_ENABLE_PIN) && EXT2_ENABLE_PIN > -1
    WRITE(EXT2_ENABLE_PIN, !EXT2_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 3 && defined(EXT3_ENABLE_PIN) && EXT3_ENABLE_PIN > -1
    WRITE(EXT3_ENABLE_PIN, !EXT3_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 4 && defined(EXT4_ENABLE_PIN) && EXT4_ENABLE_PIN > -1
    WRITE(EXT4_ENABLE_PIN, !EXT4_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 5 && defined(EXT5_ENABLE_PIN) && EXT5_ENABLE_PIN > -1
    WRITE(EXT5_ENABLE_PIN, !EXT5_ENABLE_ON );
#endif
#else // MIXING_EXTRUDER
    if(Extruder::current->enablePin > -1)
        HAL::digitalWrite(Extruder::current->enablePin,!Extruder::current->enableOn);
#if FEATURE_DITTO_PRINTING
    if(Extruder::dittoMode)
    {
        if(extruder[1].enablePin > -1)
            HAL::digitalWrite(extruder[1].enablePin,!extruder[1].enableOn);
#if NUM_EXTRUDER > 2
        if(Extruder::dittoMode > 1 && extruder[2].enablePin > -1)
            HAL::digitalWrite(extruder[2].enablePin,!extruder[2].enableOn);
#endif
#if NUM_EXTRUDER > 3
        if(Extruder::dittoMode > 2 && extruder[3].enablePin > -1)
            HAL::digitalWrite(extruder[3].enablePin,!extruder[3].enableOn);
#endif
    }
#endif
#endif // MIXING_EXTRUDER
}
void Extruder::disableAllExtruderMotors()
{
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++)
    {
        if(extruder[i].enablePin > -1)
            HAL::digitalWrite(extruder[i].enablePin, !extruder[i].enableOn);
    }
}

void TemperatureController::setTargetTemperature(float target)
{
    targetTemperatureC = target;
    stopDecouple();
    int temp = TEMP_FLOAT_TO_INT(target);
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
                return;
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
                return;
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
                return;
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
}

uint8_t autotuneIndex = 255;
void Extruder::disableAllHeater()
{
    for(uint8_t i=0; i<NUM_TEMPERATURE_LOOPS; i++)
    {
        TemperatureController *c = tempController[i];
        c->targetTemperature = 0;
        c->targetTemperatureC = 0;
        pwm_pos[c->pwmIndex] = 0;
    }
    autotuneIndex = 255;
}

#if TEMP_PID
void TemperatureController::autotunePID(float temp,uint8_t controllerId,int maxCycles,bool storeValues)
{
    float currentTemp;
    int cycles = 0;
    bool heating = true;

    uint32_t temp_millis = HAL::timeInMilliseconds();
    uint32_t t1 = temp_millis;
    uint32_t t2 = temp_millis;
    int32_t t_high = 0;
    int32_t t_low;

    int32_t bias = pidMax >> 1;
    int32_t d = pidMax >> 1;
    float Ku, Tu;
    float Kp = 0, Ki = 0, Kd = 0;
    float maxTemp = 20, minTemp = 20;
    if(maxCycles < 5)
        maxCycles = 5;
    if(maxCycles > 20)
        maxCycles = 20;
    Com::printInfoFLN(Com::tPIDAutotuneStart);

    Extruder::disableAllHeater(); // switch off all heaters.
    autotuneIndex = controllerId;
    pwm_pos[pwmIndex] = pidMax;
    if(controllerId<NUM_EXTRUDER)
    {
        extruder[controllerId].coolerPWM = extruder[controllerId].coolerSpeed;
        extruder[0].coolerPWM = extruder[0].coolerSpeed;
    }
    for(;;)
    {
#if FEATURE_WATCHDOG
        HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

        updateCurrentTemperature();
        currentTemp = currentTemperatureC;
        unsigned long d_time = HAL::timeInMilliseconds();
        maxTemp = RMath::max(maxTemp,currentTemp);
        minTemp = RMath::min(minTemp,currentTemp);
        if(heating == true && currentTemp > temp)   // switch heating -> off
        {
            if(d_time - t2 > (controllerId < NUM_EXTRUDER ? 2500 : 1500))
            {
                heating=false;
                pwm_pos[pwmIndex] = (bias - d);
                t1 = d_time;
                t_high = t1 - t2;
                maxTemp=temp;
            }
        }
        if(heating == false && currentTemp < temp)
        {
            if(d_time - t1 > (controllerId < NUM_EXTRUDER ? 5000 : 3000))
            {
                heating = true;
                t2 = d_time;
                t_low=t2 - t1; // half wave length
                if(cycles > 0)
                {
                    bias += (d*(t_high - t_low))/(t_low + t_high);
                    bias = constrain(bias, 20 ,pidMax - 20);
                    if(bias > pidMax/2) d = pidMax - 1 - bias;
                    else d = bias;

                    Com::printF(Com::tAPIDBias,bias);
                    Com::printF(Com::tAPIDD,d);
                    Com::printF(Com::tAPIDMin,minTemp);
                    Com::printFLN(Com::tAPIDMax,maxTemp);
                    if(cycles > 2)
                    {
                        // Parameter according Ziegler¡§CNichols method: http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
                        Ku = (4.0 * d) / (3.14159*(maxTemp-minTemp));
                        Tu = ((float)(t_low + t_high)/1000.0);
                        Com::printF(Com::tAPIDKu,Ku);
                        Com::printFLN(Com::tAPIDTu,Tu);
                        Kp = 0.6*Ku;
                        Ki = 2*Kp/Tu;
                        Kd = Kp*Tu*0.125;
                        Com::printFLN(Com::tAPIDClassic);
                        Com::printFLN(Com::tAPIDKp,Kp);
                        Com::printFLN(Com::tAPIDKi,Ki);
                        Com::printFLN(Com::tAPIDKd,Kd);
                        /*
                        Kp = 0.33*Ku;
                        Ki = Kp/Tu;
                        Kd = Kp*Tu/3;
                        OUT_P_LN(" Some overshoot");
                        OUT_P_F_LN(" Kp: ",Kp);
                        OUT_P_F_LN(" Ki: ",Ki);
                        OUT_P_F_LN(" Kd: ",Kd);
                        Kp = 0.2*Ku;
                        Ki = 2*Kp/Tu;
                        Kd = Kp*Tu/3;
                        OUT_P_LN(" No overshoot");
                        OUT_P_F_LN(" Kp: ",Kp);
                        OUT_P_F_LN(" Ki: ",Ki);
                        OUT_P_F_LN(" Kd: ",Kd);
                        */
                    }
                }
                pwm_pos[pwmIndex] = (bias + d);
                cycles++;
                minTemp=temp;
            }
        }
        if(currentTemp > (temp + 40))
        {
            Com::printErrorFLN(Com::tAPIDFailedHigh);
            Extruder::disableAllHeater();
            return;
        }
        if(d_time - temp_millis > 1000)
        {
            temp_millis = d_time;
            Commands::printTemperatures();
        }
        if(((d_time - t1) + (d_time - t2)) > (10L*60L*1000L*2L))   // 20 Minutes
        {
            Com::printErrorFLN(Com::tAPIDFailedTimeout);
            Extruder::disableAllHeater();
            return;
        }
        if(cycles > maxCycles)
        {
            Com::printInfoFLN(Com::tAPIDFinished);
            Extruder::disableAllHeater();
            if(storeValues)
            {
                pidPGain = Kp;
                pidIGain = Ki;
                pidDGain = Kd;
                heatManager = HTR_PID;
                EEPROM::storeDataIntoEEPROM();
            }
            return;
        }
        UI_MEDIUM;
        UI_SLOW(true);
    }
}
#endif

/** \brief Writes monitored temperatures.

This function is called every 250ms to write the monitored temperature. If monitoring is
disabled, the function is not called.
*/
void writeMonitor()
{
    Commands::printTemperatures(false);
}

bool reportTempsensorError()
{
    if(!Printer::isAnyTempsensorDefect()) return false;
    for(uint8_t i = 0; i < NUM_TEMPERATURE_LOOPS; i++)
    {
        if(i == NUM_EXTRUDER) Com::printF(Com::tHeatedBed);
        else Com::printF(Com::tExtruderSpace,i);
        int temp = tempController[i]->currentTemperatureC;
        if(temp < MIN_DEFECT_TEMPERATURE || temp > MAX_DEFECT_TEMPERATURE)
            Com::printFLN(Com::tTempSensorDefect);
        else
            Com::printFLN(Com::tTempSensorWorking);
    }
    Com::printErrorFLN(Com::tDryModeUntilRestart);
    return true;
}

#ifdef SUPPORT_MAX6675
int16_t read_max6675(uint8_t ss_pin)
{
    int16_t max6675_temp = 0;
    HAL::spiInit(2);
    HAL::digitalWrite(ss_pin, 0);  // enable TT_MAX6675
    HAL::delayMicroseconds(1);    // ensure 100ns delay - a bit extra is fine
    max6675_temp = HAL::spiReceive(0);
    max6675_temp <<= 8;
    max6675_temp |= HAL::spiReceive(0);
    HAL::digitalWrite(ss_pin, 1);  // disable TT_MAX6675
    return max6675_temp & 4 ? 2000 : max6675_temp >> 3; // thermocouple open?
}
#endif
#ifdef SUPPORT_MAX31855
int16_t read_max31855(uint8_t ss_pin)
{
    uint32_t data = 0;
    int16_t temperature;
    HAL::spiInit(2);
    HAL::digitalWrite(ss_pin, 0);  // enable TT_MAX31855
    HAL::delayMicroseconds(1);    // ensure 100ns delay - a bit extra is fine

    for (unsigned short byte = 0; byte < 4; byte++)
    {
        data <<= 8;
        data |= HAL::spiReceive();
    }

    HAL::digitalWrite(ss_pin, 1);  // disable TT_MAX31855

    //Process temp
    if (data & 0x00010000)
        return 20000; //Some form of error.
    else
    {
        data = data >> 18;
        temperature = data & 0x00001FFF;

        if (data & 0x00002000)
        {
            data = ~data;
            temperature = -1 * ((data & 0x00001FFF) + 1);
        }
    }
    return temperature;
}
#endif

#if FEATURE_RETRACTION
void Extruder::retractDistance(float dist)
{
    float oldFeedrate = Printer::feedrate;
    int32_t distance = static_cast<int32_t>(dist * stepsPerMM / Printer::extrusionFactor);
    int32_t oldEPos = Printer::currentPositionSteps[E_AXIS];
    float speed = distance > 0 ? EEPROM_FLOAT(RETRACTION_SPEED) : EEPROM_FLOAT(RETRACTION_UNDO_SPEED);
    PrintLine::moveRelativeDistanceInSteps(0, 0, 0, -distance, RMath::max(speed, 1.f), false, false);
    Printer::currentPositionSteps[E_AXIS] = oldEPos; // restore previous extruder position
    Printer::feedrate = oldFeedrate;
}

void Extruder::retract(bool isRetract,bool isLong)
{
    float oldFeedrate = Printer::feedrate;
    float distance = (isLong ? EEPROM_FLOAT( RETRACTION_LONG_LENGTH) : EEPROM_FLOAT(RETRACTION_LENGTH));
    int32_t zlift = static_cast<int32_t>(EEPROM_FLOAT(RETRACTION_Z_LIFT) * Printer::axisStepsPerMM[Z_AXIS]);
    int32_t oldZPos = Printer::currentPositionSteps[Z_AXIS];
    float oldZPosF = Printer::currentPosition[Z_AXIS];
    if(isRetract && !isRetracted())
    {
        retractDistance(distance);
        setRetracted(true);
        if(zlift > 0)
            PrintLine::moveRelativeDistanceInStepsReal(0,0,zlift,0,Printer::maxFeedrate[Z_AXIS], false);
    }
    else if(!isRetract && isRetracted())
    {
        distance += (isLong ? EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LONG_LENGTH) : EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LENGTH) );
        if(zlift > 0)
            PrintLine::moveRelativeDistanceInStepsReal(0,0,-zlift,0,Printer::maxFeedrate[Z_AXIS], false);
        retractDistance(-distance);
        setRetracted(false);
    }
    Printer::currentPositionSteps[Z_AXIS] = oldZPos; // z lift should have no visible impact
    Printer::currentPosition[Z_AXIS] = oldZPosF;
    Printer::feedrate = oldFeedrate;
}
#endif

Extruder *Extruder::current;

#if NUM_EXTRUDER>0
const char ext0_select_cmd[] PROGMEM = EXT0_SELECT_COMMANDS;
const char ext0_deselect_cmd[] PROGMEM = EXT0_DESELECT_COMMANDS;
#endif
#if NUM_EXTRUDER>1
const char ext1_select_cmd[] PROGMEM = EXT1_SELECT_COMMANDS;
const char ext1_deselect_cmd[] PROGMEM = EXT1_DESELECT_COMMANDS;
#endif
#if NUM_EXTRUDER>2
const char ext2_select_cmd[] PROGMEM = EXT2_SELECT_COMMANDS;
const char ext2_deselect_cmd[] PROGMEM = EXT2_DESELECT_COMMANDS;
#endif
#if NUM_EXTRUDER>3
const char ext3_select_cmd[] PROGMEM = EXT3_SELECT_COMMANDS;
const char ext3_deselect_cmd[] PROGMEM = EXT3_DESELECT_COMMANDS;
#endif
#if NUM_EXTRUDER>4
const char ext4_select_cmd[] PROGMEM = EXT4_SELECT_COMMANDS;
const char ext4_deselect_cmd[] PROGMEM = EXT4_DESELECT_COMMANDS;
#endif
#if NUM_EXTRUDER>5
const char ext5_select_cmd[] PROGMEM = EXT5_SELECT_COMMANDS;
const char ext5_deselect_cmd[] PROGMEM = EXT5_DESELECT_COMMANDS;
#endif

#if NUM_EXTRUDER == 0
Extruder extruder[1];
#else
Extruder extruder[NUM_EXTRUDER] =
{
#if NUM_EXTRUDER > 0
    {
        0,EXT0_X_OFFSET,EXT0_Y_OFFSET,EXT0_Z_OFFSET,EXT0_STEPS_PER_MM,EXT0_ENABLE_PIN,EXT0_ENABLE_ON,
        EXT0_MAX_FEEDRATE,EXT0_MAX_ACCELERATION,EXT0_MAX_START_FEEDRATE,0,EXT0_WATCHPERIOD
        ,EXT0_WAIT_RETRACT_TEMP,EXT0_WAIT_RETRACT_UNITS
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        ,EXT0_ADVANCE_K
#endif
        ,EXT0_ADVANCE_L,EXT0_ADVANCE_BACKLASH_STEPS
#endif
#if MIXING_EXTRUDER > 0
        ,10,10,{10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10}
#endif
        ,TemperatureController(
            0,EXT0_TEMPSENSOR_TYPE,EXT0_SENSOR_INDEX,0,0,0,0,0,EXT0_HEAT_MANAGER
#if TEMP_PID
            ,0,EXT0_PID_INTEGRAL_DRIVE_MAX,EXT0_PID_INTEGRAL_DRIVE_MIN,EXT0_PID_PGAIN_OR_DEAD_TIME,EXT0_PID_I,EXT0_PID_D,EXT0_PID_MAX,0,0,0/*,{0,0,0,0}*/
#endif
            ,0,0,0,EXT0_DECOUPLE_TEST_PERIOD
        )
        ,ext0_select_cmd,ext0_deselect_cmd,EXT0_EXTRUDER_COOLER_SPEED,0,0,0
#if EXTRUDER_JAM_CONTROL
        ,0,0,10,0,0
#endif
    }
#endif
#if NUM_EXTRUDER > 1
    ,{
        1,EXT1_X_OFFSET,EXT1_Y_OFFSET,EXT1_Z_OFFSET,EXT1_STEPS_PER_MM,EXT1_ENABLE_PIN,EXT1_ENABLE_ON,
        EXT1_MAX_FEEDRATE,EXT1_MAX_ACCELERATION,EXT1_MAX_START_FEEDRATE,0,EXT1_WATCHPERIOD
        ,EXT1_WAIT_RETRACT_TEMP,EXT1_WAIT_RETRACT_UNITS
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        ,EXT1_ADVANCE_K
#endif
        ,EXT1_ADVANCE_L,EXT1_ADVANCE_BACKLASH_STEPS
#endif
#if MIXING_EXTRUDER > 0
        ,10,10,{10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10}
#endif
        ,TemperatureController(
            1,EXT1_TEMPSENSOR_TYPE,EXT1_SENSOR_INDEX,0,0,0,0,0,EXT1_HEAT_MANAGER
#if TEMP_PID
            ,0,EXT1_PID_INTEGRAL_DRIVE_MAX,EXT1_PID_INTEGRAL_DRIVE_MIN,EXT1_PID_PGAIN_OR_DEAD_TIME,EXT1_PID_I,EXT1_PID_D,EXT1_PID_MAX,0,0,0/*,{0,0,0,0}*/
#endif
            ,0,0,0,EXT1_DECOUPLE_TEST_PERIOD
        )
        ,ext1_select_cmd,ext1_deselect_cmd,EXT1_EXTRUDER_COOLER_SPEED,0,0,0
#if EXTRUDER_JAM_CONTROL
        ,0,0,10,0,0
#endif
    }
#endif
#if NUM_EXTRUDER > 2
    ,{
        2,EXT2_X_OFFSET,EXT2_Y_OFFSET,EXT2_Z_OFFSET,EXT2_STEPS_PER_MM,EXT2_ENABLE_PIN,EXT2_ENABLE_ON,
        EXT2_MAX_FEEDRATE,EXT2_MAX_ACCELERATION,EXT2_MAX_START_FEEDRATE,0,EXT2_WATCHPERIOD
        ,EXT2_WAIT_RETRACT_TEMP,EXT2_WAIT_RETRACT_UNITS
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        ,EXT2_ADVANCE_K
#endif
        ,EXT2_ADVANCE_L,EXT2_ADVANCE_BACKLASH_STEPS
#endif
#if MIXING_EXTRUDER > 0
        ,10,10,{10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10}
#endif
        ,TemperatureController(
            2,EXT2_TEMPSENSOR_TYPE,EXT2_SENSOR_INDEX,0,0,0,0,0,EXT2_HEAT_MANAGER
#if TEMP_PID
            ,0,EXT2_PID_INTEGRAL_DRIVE_MAX,EXT2_PID_INTEGRAL_DRIVE_MIN,EXT2_PID_PGAIN_OR_DEAD_TIME,EXT2_PID_I,EXT2_PID_D,EXT2_PID_MAX,0,0,0/*,{0,0,0,0}*/
#endif
            ,0,0,0,EXT2_DECOUPLE_TEST_PERIOD
        )
        ,ext2_select_cmd,ext2_deselect_cmd,EXT2_EXTRUDER_COOLER_SPEED,0,0,0
#if EXTRUDER_JAM_CONTROL
        ,0,0,10,0,0
#endif
    }
#endif
#if NUM_EXTRUDER > 3
    ,{
        3,EXT3_X_OFFSET,EXT3_Y_OFFSET,EXT3_Z_OFFSET,EXT3_STEPS_PER_MM,EXT3_ENABLE_PIN,EXT3_ENABLE_ON,
        EXT3_MAX_FEEDRATE,EXT3_MAX_ACCELERATION,EXT3_MAX_START_FEEDRATE,0,EXT3_WATCHPERIOD
        ,EXT3_WAIT_RETRACT_TEMP,EXT3_WAIT_RETRACT_UNITS
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        ,EXT3_ADVANCE_K
#endif
        ,EXT3_ADVANCE_L,EXT3_ADVANCE_BACKLASH_STEPS
#endif
#if MIXING_EXTRUDER > 0
        ,10,10,{10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10}
#endif
        ,TemperatureController(
            3,EXT3_TEMPSENSOR_TYPE,EXT3_SENSOR_INDEX,0,0,0,0,0,EXT3_HEAT_MANAGER
#if TEMP_PID
            ,0,EXT3_PID_INTEGRAL_DRIVE_MAX,EXT3_PID_INTEGRAL_DRIVE_MIN,EXT3_PID_PGAIN_OR_DEAD_TIME,EXT3_PID_I,EXT3_PID_D,EXT3_PID_MAX,0,0,0/*,{0,0,0,0}*/
#endif
            ,0,0,0,EXT3_DECOUPLE_TEST_PERIOD
        )
        ,ext3_select_cmd,ext3_deselect_cmd,EXT3_EXTRUDER_COOLER_SPEED,0,0,0
#if EXTRUDER_JAM_CONTROL
        ,0,0,10,0,0
#endif
    }
#endif
#if NUM_EXTRUDER > 4
    ,{
        4,EXT4_X_OFFSET,EXT4_Y_OFFSET,EXT4_Z_OFFSET,EXT4_STEPS_PER_MM,EXT4_ENABLE_PIN,EXT4_ENABLE_ON,
        EXT4_MAX_FEEDRATE,EXT4_MAX_ACCELERATION,EXT4_MAX_START_FEEDRATE,0,EXT4_WATCHPERIOD
        ,EXT4_WAIT_RETRACT_TEMP,EXT4_WAIT_RETRACT_UNITS
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        ,EXT4_ADVANCE_K
#endif
        ,EXT4_ADVANCE_L,EXT4_ADVANCE_BACKLASH_STEPS
#endif
#if MIXING_EXTRUDER > 0
        ,10,10,{10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10}
#endif
        ,TemperatureController(
            4,EXT4_TEMPSENSOR_TYPE,EXT4_SENSOR_INDEX,0,0,0,0,0,EXT4_HEAT_MANAGER
#if TEMP_PID
            ,0,EXT4_PID_INTEGRAL_DRIVE_MAX,EXT4_PID_INTEGRAL_DRIVE_MIN,EXT4_PID_PGAIN_OR_DEAD_TIME,EXT4_PID_I,EXT4_PID_D,EXT4_PID_MAX,0,0,0/*,{0,0,0,0}*/
#endif
            ,0,0,0,EXT4_DECOUPLE_TEST_PERIOD
        )
        ,ext4_select_cmd,ext4_deselect_cmd,EXT4_EXTRUDER_COOLER_SPEED,0,0,0
#if EXTRUDER_JAM_CONTROL
        ,0,0,10,0,0
#endif
    }
#endif
#if NUM_EXTRUDER > 5
    ,{
        5,EXT5_X_OFFSET,EXT5_Y_OFFSET,EXT5_Z_OFFSET,EXT5_STEPS_PER_MM,EXT5_ENABLE_PIN,EXT5_ENABLE_ON,
        EXT5_MAX_FEEDRATE,EXT5_MAX_ACCELERATION,EXT5_MAX_START_FEEDRATE,0,EXT5_WATCHPERIOD
        ,EXT5_WAIT_RETRACT_TEMP,EXT5_WAIT_RETRACT_UNITS
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        ,EXT5_ADVANCE_K
#endif
        ,EXT5_ADVANCE_L,EXT5_ADVANCE_BACKLASH_STEPS
#endif
#if MIXING_EXTRUDER > 0
        ,10,10,{10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10}
#endif
        ,TemperatureController(
            5,EXT5_TEMPSENSOR_TYPE,EXT5_SENSOR_INDEX,0,0,0,0,0,EXT5_HEAT_MANAGER
#if TEMP_PID
            ,0,EXT5_PID_INTEGRAL_DRIVE_MAX,EXT5_PID_INTEGRAL_DRIVE_MIN,EXT5_PID_PGAIN_OR_DEAD_TIME,EXT5_PID_I,EXT5_PID_D,EXT5_PID_MAX,0,0,0/*,{0,0,0,0}*/
#endif
            ,0,0,0,EXT5_DECOUPLE_TEST_PERIOD
        )
        ,ext5_select_cmd,ext5_deselect_cmd,EXT5_EXTRUDER_COOLER_SPEED,0,0,0
#if EXTRUDER_JAM_CONTROL
        ,0,0,10,0,0
#endif
    }
#endif
};
#endif // NUM_EXTRUDER

// @todo  BED_0_SENSOR_INDEX  BED_SENSOR_INDEX
#if HAVE_HEATED_BED
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER+HEATED_BED_NUM
TemperatureController heatedBedController[HEATED_BED_NUM] =
{
#if HEATED_BED_NUM >= 1
     TemperatureController(NUM_EXTRUDER,HEATED_BED_SENSOR_TYPE,BED_0_SENSOR_INDEX,0,0,0,0,0,HEATED_BED_HEAT_MANAGER
#if TEMP_PID
        ,0,HEATED_BED_PID_INTEGRAL_DRIVE_MAX,HEATED_BED_PID_INTEGRAL_DRIVE_MIN,HEATED_BED_PID_PGAIN_OR_DEAD_TIME,HEATED_BED_PID_IGAIN,HEATED_BED_PID_DGAIN,HEATED_BED_PID_MAX,0,0,0/*,{0,0,0,0}*/
#endif
        ,0,0,0,HEATED_BED_DECOUPLE_TEST_PERIOD
    )
#endif // HEATED_BED_NUM
#if HEATED_BED_NUM >= 2
     , TemperatureController(NUM_EXTRUDER+1,HEATED_BED_SENSOR_TYPE,BED_1_SENSOR_INDEX,0,0,0,0,0,HEATED_BED_HEAT_MANAGER
#if TEMP_PID
        ,0,HEATED_BED_PID_INTEGRAL_DRIVE_MAX,HEATED_BED_PID_INTEGRAL_DRIVE_MIN,HEATED_BED_PID_PGAIN_OR_DEAD_TIME,HEATED_BED_PID_IGAIN,HEATED_BED_PID_DGAIN,HEATED_BED_PID_MAX,0,0,0/*,{0,0,0,0}*/
#endif
        ,0,0,0,HEATED_BED_DECOUPLE_TEST_PERIOD
    )
#endif // HEATED_BED_NUM
#if HEATED_BED_NUM >= 3
     , TemperatureController(NUM_EXTRUDER+2,HEATED_BED_SENSOR_TYPE,BED_2_SENSOR_INDEX,0,0,0,0,0,HEATED_BED_HEAT_MANAGER
#if TEMP_PID
        ,0,HEATED_BED_PID_INTEGRAL_DRIVE_MAX,HEATED_BED_PID_INTEGRAL_DRIVE_MIN,HEATED_BED_PID_PGAIN_OR_DEAD_TIME,HEATED_BED_PID_IGAIN,HEATED_BED_PID_DGAIN,HEATED_BED_PID_MAX,0,0,0/*,{0,0,0,0}*/
#endif
        ,0,0,0,HEATED_BED_DECOUPLE_TEST_PERIOD
    )
#endif // HEATED_BED_NUM
#if HEATED_BED_NUM >= 4
     , TemperatureController(NUM_EXTRUDER+3,HEATED_BED_SENSOR_TYPE,BED_3_SENSOR_INDEX,0,0,0,0,0,HEATED_BED_HEAT_MANAGER
#if TEMP_PID
        ,0,HEATED_BED_PID_INTEGRAL_DRIVE_MAX,HEATED_BED_PID_INTEGRAL_DRIVE_MIN,HEATED_BED_PID_PGAIN_OR_DEAD_TIME,HEATED_BED_PID_IGAIN,HEATED_BED_PID_DGAIN,HEATED_BED_PID_MAX,0,0,0/*,{0,0,0,0}*/
#endif
        ,0,0,0,HEATED_BED_DECOUPLE_TEST_PERIOD
    )
#endif // HEATED_BED_NUM

}; //end tab beds
#else
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER
#endif

TemperatureController *tempController[NUM_TEMPERATURE_LOOPS] =
{
#if NUM_EXTRUDER>0
    &extruder[0].tempControl
#endif
#if NUM_EXTRUDER>1
    ,&extruder[1].tempControl
#endif
#if NUM_EXTRUDER>2
    ,&extruder[2].tempControl
#endif
#if NUM_EXTRUDER>3
    ,&extruder[3].tempControl
#endif
#if NUM_EXTRUDER>4
    ,&extruder[4].tempControl
#endif
#if NUM_EXTRUDER>5
    ,&extruder[5].tempControl
#endif
#if HAVE_HEATED_BED
#if NUM_EXTRUDER==0
 #if HEATED_BED_NUM >= 1
    &heatedBedController[0]
 #endif
#else
 #if HEATED_BED_NUM >= 1
    ,&heatedBedController[0]
 #endif
#endif
#if HEATED_BED_NUM >= 2
    ,&heatedBedController[1]
#endif
#if HEATED_BED_NUM >= 3
    ,&heatedBedController[2]
#endif
#if HEATED_BED_NUM >= 3
    ,&heatedBedController[2]
#endif
#endif
};

