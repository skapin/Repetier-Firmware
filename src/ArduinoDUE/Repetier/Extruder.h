#ifndef EXTRUDER_H_INCLUDED
#define EXTRUDER_H_INCLUDED

#define CELSIUS_EXTRA_BITS 3
#define VIRTUAL_EXTRUDER 16 // don't change this to more then 16 without modifying the eeprom positions


#define NUMTEMPS_1 28
// Epcos B57560G0107F000
const short temptable_1[NUMTEMPS_1][2] PROGMEM =
{
    {0,4000},{92,2400},{105,2320},{121,2240},{140,2160},{162,2080},{189,2000},{222,1920},{261,1840},{308,1760},
    {365,1680},{434,1600},{519,1520},{621,1440},{744,1360},{891,1280},{1067,1200},{1272,1120},
    {1771,960},{2357,800},{2943,640},{3429,480},{3760,320},{3869,240},{3912,200},{3948,160},{4077,-160},{4094,-440}
};
#define NUMTEMPS_2 21
const short temptable_2[NUMTEMPS_2][2] PROGMEM =
{
    {1*4, 848*8},{54*4, 275*8}, {107*4, 228*8}, {160*4, 202*8},{213*4, 185*8}, {266*4, 171*8}, {319*4, 160*8}, {372*4, 150*8},
    {425*4, 141*8}, {478*4, 133*8},{531*4, 125*8},{584*4, 118*8},{637*4, 110*8},{690*4, 103*8},{743*4, 95*8},{796*4, 86*8},
    {849*4, 77*8},{902*4, 65*8},{955*4, 49*8},{1008*4, 17*8},{1020*4, 0*8} //safety
};

#define NUMTEMPS_3 28
const short temptable_3[NUMTEMPS_3][2] PROGMEM =
{
    {1*4,864*8},{21*4,300*8},{25*4,290*8},{29*4,280*8},{33*4,270*8},{39*4,260*8},{46*4,250*8},{54*4,240*8},{64*4,230*8},{75*4,220*8},
    {90*4,210*8},{107*4,200*8},{128*4,190*8},{154*4,180*8},{184*4,170*8},{221*4,160*8},{265*4,150*8},{316*4,140*8},{375*4,130*8},
    {441*4,120*8},{513*4,110*8},{588*4,100*8},{734*4,80*8},{856*4,60*8},{938*4,40*8},{986*4,20*8},{1008*4,0*8},{1018*4,-20*8}
};

#define NUMTEMPS_4 20
const short temptable_4[NUMTEMPS_4][2] PROGMEM =
{
    {1*4, 430*8},{54*4, 137*8},{107*4, 107*8},{160*4, 91*8},{213*4, 80*8},{266*4, 71*8},{319*4, 64*8},{372*4, 57*8},{425*4, 51*8},
    {478*4, 46*8},{531*4, 41*8},{584*4, 35*8},{637*4, 30*8},{690*4, 25*8},{743*4, 20*8},{796*4, 14*8},{849*4, 7*8},{902*4, 0*8},
    {955*4, -11*8},{1008*4, -35*8}
};

#define NUMTEMPS_8 34
const short temptable_8[NUMTEMPS_8][2] PROGMEM =
{
    {0,8000},{69,2400},{79,2320},{92,2240},{107,2160},{125,2080},{146,2000},{172,1920},{204,1840},{222,1760},{291,1680},{350,1600},
    {422,1520},{511,1440},{621,1360},{755,1280},{918,1200},{1114,1120},{1344,1040},{1608,960},{1902,880},{2216,800},{2539,720},
    {2851,640},{3137,560},{3385,480},{3588,400},{3746,320},{3863,240},{3945,160},{4002,80},{4038,0},{4061,-80},{4075,-160}
};
#define NUMTEMPS_9 67 // 100k Honeywell 135-104LAG-J01
const short temptable_9[NUMTEMPS_9][2] PROGMEM =
{
    {1*4, 941*8},{19*4, 362*8},{37*4, 299*8}, //top rating 300C
    {55*4, 266*8},{73*4, 245*8},{91*4, 229*8},{109*4, 216*8},{127*4, 206*8},{145*4, 197*8},{163*4, 190*8},{181*4, 183*8},{199*4, 177*8},
    {217*4, 171*8},{235*4, 166*8},{253*4, 162*8},{271*4, 157*8},{289*4, 153*8},{307*4, 149*8},{325*4, 146*8},{343*4, 142*8},{361*4, 139*8},
    {379*4, 135*8},{397*4, 132*8},{415*4, 129*8},{433*4, 126*8},{451*4, 123*8},{469*4, 121*8},{487*4, 118*8},{505*4, 115*8},{523*4, 112*8},
    {541*4, 110*8},{559*4, 107*8},{577*4, 105*8},{595*4, 102*8},{613*4, 99*8},{631*4, 97*8},{649*4, 94*8},{667*4, 92*8},{685*4, 89*8},
    {703*4, 86*8},{721*4, 84*8},{739*4, 81*8},{757*4, 78*8},{775*4, 75*8},{793*4, 72*8},{811*4, 69*8},{829*4, 66*8},{847*4, 62*8},
    {865*4, 59*8},{883*4, 55*8},{901*4, 51*8},{919*4, 46*8},{937*4, 41*8},
    {955*4, 35*8},{973*4, 27*8},{991*4, 17*8},{1009*4, 1*8},{1023*4, 0}  //to allow internal 0 degrees C
};
#define NUMTEMPS_10 20 // 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
const short temptable_10[NUMTEMPS_10][2] PROGMEM =
{
    {1*4, 704*8},{54*4, 216*8},{107*4, 175*8},{160*4, 152*8},{213*4, 137*8},{266*4, 125*8},{319*4, 115*8},{372*4, 106*8},{425*4, 99*8},
    {478*4, 91*8},{531*4, 85*8},{584*4, 78*8},{637*4, 71*8},{690*4, 65*8},{743*4, 58*8},{796*4, 50*8},{849*4, 42*8},{902*4, 31*8},
    {955*4, 17*8},{1008*4, 0}
};
#define NUMTEMPS_11 31 // 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
const short temptable_11[NUMTEMPS_11][2] PROGMEM =
{
    {1*4, 936*8},{36*4, 300*8},{71*4, 246*8},{106*4, 218*8},{141*4, 199*8},{176*4, 185*8},{211*4, 173*8},{246*4, 163*8},{281*4, 155*8},
    {316*4, 147*8},{351*4, 140*8},{386*4, 134*8},{421*4, 128*8},{456*4, 122*8},{491*4, 117*8},{526*4, 112*8},{561*4, 107*8},{596*4, 102*8},
    {631*4, 97*8},{666*4, 92*8},{701*4, 87*8},{736*4, 81*8},{771*4, 76*8},{806*4, 70*8},{841*4, 63*8},{876*4, 56*8},{911*4, 48*8},
    {946*4, 38*8},{981*4, 23*8},{1005*4, 5*8},{1016*4, 0}
};
#define NUMTEMPS_12 31 // 100k RS thermistor 198-961 (4.7k pullup)
const short temptable_12[NUMTEMPS_12][2] PROGMEM =
{
    {1*4, 929*8},{36*4, 299*8},{71*4, 246*8},{106*4, 217*8},{141*4, 198*8},{176*4, 184*8},{211*4, 173*8},{246*4, 163*8},{281*4, 154*8},{316*4, 147*8},
    {351*4, 140*8},{386*4, 134*8},{421*4, 128*8},{456*4, 122*8},{491*4, 117*8},{526*4, 112*8},{561*4, 107*8},{596*4, 102*8},{631*4, 97*8},{666*4, 91*8},
    {701*4, 86*8},{736*4, 81*8},{771*4, 76*8},{806*4, 70*8},{841*4, 63*8},{876*4, 56*8},{911*4, 48*8},{946*4, 38*8},{981*4, 23*8},{1005*4, 5*8},{1016*4, 0*8}
};
#if NUM_TEMPS_USERTHERMISTOR0 > 0
const short temptable_5[NUM_TEMPS_USERTHERMISTOR0][2] PROGMEM = USER_THERMISTORTABLE0 ;
#endif
#if NUM_TEMPS_USERTHERMISTOR1 > 0
const short temptable_6[NUM_TEMPS_USERTHERMISTOR1][2] PROGMEM = USER_THERMISTORTABLE1 ;
#endif
#if NUM_TEMPS_USERTHERMISTOR2 > 0
const short temptable_7[NUM_TEMPS_USERTHERMISTOR2][2] PROGMEM = USER_THERMISTORTABLE2 ;
#endif
const short * const temptables[12] PROGMEM = {(short int *)&temptable_1[0][0],(short int *)&temptable_2[0][0],(short int *)&temptable_3[0][0],(short int *)&temptable_4[0][0]
#if NUM_TEMPS_USERTHERMISTOR0 > 0
        ,(short int *)&temptable_5[0][0]
#else
        ,0
#endif
#if NUM_TEMPS_USERTHERMISTOR1 > 0
        ,(short int *)&temptable_6[0][0]
#else
        ,0
#endif
#if NUM_TEMPS_USERTHERMISTOR2 > 0
        ,(short int *)&temptable_7[0][0]
#else
        ,0
#endif
        ,(short int *)&temptable_8[0][0]
        ,(short int *)&temptable_9[0][0]
        ,(short int *)&temptable_10[0][0]
        ,(short int *)&temptable_11[0][0]
        ,(short int *)&temptable_12[0][0]
                                             };
const uint8_t temptables_num[12] PROGMEM = {NUMTEMPS_1,NUMTEMPS_2,NUMTEMPS_3,NUMTEMPS_4,NUM_TEMPS_USERTHERMISTOR0,NUM_TEMPS_USERTHERMISTOR1,NUM_TEMPS_USERTHERMISTOR2,NUMTEMPS_8,
                                 NUMTEMPS_9,NUMTEMPS_10,NUMTEMPS_11,NUMTEMPS_12
                                           };



//#if TEMP_PID
//extern uint8_t current_extruder_out;
//#endif

// Updates the temperature of all extruders and heated bed if it's time.
// Toggels the heater power if necessary.
extern bool reportTempsensorError(); ///< Report defect sensors
extern uint8_t manageMonitor;

#define TEMPERATURE_CONTROLLER_FLAG_ALARM 1
#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL 2  //< Full heating enabled
#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD 4  //< Holding target temperature
#define TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT    8  //< Indicating sensor defect
#define TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED 16 //< Indicating sensor decoupling
#define TEMPERATURE_CONTROLLER_FLAG_JAM           32 //< Indicates a jammed filament
#define TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN      64 //< Indicates a slowed down extruder

/** TemperatureController manages one heater-temperature sensore loop. You can have up to
4 loops allowing pid/bang bang for up to 3 extruder and the heated bed.

*/
class TemperatureController : public Sensor, public Heater
{
public:
    uint8_t pwmIndex; ///< pwm index for output control. 0-2 = Extruder, 3 = Fan, 4 = Heated Bed

    uint8_t flags;
    millis_t lastDecoupleTest;  ///< Last time of decoupling sensor-heater test
    float  lastDecoupleTemp;  ///< Temperature on last test
    millis_t decoupleTestPeriod; ///< Time between setting and testing decoupling.

    void setTargetTemperature(float target);
    void updateTempControlVars();

    TemperatureController( uint8_t pwmIndex,  uint8_t sensorType, uint8_t sensorPin, int16_t currentTemperature, int16_t targetTemperature,
    float currentTemperatureC, float targetTemperatureC, uint32_t lastTemperatureUpdate, int8_t heatManager
#ifdef TEMP_PID
    , float tempIState, uint8_t pidDriveMax, uint8_t pidDriveMin, float pidPGain, float pidIGain, float pidDGain, uint8_t pidMax,
     float tempIStateLimitMax, float tempIStateLimitMin, uint8_t tempPointer,
#endif
    uint8_t flags, millis_t lastDecoupleTest, float  lastDecoupleTemp, millis_t decoupleTestPeriod
 ):Sensor(sensorType, sensorPin, currentTemperature, currentTemperatureC),
    Heater(targetTemperatureC, targetTemperature, lastTemperatureUpdate, heatManager,
            #ifdef TEMP_PID
            tempIState,pidDriveMax,pidDriveMin, pidPGain,pidIGain,pidDGain,pidMax,tempIStateLimitMax,tempIStateLimitMin,tempPointer
            #endif
            )
    {
        this->pwmIndex = pwmIndex;
        this->flags = flags;
        this->lastDecoupleTest = lastDecoupleTest;
        this->lastDecoupleTemp = lastDecoupleTemp;
        this->decoupleTestPeriod = decoupleTestPeriod;
    }

    uint8_t computeHeaterOutputDecoupled( uint8_t current_output, millis_t& t )
    {
        uint8_t state = STOP_DECOUPLE;
        uint8_t output = this->computeHeaterOutput( currentTemperatureC, state, t );
        if ( state == STOP_DECOUPLE )
        {
            this->stopDecouple();
        }
        else if ( state == HOLD_DECOUPLE )
        {
            this->startHoldDecouple(t);
        }
        else if ( state == FULL_DECOUPLE )
        {
            this->startFullDecouple(t);
        }
        else if ( state = FOR_CONTINUE )
        {
            output = current_output;
        }
        return output;
    }
    inline bool isAlarm()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_ALARM;
    }
    inline void setAlarm(bool on)
    {
        if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_ALARM;
        else flags &= ~TEMPERATURE_CONTROLLER_FLAG_ALARM;
    }
    inline bool isDecoupleFull()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL;
    }
    inline bool isDecoupleFullOrHold()
    {
        return flags & (TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL | TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD);
    }
    inline void setDecoupleFull(bool on)
    {
        flags &= ~(TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL | TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD);
        if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL;
    }
    inline bool isDecoupleHold()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD;
    }
    inline void setDecoupleHold(bool on)
    {
        flags &= ~(TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL | TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD);
        if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD;
    }
    inline void startFullDecouple(millis_t &t)
    {
        if(isDecoupleFull()) return;
        lastDecoupleTest = t;
        lastDecoupleTemp = currentTemperatureC;
        setDecoupleFull(true);
    }
    inline void startHoldDecouple(millis_t &t)
    {
        if(isDecoupleHold()) return;
        if(fabs(currentTemperatureC - targetTemperatureC) + 1 > DECOUPLING_TEST_MAX_HOLD_VARIANCE) return;
        lastDecoupleTest = t;
        lastDecoupleTemp = targetTemperatureC;
        setDecoupleHold(true);
    }
    inline void stopDecouple()
    {
        setDecoupleFull(false);
    }
    inline bool isSensorDefect()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
    }
    inline bool isSensorDecoupled()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED;
    }
#if EXTRUDER_JAM_CONTROL
    inline bool isJammed()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_JAM;
    }
    void setJammed(bool on);
    inline bool isSlowedDown()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN;
    }
    inline void setSlowedDown(bool on)
    {
        flags &= ~TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN;
        if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN;
    }

#endif
    void waitForTargetTemperature();
#if TEMP_PID
    void autotunePID(float temp,uint8_t controllerId,int maxCycles,bool storeResult);
#endif
};
class Extruder;
extern Extruder extruder[];

#if EXTRUDER_JAM_CONTROL
#define _TEST_EXTRUDER_JAM(x,pin) {\
        uint8_t sig = READ(pin);extruder[x].jamStepsSinceLastSignal += extruder[x].jamLastDir;\
        if(extruder[x].jamLastSignal != sig && abs(extruder[x].jamStepsSinceLastSignal - extruder[x].jamLastChangeAt) > JAM_MIN_STEPS) {\
          if(sig) {extruder[x].resetJamSteps();} \
          extruder[x].jamLastSignal = sig;extruder[x].jamLastChangeAt = extruder[x].jamStepsSinceLastSignal;\
        } else if(abs(extruder[x].jamStepsSinceLastSignal) > JAM_ERROR_STEPS && !Printer::isDebugJamOrDisabled() && !extruder[x].tempControl.isJammed()) \
            extruder[x].tempControl.setJammed(true);\
    }
#define ___TEST_EXTRUDER_JAM(x,y) _TEST_EXTRUDER_JAM(x,y)
#define __TEST_EXTRUDER_JAM(x) ___TEST_EXTRUDER_JAM(x,EXT ## x ## _JAM_PIN)
#define TEST_EXTRUDER_JAM(x) __TEST_EXTRUDER_JAM(x)
#define RESET_EXTRUDER_JAM(x,dir) extruder[x].jamLastDir = dir ? 1 : -1;
#else
#define TEST_EXTRUDER_JAM(x)
#define RESET_EXTRUDER_JAM(x,dir)
#endif

#define EXTRUDER_FLAG_RETRACTED 1
#define EXTRUDER_FLAG_WAIT_JAM_STARTCOUNT 2 ///< Waiting for the first signal to start counting

/** \brief Data to drive one extruder.

This structure contains all definitions for an extruder and all
current state variables, like current temperature, feeder position etc.
*/
class Extruder   // Size: 12*1 Byte+12*4 Byte+4*2Byte = 68 Byte
{
public:
    static Extruder *current;
#if FEATURE_DITTO_PRINTING
    static uint8_t dittoMode;
#endif
#if MIXING_EXTRUDER > 0
    static int mixingS; ///< Sum of all weights
    static uint8_t mixingDir; ///< Direction flag
    static uint8_t activeMixingExtruder;
#endif
    uint8_t id;
    int32_t xOffset;
    int32_t yOffset;
    int32_t zOffset;
    float stepsPerMM;        ///< Steps per mm.
    int8_t enablePin;          ///< Pin to enable extruder stepper motor.
//  uint8_t directionPin; ///< Pin number to assign the direction.
//  uint8_t stepPin; ///< Pin number for a step.
    uint8_t enableOn;
//  uint8_t invertDir; ///< 1 if the direction of the extruder should be inverted.
    float maxFeedrate;      ///< Maximum feedrate in mm/s.
    float maxAcceleration;  ///< Maximum acceleration in mm/s^2.
    float maxStartFeedrate; ///< Maximum start feedrate in mm/s.
    int32_t extrudePosition;   ///< Current extruder position in steps.
    int16_t watchPeriod;        ///< Time in seconds, a M109 command will wait to stabalize temperature
    int16_t waitRetractTemperature; ///< Temperature to retract the filament when waiting for heatup
    int16_t waitRetractUnits;   ///< Units to retract the filament when waiting for heatup
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
    float advanceK;         ///< Koefficient for advance algorithm. 0 = off
#endif
    float advanceL;
    int16_t advanceBacklash;
#endif // USE_ADVANCE
#if MIXING_EXTRUDER > 0
    int mixingW;   ///< Weight for this extruder when mixing steps
    int mixingE;   ///< Cumulated error for this step.
    int virtualWeights[VIRTUAL_EXTRUDER]; // Virtual extruder weights
#endif // MIXING_EXTRUDER > 0
    TemperatureController tempControl;
    const char * PROGMEM selectCommands;
    const char * PROGMEM deselectCommands;
    uint8_t coolerSpeed; ///< Speed to use when enabled
    uint8_t coolerPWM; ///< current PWM setting
    float diameter;
    uint8_t flags;
#if EXTRUDER_JAM_CONTROL
    int16_t jamStepsSinceLastSignal; // when was the last signal
    uint8_t jamLastSignal; // what was the last signal
    int8_t jamLastDir;
    int16_t jamStepsOnSignal;
    int16_t jamLastChangeAt;
#endif

    // Methods here

#if EXTRUDER_JAM_CONTROL
    inline bool isWaitJamStartcount()
    {
        return flags & EXTRUDER_FLAG_WAIT_JAM_STARTCOUNT;
    }
    inline void setWaitJamStartcount(bool on)
    {
        if(on) flags |= EXTRUDER_FLAG_WAIT_JAM_STARTCOUNT;
        else flags &= ~(EXTRUDER_FLAG_WAIT_JAM_STARTCOUNT);
    }
    static void markAllUnjammed();
    void resetJamSteps();
#endif
#if MIXING_EXTRUDER > 0
    static void setMixingWeight(uint8_t extr,int weight);
#endif
    static void step();
    static void unstep();
    static void setDirection(uint8_t dir);
    static void enable();
#if FEATURE_RETRACTION
    inline bool isRetracted() {return (flags & EXTRUDER_FLAG_RETRACTED) != 0;}
    inline void setRetracted(bool on) {
        flags = (flags & (255 - EXTRUDER_FLAG_RETRACTED)) | (on ? EXTRUDER_FLAG_RETRACTED : 0);
    }
    void retract(bool isRetract,bool isLong);
    void retractDistance(float dist);
#endif
    static void manageTemperatures();
    static void disableCurrentExtruderMotor();
    static void disableAllExtruderMotors();
    static void selectExtruderById(uint8_t extruderId);
    static void disableAllHeater();
    static void initExtruder();
    static void initHeatedBed();
    static void setHeatedBedTemperature(float temp_celsius,bool beep = false);
    static float getHeatedBedTemperature();
    static void setTemperatureForExtruder(float temp_celsius,uint8_t extr,bool beep = false,bool wait = false);
    static void pauseExtruders();
    static void unpauseExtruders();
};

#if HAVE_HEATED_BED
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER+1
extern TemperatureController heatedBedController;
#else
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER
#endif
#define TEMP_INT_TO_FLOAT(temp) ((float)(temp)/(float)(1<<CELSIUS_EXTRA_BITS))
#define TEMP_FLOAT_TO_INT(temp) ((int)((temp)*(1<<CELSIUS_EXTRA_BITS)))

//extern Extruder *Extruder::current;
extern TemperatureController *tempController[NUM_TEMPERATURE_LOOPS];
extern uint8_t autotuneIndex;


#endif // EXTRUDER_H_INCLUDED
