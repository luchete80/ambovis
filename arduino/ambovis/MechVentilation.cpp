/** Mechanical ventilation.
 *
 * @file MechVentilation.cpp
 *
 * This is the mechanical ventilation software module.
 * It handles the mechanical ventilation control loop.
 */

#include "MechVentilation.h"

int currentWaitTriggerTime = 0;
int currentStopInsufflationTime = 0;
float currentFlow = 0;

MechVentilation::MechVentilation(
    FlexyStepper *stepper,
    Sensors *sensors,
    AutoPID *pid,
    VentilationOptions_t options)
{

    _init(
        stepper,
        sensors,
        pid,
        options);
}

//TODO: use this method to play a beep in main loop, 1 second long for example.
boolean MechVentilation::getStartWasTriggeredByPatient()
{ //returns true if last respiration cycle was started by patient trigger. It is cleared when read.
    if (_startWasTriggeredByPatient)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//TODO: use this method to play a beep in main loop, 2 second long for example.
boolean MechVentilation::getSensorErrorDetected()
{ //returns true if there was an sensor error detected. It is cleared when read.
    if (_sensor_error_detected)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void MechVentilation::start(void)
{
    _running = true;
}

void MechVentilation::stop(void)
{
    _running = false;
}

uint8_t MechVentilation::getRPM(void)
{
    return _rpm;
}
short MechVentilation::getExsuflationTime(void)
{
    return _timeoutEsp;
}
short MechVentilation::getInsuflationTime(void)
{
    return _timeoutIns;
}

short MechVentilation::getPeakInspiratoryPressure(void)
{
    return _pip;
}

short MechVentilation::getPeakEspiratoryPressure(void)
{
    return _peep;
}

State MechVentilation::getState(void)
{
    return _currentState;
}

void MechVentilation::setRPM(uint8_t rpm)
{
    _rpm = rpm;
    _setInspiratoryCycle();
}

void MechVentilation::setPeakInspiratoryPressure(float pip)
{
    _pip = pip;
}

void MechVentilation::setPeakEspiratoryPressure(float peep)
{
    _peep = peep;
}

void MechVentilation::_setInspiratoryCycle(void)
{
    float timeoutCycle = ((float)60) * 1000 / ((float)_rpm); // Tiempo de ciclo en msegundos
    _timeoutIns = timeoutCycle * DEFAULT_POR_INSPIRATORIO / 100;
    _timeoutEsp = (timeoutCycle) - _timeoutIns;
}

void MechVentilation::evaluatePressure(void)
{
    if (_currentPressure > ALARM_MAX_PRESSURE)
    {
        digitalWrite(PIN_BUZZ, HIGH);
        _currentAlarm = Alarm_Overpressure;
    }
    // else if (_currentPressure < ALARM_MIN_PRESSURE)
    // {
        // digitalWrite(PIN_BUZZ, HIGH);
        // _currentAlarm = Alarm_Underpressure;
    // }
    else
    {
        if (_currentAlarm != No_Alarm) {
            digitalWrite(PIN_BUZZ, LOW);
            _currentAlarm = No_Alarm;
        }
    }

    // Valve
    if (_currentPressure > VALVE_MAX_PRESSURE)
    {
        digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);
    }
}

void MechVentilation::activateRecruitment(void)
{
    _nominalConfiguration.pip = _pip;
    _nominalConfiguration.timeoutIns = _timeoutIns;
    _pip = DEFAULT_RECRUITMENT_PIP;
    _timeoutIns = DEFAULT_RECRUITMENT_TIMEOUT;
    _recruitmentMode = true;
}

void MechVentilation::deactivateRecruitment(void)
{
    _pip = _nominalConfiguration.pip;
    _timeoutIns = _nominalConfiguration.timeoutIns;
    _recruitmentMode = false;
    _setState(Init_Exsufflation);
}

/**
 * It's called from timer1Isr
 */
void MechVentilation::update(void)
{

    static int totalCyclesInThisState = 0;
    static int currentTime = 0;
    static int flowSetpoint = 0;

#if DEBUG_STATE_MACHINE
    extern volatile String debugMsg[];
    extern volatile byte debugMsgCounter;
#endif


  _msecTimerCnt=(unsigned long)(millis()-_msecTimerStartCycle);
  
    SensorPressureValues_t pressures = _sensors->getRelativePressureInCmH20();
    _currentPressure = pressures.pressure1;
    // @dc unused
    // _currentVolume = _sensors->getVolume().volume;
    //_currentFlow = _sensors->getFlow();
    if (pressures.state != SensorStateOK)
    {                                  // Sensor error detected: return to zero position and continue from there
        _sensor_error_detected = true; //An error was detected in sensors
        /* Status update, for this time */
        // TODO: SAVE PREVIOUS CYCLE IN MEMORY AND RERUN IT
        Serial.println("fail sensor");
        _setState(State_Exsufflation);
    }
    else
    {
        _sensor_error_detected = false; //clear flag
    }

    // Check pressures
    evaluatePressure();

    refreshWatchDogTimer();

    switch (_currentState)
    {
    case Init_Insufflation:
    {
#if DEBUG_UPDATE
        Serial.println("Starting insuflation");
#endif
        // Close Solenoid Valve
        digitalWrite(PIN_SOLENOID, SOLENOID_CLOSED);

        // Reset volume
        _sensors->resetVolumeIntegrator();

        totalCyclesInThisState = (_timeoutIns) / TIME_BASE;

        _msecTimerStartCycle=millis();  //Luciano

        /* Stepper control: set acceleration and end-position */

        #ifdef ACCEL_STEPPER

        #else
        // Note: this can only be called when the motor is stopped
        //IMPORTANT FROM https://github.com/Stan-Reifel/FlexyStepper/blob/master/Documentation.md
        _stepper->setSpeedInStepsPerSecond(_stepperSpeed);
        _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_INSUFFLATION);
        _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
        #endif
        
        _pid->reset();

#if DEBUG_STATE_MACHINE
        debugMsg[debugMsgCounter++] = "State: InitInsuflation at " + String(millis());
#endif

        /* Status update, reset timer, for next time, and reset PID integrator to zero */
        _setState(State_Insufflation);

        currentTime = millis();
    }
    break;
    case State_Insufflation:
    {

        /* Stepper control: set end position */
////LUCIANO
//#if DEBUG_UPDATE
//  Serial.println("Motor:speed=" + String(_stepperSpeed) + "steps/sec");
//#endif

        // time expired
        //if (currentTime > totalCyclesInThisState)
        if(_msecTimerCnt > _msecTimeoutInsufflation)
        {
            if (!_stepper->motionComplete())
            {
                // motor not finished, force motor to stop in current position
                //_stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
                //MODIFIED
                _stepper->setTargetPositionToStop();

            }
            _setState(Init_Exsufflation);
            currentTime = millis();
            float dt=(float)(_msecTimerCnt-_msecLastUpdate)*1000.;

            if (_recruitmentMode) {
                deactivateRecruitment();
            }
        }
        else //Time has not expired (State Insufflation)
        {
            //_pid->run(_currentPressure, (float)_pip, &_stepperSpeed);

            // TODO: if _currentPressure > _pip + 5, trigger alarm
            #ifdef ACCEL_STEPPER  //LUCIANO
            
            #else
            _stepper->setSpeedInStepsPerSecond(800);
//            if (_stepperSpeed >= 0){
//                _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
//            }
//            else{
//                _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
//            }
            _stepper->setTargetPositionInSteps(-STEPPER_HIGHEST_POSITION);
            //_stepper->moveRelativeInSteps(200);
            #endif
            Serial.println("CUrrtime"+String(currentTime));
            Serial.println("Ciclos totales"+totalCyclesInThisState);
            currentTime++;
        }
    }
    break;
    case Init_Exsufflation:
    {
#if DEBUG_UPDATE
        Serial.println("Starting exsuflation");
#endif
        // Open Solenoid Valve
        digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);

        totalCyclesInThisState = _timeoutEsp / TIME_BASE;
        Serial.println("Ciclos exsuff"+String(totalCyclesInThisState));
        _sensors->saveVolume();
        _sensors->resetVolumeIntegrator();


#if DEBUG_STATE_MACHINE
        debugMsg[debugMsgCounter++] = "ExsuflationTime=" + String(totalCyclesInThisState);
#endif


        /* Stepper control*/
        #ifdef ACCEL

        #else
        _stepper->setSpeedInStepsPerSecond(_stepperSpeed);
        _stepper->setAccelerationInStepsPerSecondPerSecond(
            STEPPER_ACC_EXSUFFLATION);
        //LUCIANO
        //_stepper->setTargetPositionInSteps(
          //  STEPPER_DIR * (STEPPER_LOWEST_POSITION));
          _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
        //---------------------LUCIANO

          #endif
          
#if DEBUG_STATE_MACHINE
        debugMsg[debugMsgCounter++] = "Motor: to exsuflation at " + String(millis());
#endif

        _pid->reset();

        /* Status update and reset timer, for next time */
        _setState(State_Exsufflation);
        currentTime = millis();
    }
    break;
    case State_Exsufflation:
    {
        Serial.println("exsuflacion"+String(currentTime));
//#if 0
//        if (_stepper->motionComplete())
//        {
//            if (currentFlow < _triggerThreshold && _hasTrigger)
//            { // The start was triggered by patient
//                _startWasTriggeredByPatient = true;
//
//#if DEBUG_STATE_MACHINE
//                debugMsg[debugMsgCounter++] = "!!!! Trigered by patient";
//#endif
//
//                /* Status update, for next time */
//                _setState(Init_Insufflation);
//            }
//        }
//#endif
        // Time has expired
        //if (currentTime > totalCyclesInThisState)
        if(_msecTimerCnt > _msecTimeoutExsufflation) 
        {
            if (!_stepper->motionComplete())
            {
                // motor not finished, force motor to stop in current position
                //BUG
                //_stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
                _stepper->setTargetPositionToStop();
            }
            /* Status update and reset timer, for next time */
            _setState(Init_Insufflation);
            _startWasTriggeredByPatient = false;
            
            _msecTimerStartCycle=millis();
            currentTime = 0;
        }
        else    //Time hasnot expired
        {
            //_pid->run(_currentPressure, (float)_peep, &_stepperSpeed);

//LUCIANO
//              _stepper->setSpeedInStepsPerSecond(abs(_stepperSpeed));
//            if (_stepperSpeed >= 0)
//            {
//                _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
//            }
//            else
//            {
//                _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
//            }
//-----------------
       //_stepper-> moveRelativeInSteps(-200);
       _stepper->setSpeedInStepsPerSecond(800);
       _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);

            //Serial.println("CUrrtime"+String(currentTime));
            //currentTime++;
        }
    }
    break;

    case State_Homing:
    {
        // Open Solenoid Valve
        digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);

        if (_sensor_error_detected)
        {
            // error sensor reading
            _running = false;
#if DEBUG_UPDATE
            Serial.println("Sensor: FAILED");
#endif
        }

        /*
                 * If not in home, do Homing.
                 * 0: stepper is in home
                 * 1: stepper is not in home
                 */

        if (digitalRead(PIN_ENDSTOP))
        {

            /* Stepper control: homming */
#if DEBUG_UPDATE
            Serial.println("Attempting homing...");
#endif
            if (_stepper->moveToHomeInSteps(
                    STEPPER_HOMING_DIRECTION,
                    STEPPER_HOMING_SPEED,
                    STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS,
                    PIN_ENDSTOP) != true)
            {
#if DEBUG_UPDATE
                    Serial.println("Homing failed");
#endif
            }
        }

        /* Status update and reset timer, for next time */
        currentTime = 0;
        _setState(Init_Exsufflation);
    }
    break;

    case State_Error:
        break;
    default:
        //TODO
        break;
    }
}

void MechVentilation::_init(
    FlexyStepper *stepper,
    Sensors *sensors,
    AutoPID *pid,
    VentilationOptions_t options)
{
    /* Set configuration parameters */
    _stepper = stepper;
    _sensors = sensors;
    _pid = pid;
    _rpm = options.respiratoryRate;
    _pip = options.peakInspiratoryPressure;
    _peep = options.peakEspiratoryPressure;
    setRPM(_rpm);
    _hasTrigger = options.hasTrigger;
    if (_hasTrigger)
    {
        _triggerThreshold = options.triggerThreshold;
    }
    else
    {
        _triggerThreshold = FLT_MAX;
    }

    /* Initialize internal state */
    _currentState = State_Homing;
    _stepperSpeed = STEPPER_SPEED_DEFAULT;

    //
    // connect and configure the stepper motor to its IO pins
    //
    //;
    #ifdef ACCEL_STEPPER
    #else
    _stepper->connectToPins(PIN_STEPPER_STEP, PIN_STEPPER_DIRECTION);
    _stepper->setStepsPerRevolution(STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS);
    #endif

    _sensor_error_detected = false;
}

void MechVentilation::_setState(State state)
{
    _currentState = state;
}

void MechVentilation::_setAlarm(Alarm alarm)
{
    _currentAlarm = alarm;
}
