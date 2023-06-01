/** Mechanical ventilation.
 *
 * @file MechVentilation.cpp
 *
 * This is the mechanical ventilation software module.
 * It handles the mechanical ventilation control loop.
 */

#include "MechVentilation.h"
#include <avr/wdt.h>

int currentWaitTriggerTime = 0;
int currentStopInsufflationTime = 0;
float currentFlow = 0;

/**
 * @brief Construct a new Mech Ventilation object and initialize it
 *
 * @param stepper
 * @param sensors
 * @param pid
 * @param options
 */
MechVentilation::MechVentilation(
    FlexyStepper *stepper,
    Sensors *sensors,
    AutoPID *pid,
    VentilationOptions_t options)
{
    _init(stepper, sensors, pid, options);
}

/**
 * @brief Initialize a new Mech Ventilation object
 *
 * @param stepper
 * @param sensors
 * @param pid
 * @param options
 */
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
    _ie = DEFAULT_INSPIRATORY_FRACTION;
    setRPM(_rpm);

    /* Initialize internal state */
    _currentState = State_Homing;
    _stepperSpeed = STEPPER_SPEED_DEFAULT;

    _sensor_error_detected = false;
}

/*!
 *  @brief  Start ventilation actuation
 */
void MechVentilation::start(void)
{
    _running = true;
}

/*!
 *  @brief Stop ventilation actuation
 */
void MechVentilation::stop(void)
{
    _running = false;
    digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);
    _sensors->resetPressures();
    _sensors->resetVolumeIntegrator();
}

/**
 * getters
 */

/*!
 *  @brief Get current Mech Ventilation mode
 *  @see VentilationMode
 */
VentilationMode MechVentilation::getMode(void)
{
    return _currentMode;
}

/*!
 *  @brief Get current Mech Ventilation type of control
 *  @see VentilationControl
 */
VentilationControl MechVentilation::getControl(void)
{
    return _currentControl;
}

/*!
 *  @brief Get current Mech Ventilation state
 */
State MechVentilation::getState(void)
{
    return _currentState;
}

/*!
 *  @brief Get current Start/Stop
 */
bool MechVentilation::getStartStop(void)
{
    return _running;
}

/*!
 *  @brief Get breaths per minute
 */
uint8_t MechVentilation::getRPM(void)
{
    return _rpm;
}

/*!
 *  @brief Get exsufflation timeout in seconds
 */
short MechVentilation::getExsuflationTime(void)
{
    return _timeoutEsp;
}

/*!
 *  @brief Get insufflation timeout in seconds
 */
short MechVentilation::getInsuflationTime(void)
{
    return _timeoutIns;
}

/*!
 *  @brief Get peak inspiratory pressure, or PIP, in cmH2O
 */
short MechVentilation::getPeakInspiratoryPressure(void)
{
    return _pip;
}

/*!
 *  @brief Get Positive end-expiratory pressure, or PEEP, in cmH2O
 */
short MechVentilation::getPeakEspiratoryPressure(void)
{
    return _peep;
}

/*!
 *  @brief Get flow trigger activation threshold value, in litres per minute.
 */
int MechVentilation::getTriggerThreshold(void)
{
    return _triggerThreshold;
}

/*!
 *  @brief Get inspiratory fraction, in % of the total cycle
 */
int MechVentilation::getIE(void)
{
    return _ie;
}

/*!
 *  @brief Get target flow in litres per minute
 */
int MechVentilation::getFlow(void)
{
    return _targetFlow;
}

/*!
 *  @brief Get target tidal volumen in millilitres
 */
int MechVentilation::getTidalVolume(void)
{
    return _targetTidalVolume;
}

/*!
 *  @brief  Get sensor health. It is cleared when read.
 *
 *  @return true if there was an sensor error detected, else returns false.
 *
 * @todo use this method to evaluate sensor health and beep Alarms
 * @warning this method is unused
 */
bool MechVentilation::getSensorErrorDetected()
{
    if (_sensor_error_detected)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * setters
 */

/*!
 *  @brief Set current Mech Ventilation mode
 *  @param mode controlled, assisted, or recruitment mode
 *  @see VentilationMode
 */
void MechVentilation::setMode(VentilationMode mode)
{
    _currentMode = mode;
}

/*!
 *  @brief Set current Mech Ventilation type of control
 *  @param control pressure control (PCV) or volume control (VCV)
 *  @see VentilationControl
 */
void MechVentilation::setControl(VentilationControl control)
{
    _currentControl = control;
}

/*!
 *  @brief Set current Mech Ventilation state
 *  @param state insufflation (init, state), exsufflation (init, state)
 *  @see State
 */
void MechVentilation::setState(State state)
{
    _currentState = state;
}

/*!
 *  @brief Set breaths per minute
 *  @param rpm number of breaths
 */
void MechVentilation::setRPM(uint8_t rpm)
{
    _rpm = rpm;
    _setInspiratoryCycle();
    setFlow(_targetFlow);
}

/*!
 *  @brief Set Peak inspiratory pressure
 *  @param pip pressure in cmH2O
 */
void MechVentilation::setPeakInspiratoryPressure(float pip)
{
    _pip = pip;
}

/*!
 *  @brief Set Positive end-expiratory pressure
 *  @param peep pressure in cmH2O
 */
void MechVentilation::setPeakEspiratoryPressure(float peep)
{
    _peep = peep;
}

/*!
 *  @brief Set low trigger value in litres per minute
 *  @param triggerThreshold
 */
void MechVentilation::setTriggerThreshold(int triggerThreshold)
{
    _triggerThreshold = triggerThreshold;
}

/*!
 *  @brief Set Inspiratory fraction
 *  @param ie fraction of total cycle, in %
 */
void MechVentilation::setIE(int ie)
{
    _ie = ie;
    _setInspiratoryCycle();
    setFlow(_targetFlow);
}

/*!
 *  @brief Set target flow
 *  @param flow target flow
 */
void MechVentilation::setFlow(int flow)
{
    // liters per minute
    int preMin = _targetTidalVolume * 60;
    int minFlow = (int)(preMin / _timeoutIns);
    if (flow <= minFlow)
    {
        _targetFlow = minFlow;
        return;
    }
    _targetFlow = flow;
    // _targetFlow = constrain(flow, minFlow, INT32_MAX);
}

/*!
 *  @brief Set target tidal volumen in millilitres
 *  @param tidalVolume target volume, in ml
 */
void MechVentilation::setTidalVolume(int tidalVolume)
{
    _targetTidalVolume = tidalVolume;
    setFlow(_targetFlow);
}

/*!
 *  @brief Set times for breathing cicle
 *  @see setIE
 *  @see setRPM
 */
void MechVentilation::_setInspiratoryCycle(void)
{
    float timeoutCycle = ((float)60) * 1000 / ((float)_rpm); // Tiempo de ciclo en msegundos
    _timeoutIns = timeoutCycle * _ie / 100;
    _timeoutEsp = (timeoutCycle)-_timeoutIns;
}

/**
 * recruitment
 */

/*!
 *  @brief Activate Recruitment Mode
 */
void MechVentilation::activateRecruitment(void)
{
    _nominalConfiguration.pip = _pip;
    _nominalConfiguration.timeoutIns = _timeoutIns;
    _nominalConfiguration.mode = _currentMode;
    _pip = DEFAULT_RECRUITMENT_PIP;
    _timeoutIns = DEFAULT_RECRUITMENT_TIMEOUT;
    _currentMode = Recruitment_Mode;
    setState(Init_Insufflation);
}

/*!
 *  @brief Deactivate Recruitment Mode
 */
void MechVentilation::deactivateRecruitment(void)
{
    _pip = _nominalConfiguration.pip;
    _timeoutIns = _nominalConfiguration.timeoutIns;
    _currentMode = _nominalConfiguration.mode;
    setState(Init_Exsufflation);
}

/*!
 *  @brief Open electrovalve if overpressure occurs
 */
void MechVentilation::evaluatePressure(void)
{
    // Valve
    if (_currentPressure > VALVE_MAX_PRESSURE)
    {
        digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);
    }
}

/*!
 *  @brief Update Mech Ventilation
 *  @note this method is called from timer1Isr
 *  @see timer1Isr
 */
void MechVentilation::update(void)
{
    wdt_reset();

    static int totalCyclesInThisState = 0;
    static int currentTime = 0;
    static int flowSetpoint = 0;

    if (!_running)
    {
        return;
    }

    SensorPressureValues_t pressures = _sensors->getRelativePressure();
    _currentPressure = pressures.pressure1;
    _currentFlow = _sensors->getFlow();
    _currentVolume = _sensors->getVolume().instantVolume;
    if (pressures.state != SensorStateOK)
    {
        // Sensor error detected: return to zero position and continue from there
        _sensor_error_detected = true; // An error was detected in sensors
        /* Status update, for this time */
        // @ŧodo SAVE PREVIOUS CYCLE IN MEMORY AND RERUN IT

        setState(State_Exsufflation);
    }
    else
    {
        _sensor_error_detected = false; // clear flag
    }

    // Check pressures
    evaluatePressure();

    switch (_currentState)
    {
    case Init_Insufflation:
    {

        if (_currentMode == Assisted_Mode)
        {
            // If trigger was not raised, go to exsufflation
            if (!_triggerRaised)
            {
                setState(Init_Exsufflation);
                currentTime = 0;
                break;
            }
        }

        if (_currentControl == Pressure_Control)
        {
            _pid->setGains(PID_PRESSURE_KP, PID_PRESSURE_KI, PID_PRESSURE_KD);
        }
        else if (_currentControl == Volume_Control)
        {
            _pid->setGains(PID_VOLUME_KP, PID_VOLUME_KI, PID_VOLUME_KD);
        }
        _pid->reset();

        // Close Solenoid Valve
        digitalWrite(PIN_SOLENOID, SOLENOID_CLOSED);

        // Reset volume
        _sensors->resetVolumeIntegrator();

        totalCyclesInThisState = (_timeoutIns) / TIME_BASE;

        /* Stepper control: set acceleration and end-position */
        _stepper->setSpeedInStepsPerSecond(_stepperSpeed);
        _stepper->setAccelerationInStepsPerSecondPerSecond(
            STEPPER_ACC_INSUFFLATION);
        _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);

        /* Status update, reset timer, for next time, and reset PID integrator to zero */
        setState(State_Insufflation);

        currentTime = 0;
    }
    break;

    /*
     * Insufflation
     */
    case State_Insufflation:
    {

        /* Stepper control: set end position */

        // time expired
        if (currentTime > totalCyclesInThisState)
        {
            if (!_stepper->motionComplete())
            {
                // motor not finished, force motor to stop in current position
                //_stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
            }
            setState(Init_Exsufflation);
            currentTime = 0;
            if (_currentMode == Recruitment_Mode)
            {
                deactivateRecruitment();
            }

            if (_currentMode == Assisted_Mode && _triggerRaised)
            {
                _triggerRaised = false;
            }
        }
        else
        {

            if (_currentControl == Pressure_Control)
            {
                _pid->run(_currentPressure, (float)_pip, &_stepperSpeed);
            }
            else if (_currentControl == Volume_Control)
            {
                // If volume reaches target tidal volume, stop flow
                if (_currentVolume >= _targetTidalVolume)
                {
                    _pid->run(_currentFlow, 0.0, &_stepperSpeed);

                    // Assist stepper with solenoid
                    if (_currentVolume <= _targetTidalVolume * (1.0 + STEPPER_VCV_FLOW_SOLENOID_HYSTERESIS))
                    {
                        digitalWrite(PIN_SOLENOID, SOLENOID_CLOSED);
                    }
                    else if (_currentVolume > _targetTidalVolume * (1.0 - STEPPER_VCV_FLOW_SOLENOID_HYSTERESIS) && _stepper->motionComplete())
                    {
                        digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);
                    }
                }
                else
                {
                    _pid->run(_currentFlow, (float)_targetFlow, &_stepperSpeed);
                }
            }

            _stepper->setSpeedInStepsPerSecond(abs(_stepperSpeed));
            if (_stepperSpeed >= 0)
            {
                _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
            }
            else
            {
                _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
            }

            if (_currentMode == Recruitment_Mode)
            {
                if (_currentPressure > DEFAULT_RECRUITMENT_PIP + 2)
                {
                    digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);
                }
                else if (_currentPressure < DEFAULT_RECRUITMENT_PIP - 0.5)
                {
                    digitalWrite(PIN_SOLENOID, SOLENOID_CLOSED);
                }
            }

            currentTime++;
        }
    }
    break;

    case Init_Exsufflation:
    {
        _pid->setGains(PID_PRESSURE_KP, PID_PRESSURE_KI, PID_PRESSURE_KD);

        // Open Solenoid Valve
        digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);

        totalCyclesInThisState = _timeoutEsp / TIME_BASE;
        _sensors->saveVolume();
        _sensors->resetVolumeIntegrator();

        /* Stepper control*/
        _stepper->setSpeedInStepsPerSecond(_stepperSpeed);
        _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_EXSUFFLATION);
        _stepper->setTargetPositionInSteps(STEPPER_DIR * (STEPPER_LOWEST_POSITION));

        _pid->reset();

        /* Status update and reset timer, for next time */
        setState(State_Exsufflation);
        currentTime = 0;
    }
    break;

    /*
     * Exsufflation
     */
    case State_Exsufflation:
    {

        // Time has expired
        if (currentTime > totalCyclesInThisState)
        {
            if (!_stepper->motionComplete())
            {
                // motor not finished, force motor to stop in current position
                //_stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
            }
            /* Status update and reset timer, for next time */
            setState(Init_Insufflation);

            currentTime = 0;
            break;
        }

        // Assisted mode
        if (_currentMode == Assisted_Mode)
        {
            if (_currentFlow >= _triggerThreshold)
            {
                _triggerRaised = true;
                setState(Init_Insufflation);
                currentTime = 0;
                break;
            }
            else
            {
                currentTime++;
                break;
            }
        }

        // PCV, Controlled mode
        // ---------------------------------------------------------
        _pid->run(_currentPressure, (float)_peep, &_stepperSpeed);

        _stepper->setSpeedInStepsPerSecond(abs(_stepperSpeed));
        if (_stepperSpeed >= 0)
        {
            _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
        }
        else
        {
            _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
        }

        // Assist stepper with solenoid
        if (_currentPressure <= _peep - STEPPER_PEEP_SOLENOID_HYSTERESIS)
        {
            digitalWrite(PIN_SOLENOID, SOLENOID_CLOSED);
        }
        else if (_currentPressure > _peep + STEPPER_PEEP_SOLENOID_HYSTERESIS)
        {
            digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);
        }
        currentTime++;
    }
    break;

    case State_Homing:
    {
        // Open Solenoid Valve
        digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);

#if 0
        if (_sensor_error_detected)
        {
            // error sensor reading
            _running = false;

        }
#endif
        /*
                 * If not in home, do Homing.
                 * 0: stepper is in home
                 * 1: stepper is not in home
                 */

        if (digitalRead(PIN_STEPPER_ENDSTOP))
        {

            /* Stepper control: homing */
            if (_stepper->moveToHomeInSteps(
                    STEPPER_HOMING_DIRECTION,
                    STEPPER_HOMING_SPEED,
                    STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS,
                    PIN_STEPPER_ENDSTOP) != true)
            {
                /** @todo Handle no homing */
            }
        }

        /* Status update and reset timer, for next time */
        currentTime = 0;
        _stepper->setTargetPositionInSteps(0);
        setState(Init_Exsufflation);
    }
    break;

    case State_Error:
        break;
    default:
        /** @todo Handle default */
        break;
    }
}
