#include "MechVentilation.h"

unsigned long _msecTimerStartCycle;



int PID_KP=400.01;
int PID_KI=20.01;
int PID_KD=50.01;
int STEPPER_ACC_INSUFFLATION = STEPPER_MICROSTEPS * 1500;
int STEPPER_SPEED_MAX        = STEPPER_MICROSTEPS * 1500;
int STEPPER_ACCEL_MAX        = STEPPER_MICROSTEPS * 1500;

float dpip;
byte dpip_b;

float f_acc;
byte f_acc_b;
byte  p_acc;
bool ended_whilemov;

MechVentilation::MechVentilation(
        #ifdef ACCEL_STEPPER
        AccelStepper *stepper,
      #else
        FlexyStepper *stepper,
      #endif
    AutoPID *pid,
    VentilationOptions_t options)
{

    _init(
        stepper,
        pid,
        options);
        force_stop = false;
        stopped = false;
        force_start = false;
}

//TODO: use this method to play a beep in main loop, 1 second long for example.
boolean MechVentilation::getStartWasTriggeredByPatient()
{ //returns true if last respiration cycle was started by patient trigger. It is cleared when read.
    if (_startWasTriggeredByPatient) {
        return true;
    } else {
        return false;
    }
}

//TODO: use this method to play a beep in main loop, 2 second long for example.
boolean MechVentilation::getSensorErrorDetected() { //returns true if there was an sensor error detected. It is cleared when read.
    if ( _sensor_error_detected ) {
        return true;
    } else {
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

void MechVentilation::setPeakEspiratoryPressure(float peep) {
    _peep = peep;
}

void MechVentilation::_setInspiratoryCycle(void) {
    timeoutCycle = ((float)60) * 1000.0f / ((float)_rpm); // Tiempo de ciclo en msegundos
    _timeoutIns = timeoutCycle / (float(_percIE+1));
    _timeoutEsp = (timeoutCycle) - _timeoutIns;
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
void MechVentilation :: update( SensorData& sensorData )
{
   _msecTimerCnt=(unsigned long)(millis()-_msecTimerStartCycle);
  
    int extra_time=0;
    if (_currentState == State_Exsufflation) extra_time=_timeoutIns;
    cycle_pos=byte( (float) ( (_msecTimerCnt+(float)extra_time)/(float)timeoutCycle * 127.0f) );

    if (force_start) {
      stopped=false;
      force_start = false;
    }
          
    if (!stopped) {
        if (force_stop){
            force_stop = false;
            stopped = true;
            digitalWrite(PIN_STEPPER, LOW);
            return;
        }
    switch (_currentState)
    {
    case Init_Insufflation:
    {
        resetLimitsForInitInsufflation(_mllastInsVol, _mllastExsVol, sensorData);


        last_pressure_max=pressure_max;
        last_pressure_min=pressure_min;
        pressure_max=0;
        pressure_min=60;

        // Close Solenoid Valve

        totalCyclesInThisState = (_timeoutIns) / TIME_BASE;

        _msecTimerStartCycle=millis();  //Luciano
        
        for (int i=0;i<2;i++) Cdyn_pass[i]=Cdyn_pass[i+1];
        Cdyn_pass[2]=_mllastInsVol/(last_pressure_max-last_pressure_min);
        Cdyn=(Cdyn_pass[0]+Cdyn_pass[1]+Cdyn_pass[2])/3.;
        _mllastInsVol=int(sensorData.ml_ins_vol);
        _mllastExsVol=int(fabs(sensorData.ml_exs_vol));
        //_mlInsVol2=0;
        _mlInsVol=0.;
        _mlExsVol=0.;
        
        wait_NoMove=false;
        /* Stepper control: set acceleration and end-position */

        _stepper->setSpeed(STEPPER_SPEED_MAX);
        _stepper->moveTo(-STEPPER_HIGHEST_POSITION);
        _stepper->setAcceleration(STEPPER_ACCEL_MAX);

        _setState(State_Insufflation);

        display_needs_update = true;
        curr_ended_whilemov = false;

    }// INIT INSUFFLATION
    break;
    case State_Insufflation:
    {
        if (_msecTimerCnt > _timeoutIns)
        {
            if (_stepper->distanceToGo() != 0 ) {
                curr_ended_whilemov = true;
            }
            _setState(Init_Exsufflation);
        }
    }
    break;
    case Init_Exsufflation:
    {
        ended_whilemov = curr_ended_whilemov;
      
        _msecTimerStartCycle=millis();

        _stepper->setAcceleration(STEPPER_ACCEL_MAX);
        _stepper->setSpeed(STEPPER_SPEED_EXSUFF);
        _stepper->moveTo(STEPPER_LOWEST_POSITION);

        _setState(State_Exsufflation);

    }
    break;
    case State_Exsufflation:
    {
        if (_msecTimerCnt > _timeoutEsp) {
            if (_stepper->currentPosition()==STEPPER_LOWEST_POSITION) {
                _setState(Init_Insufflation);
                _msecTimerStartCycle=millis();
                _cyclenum++;  //THIS ALWAYS SGOULD BE PRESENT
            }
        }
    }
    break;
    case State_Homing:
    {
        _setState(Init_Insufflation);
    }
    break;
    }

    }//!stopped
      
}//update

void MechVentilation::_init(
#ifdef ACCEL_STEPPER
    AccelStepper *stepper,
#else
    FlexyStepper *stepper,
#endif

    AutoPID *pid,
    VentilationOptions_t options)
{
    /* Set configuration parameters */
    _stepper = stepper;
    _pid = pid;
    _rpm = options.respiratoryRate;
    _pip = options.peakInspiratoryPressure;
    _peep = options.peakEspiratoryPressure;
    _tidalVol=options.tidalVolume;
    _percIE= options.percInspEsp;
    _percVol=options.percVolume;
    
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
    _stepper->setStepsPerRevolution(1600);
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

float MechVentilation::getInsVol() {
    return (_mllastInsVol+_mllastExsVol)/2.;
}

void MechVentilation::change_config(VentilationOptions_t options) {
    _rpm = options.respiratoryRate;
    _pip = options.peakInspiratoryPressure;
    _peep = options.peakEspiratoryPressure;
    _tidalVol=options.tidalVolume;
    _percIE= options.percInspEsp;
    setRPM(_rpm); //Include set inspiratory cycle
    _percVol=options.percVolume;
}
