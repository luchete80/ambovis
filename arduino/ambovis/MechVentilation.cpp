#include "MechVentilation.h"

int currentWaitTriggerTime = 0;
int currentStopInsufflationTime = 0;
float currentFlow = 0;

float pressure_max;
float pressure_min;

static int highest_man_pos;
unsigned long _msecTimerStartCycle;

byte Cdyn_pass[3];

int PID_KP=400.01;
int PID_KI=20.01;
int PID_KD=50.01;
int STEPPER_ACC_INSUFFLATION=STEPPER_MICROSTEPS *  600;
int STEPPER_SPEED_MAX=STEPPER_MICROSTEPS *  900;

//static
float speed_m,accel_m,speed_b,accel_b;
float pidk_m,pidk_b;
float pidi_m,pidi_b;
float pidd_m,pidd_b;
float dpip;
byte dpip_b;

float f_acc;byte f_acc_b;
byte  p_acc;

MechVentilation::MechVentilation(
        #if FOR_TEST
        #ifdef ACCEL_STEPPER
        AccelStepper *stepper,
        #else
        FlexyStepper *stepper,
        #endif
        AutoPID *pid,
        VentilationOptions_t options)
{
        _init(stepper, pid, options);
        #else
            VentilationOptions_t options) {
            _init(options);
        #endif //FOR_TEST
}

void MechVentilation::start(void)
{
    _running = true;
}

void MechVentilation::stop(void)
{
    _running = false;
}

bool MechVentilation::isRunning() {
    return _running;
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
    //_timeoutIns = timeoutCycle * DEFAULT_POR_INSPIRATORIO / 100;
    _timeoutIns = timeoutCycle / (float(_percIE+1));
    _timeoutEsp = (timeoutCycle) - _timeoutIns;    
  #ifdef DEBUG_UPDATE
      Serial.print("Timeout Cycle");Serial.println(timeoutCycle);
      Serial.print("_timeoutIns");Serial.println(_timeoutIns);
      Serial.print("_timeoutEsp");Serial.println(_timeoutEsp);
  #endif
    
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
void MechVentilation :: update ( void )
{
    last_vent_time = millis();

    static int totalCyclesInThisState = 0;
    static int currentTime = 0;
    static int flowSetpoint = 0;

    _msecTimerCnt=(unsigned long)(millis()-_msecTimerStartCycle);
  
    cycle_pos=byte( (float) ( (_msecTimerCnt)/(float)timeoutCycle * 127.0f) );
    int extra_time=0;
    if (_currentState == State_Exsufflation) {
        extra_time=_timeoutIns;
    }

    cycle_pos=byte( (float) ( (_msecTimerCnt+(float)extra_time)/(float)timeoutCycle * 127.0f) );

    switch (_currentState)
    {
    case Init_Insufflation:
    {

        //Filter vars
        #ifdef FLUX_FILTER
        flux_filter_time=millis();
        flux_count=0;
        #endif
  
        last_pressure_max=pressure_max;
        last_pressure_min=pressure_min;
        pressure_max=0;
        pressure_min=60;

        // Close Solenoid Valve

        totalCyclesInThisState = (_timeoutIns) / TIME_BASE;

        _msecTimerStartCycle=millis();  //Luciano
        
        for(int i=0;i<2;i++) {
            Cdyn_pass[i]=Cdyn_pass[i+1];
        }
        Cdyn_pass[2]=_mllastInsVol/(last_pressure_max-last_pressure_min);
        Cdyn=(Cdyn_pass[0]+Cdyn_pass[1]+Cdyn_pass[2])/3.;
        _mllastInsVol=int(_mlInsVol);
        _mllastExsVol=int(fabs(_mlExsVol));
        
        //_mlInsVol2=0;
        _mlInsVol=0.;
        _mlExsVol=0.;
        
        wait_NoMove=false;

        #if FOR_TEST
        #ifdef ACCEL_STEPPER
        /* Stepper control: set acceleration and end-position */
        _stepper->setSpeed(STEPPER_SPEED_MAX);
        _stepper->moveTo(-STEPPER_HIGHEST_POSITION);
        _stepper->setAcceleration(STEPPER_ACCEL_MAX);
        #else
        // Note: this can only be called when the motor is stopped
        //IMPORTANT FROM https://github.com/Stan-Reifel/FlexyStepper/blob/master/Documentation.md
        _stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_DEFAULT);
        _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_INSUFFLATION);

        if (vent_mode!=VENTMODE_MAN)  {//VCL && PCL
            _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
        } else { //MANUAL MODE
            _stepper->setTargetPositionInSteps(int (STEPPER_HIGHEST_POSITION*(float)_percVol/100.));
            _stepperSpeed=STEPPER_HIGHEST_POSITION*(float(_percVol)*0.01)/( (float)(_timeoutIns*0.001) * DEFAULT_FRAC_CYCLE_VCL_INSUFF);//En [ml/s]
            #ifdef DEBUG_UPDATE
                Serial.print("Manual mode Timeout ins , speed: ");Serial.print(_timeoutIns);Serial.print(" ");Serial.println(_stepperSpeed);
            #endif
            _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACCEL_MAX);
            if (_stepperSpeed>STEPPER_SPEED_MAX)
                _stepperSpeed=STEPPER_SPEED_MAX;
            _stepper->setSpeedInStepsPerSecond(_stepperSpeed);
        }
        #endif
        #endif //FOR_TEST

        /* Status update, reset timer, for next time, and reset PID integrator to zero */
        _setState(State_Insufflation);

        currentTime = millis();
        display_needs_update=true;

    }// INIT INSUFFLATION
    break;
    case State_Insufflation:
    {
        // time expired
        if(_msecTimerCnt > _timeoutIns)
        {
            #if FOR_TEST
            #ifdef ACCEL_STEPPER
            if (_stepper->distanceToGo() != 0 )
            #else
            if (!_stepper->motionComplete()) //LUCIANO: NEW
            #endif
            {
                // motor not finished, force motor to stop in current position
                //_stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
                //MODIFIED
                #ifdef ACCEL_STEPPER
                _stepper->stop();
                #else
                _stepper->setTargetPositionToStop();
                #endif
            }
            #endif
            _setState(Init_Exsufflation);
        }
    }
    break;
    case Init_Exsufflation:
    {
      _msecTimerStartCycle=millis();
        totalCyclesInThisState = _timeoutEsp / TIME_BASE;

        #if FOR_TEST
        /* Stepper control*/
        #ifdef ACCEL_STEPPER
        _stepper->setAcceleration(STEPPER_ACCEL_MAX);
        _stepper->setSpeed(STEPPER_SPEED_EXSUFF);
        _stepper->moveTo(STEPPER_LOWEST_POSITION);
        #ifdef DEBUG_STEPPER
        unsigned long reltime = ventilation->getMSecTimerCnt();
        Serial.print("Exsuff. Rel Msec: ");Serial.print(reltime);Serial.print(", Abs: ");
        Serial.println(time);
        #endif
        #else
        _stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_EXSUFF);
        _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACCEL_MAX);//EXSUFF NO SE UTILIZA MAS; ES LA MAXIMA
        //LUCIANO
        //_stepper->setTargetPositionInSteps(
        //  STEPPER_DIR * (STEPPER_LOWEST_POSITION));
        _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
        //---------------------LUCIANO
         #endif
        _pid->reset();
        #endif// FOR_TEST

        /* Status update and reset timer, for next time */
        _setState(State_Exsufflation);
    }
    break;
    case State_Exsufflation:
    {
        if(_msecTimerCnt > _timeoutEsp)
        {

        //////////////////// NEW //////////////////////////
            #if FOR_TEST
            #ifdef ACCEL_STEPPER
            if (_stepper->currentPosition()==STEPPER_LOWEST_POSITION)
            /// in steps. Positive is clockwise from the 0 position.
            #else
            if (_stepper->getCurrentPositionInSteps()==STEPPER_LOWEST_POSITION)
            #endif
            {

                _setState(Init_Insufflation);
                #ifdef DEBUG_STEPPER
                unsigned long reltime = ventilation->getMSecTimerCnt();
                Serial.print("End Exsuff. Rel Msec: ");Serial.print(reltime);Serial.print(", Abs: ");
                Serial.print(time);Serial.print(" Exsuff time: ");Serial.println(_timeoutEsp);
                #endif

                _msecTimerStartCycle=millis();
                _cyclenum++;  //THIS ALWAYS SGOULD BE PRESENT
            }
            /////////////////// NEW //////////////////////////
            #endif //FOR_TEST
        }
    }
    break;

    case State_Homing:
    {
        #if FOR_TEST
        // Open Solenoid Valve
        #ifdef ACCEL_STEPPER
        //Do nothing
        #else

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
//        Now is homing without setting
        if (digitalRead(PIN_ENDSTOP))
        {
            /* Stepper control: homming */
            #if DEBUG_UPDATE
            //Serial.println("Attempting homing...");
            #endif
            if (_stepper->moveToHomeInSteps(
                    STEPPER_HOMING_DIRECTION,
                    STEPPER_HOMING_SPEED,
                    4000, //ATTENTION
                    PIN_ENDSTOP) != true)
            {
                #if DEBUG_UPDATE
                    Serial.println("Homing failed");
                #endif
            }
        }
        else {
            #if DEBUG_UPDATE
                Serial.println("No end stop detected.");
            #endif
        }
        #endif//ACCEL_STEPPER
        #endif //FOR_TEST

        /* Status update and reset timer, for next time */
        currentTime = 0;
        _setState(Init_Insufflation);
    }
    break;
//
//    case State_Error:
//        break;
//    default:
//        //TODO
//        break;
    }
      
}//update

void MechVentilation::_init(
        #if FOR_TEST
        #ifdef ACCEL_STEPPER
        AccelStepper *stepper,
        #else
        FlexyStepper *stepper,
        #endif
        AutoPID *pid, VentilationOptions_t options)
        #else
        VentilationOptions_t options)
        #endif//FOR_TEST
{
    /* Set configuration parameters */
    #if FOR_TEST
    _stepper = stepper;
    _pid = pid;
    #endif
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

    // connect and configure the stepper motor to its IO pins
    #if FOR_TEST
    #ifdef ACCEL_STEPPER
    #else
    _stepper->connectToPins(PIN_STEPPER_STEP, PIN_STEPPER_DIRECTION);
    _stepper->setStepsPerRevolution(1600);
    #endif
    #endif //FOR_TEST

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

    _mode = options.modeCtl;
}
