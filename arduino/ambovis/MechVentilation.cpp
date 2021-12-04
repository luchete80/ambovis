#include "MechVentilation.h"

//float pressure_max;
//float pressure_min;
byte Cdyn_pass[3];

MechVentilation::MechVentilation(
        #if TESTING_MODE_DISABLED
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
        #endif //TESTING_MODE_DISABLED
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

float MechVentilation::getTimeoutCycle() {
    return timeoutCycle;
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
void MechVentilation :: update (SystemState& systemState, SensorParams& sensorParams) {

    last_vent_time = millis();

    static int totalCyclesInThisState = 0;
    static int currentTime = 0;
    static int flowSetpoint = 0;

    _msecTimerCnt=(unsigned long)(millis()-_msecTimerStartCycle);
  
    int extra_time=0;
    if (_currentState == State_Exsufflation) {
        extra_time=_timeoutIns;
    }

    cycle_pos=byte( (float) ( (_msecTimerCnt+(float)extra_time)/(float)timeoutCycle * 127.0f) );

    switch (_currentState) {
    case Init_Insufflation:
    {
        sensorParams.last_pressure_max = sensorParams.pressure_max;
        sensorParams.last_pressure_min = sensorParams.pressure_min;
        sensorParams.pressure_max = 0;
        sensorParams.pressure_min = 60;

        // Close Solenoid Valve
        totalCyclesInThisState = (_timeoutIns) / TIME_BASE;

        _msecTimerStartCycle=millis();  //Luciano
        
        for(int i=0;i<2;i++) {
            Cdyn_pass[i]=Cdyn_pass[i+1];
        }
        Cdyn_pass[2]=_mllastInsVol/(sensorParams.last_pressure_max - sensorParams.last_pressure_min);
        Cdyn = (Cdyn_pass[0]+Cdyn_pass[1]+Cdyn_pass[2])/3.;
        _mllastInsVol=int(_mlInsVol);
        _mllastExsVol=int(fabs(_mlExsVol));
        
        _mlInsVol=0.;
        _mlExsVol=0.;
        
        wait_NoMove=false;

        #ifdef ACCEL_STEPPER
        #if TESTING_MODE_DISABLED
        _stepper->setSpeed(STEPPER_SPEED_DEFAULT);
        _stepper->moveTo(STEPPER_HIGHEST_POSITION);
        #endif //TESTING_MODE_DISABLED
        #else
        // Note: this can only be called when the motor is stopped
        //IMPORTANT FROM https://github.com/Stan-Reifel/FlexyStepper/blob/master/Documentation.md
        #if TESTING_MODE_DISABLED
        _stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_DEFAULT);
        _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_INSUFFLATION);
        #endif //TESTING_MODE_DISABLED

        if (systemState.vent_mode!=VENTMODE_MAN)  {//VCL && PCL
            #if TESTING_MODE_DISABLED
            _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
            #endif //TESTING_MODE_DISABLED
        } else { //MANUAL MODE
            long absPositionInSteps = int (STEPPER_HIGHEST_POSITION*(float)_percVol/100.);

            #if TESTING_MODE_DISABLED
            _stepper->setTargetPositionInSteps(absPositionInSteps);
            #endif //TESTING_MODE_DISABLED

            _stepperSpeed=STEPPER_HIGHEST_POSITION*(float(_percVol)*0.01)/( (float)(_timeoutIns*0.001) * DEFAULT_FRAC_CYCLE_VCL_INSUFF);//En [ml/s]
            #ifdef DEBUG_UPDATE
                Serial.print("Manual mode Timeout ins , speed: ");Serial.print(_timeoutIns);Serial.print(" ");Serial.println(_stepperSpeed);
            #endif
            #if TESTING_MODE_DISABLED
            _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACCEL_MAX);
            #endif //TESTING_MODE_DISABLED

            if (_stepperSpeed>STEPPER_SPEED_MAX) {
                _stepperSpeed=STEPPER_SPEED_MAX;
            }
            #if TESTING_MODE_DISABLED
            _stepper->setSpeedInStepsPerSecond(_stepperSpeed);
            #endif //TESTING_MODE_DISABLED
        }
        #endif //ACCEL_STEPPER

        #if TESTING_MODE_DISABLED
        _pid->reset();
        #endif //TESTING_MODE_DISABLED

        /* Status update, reset timer, for next time, and reset PID integrator to zero */
        _setState(State_Insufflation);

        currentTime = millis();
        systemState.display_needs_update = true;

        if (systemState.vent_mode == VENTMODE_PCL) {
            float speed_m;
            float accel_m;
            float speed_b;
            float accel_b;
            float pidk_m;
            float pidk_b;
            float pidi_m;
            float pidi_b;
            float pidd_m;
            float pidd_b;

            if (autopid) {
                if (change_pid_params) {
                    speed_m = (float)STEPPER_MICROSTEPS*float(max_speed-min_speed)/float(max_cd-min_cd);
                    speed_b = (float)STEPPER_MICROSTEPS*(float)max_speed-speed_m*(float)max_cd;
                    accel_m = (float)STEPPER_MICROSTEPS*float(max_accel-min_accel)/float(max_cd-min_cd);
                    accel_b = (float)STEPPER_MICROSTEPS*(float)max_accel-accel_m*(float)max_cd;
                    pidk_m = (float)(max_pidk-min_pidk)/float(max_cd-min_cd);
                    pidk_b = (float)max_pidk - pidk_m*(float)max_cd;

                    pidi_m = (float)(max_pidi-min_pidi)/float(max_cd-min_cd);
                    pidi_b = (float)max_pidi - pidi_m*(float)max_cd;
                    pidd_m = (float)(max_pidd-min_pidd)/float(max_cd-min_cd);
                    pidd_b = (float)max_pidd - pidd_m*(float)max_cd;

                    change_pid_params=false;
                }

                if ( abs ( sensorParams.last_pressure_max - _pip) > dpip ) {
                    if ( Cdyn < min_cd ) {
                        PID_KP = min_pidk * peep_fac; //Orig 250
                        STEPPER_SPEED_MAX = STEPPER_MICROSTEPS * min_speed;	//Originally 4000
                        STEPPER_ACC_INSUFFLATION = STEPPER_MICROSTEPS * min_accel;
                    } else if ( Cdyn > max_cd ) {
                        PID_KP = max_pidk*peep_fac; //orig 1000
                        STEPPER_SPEED_MAX = STEPPER_MICROSTEPS * max_speed; //Originally 12000
                        if ( _pip > p_acc ) {
                            STEPPER_ACC_INSUFFLATION = STEPPER_MICROSTEPS * max_accel * f_acc;//But the limit is calculated with range from 200 to 700
                        } else {
                            STEPPER_ACC_INSUFFLATION = STEPPER_MICROSTEPS * max_accel;
                        }
                    } else {
                        PID_KP = ( pidk_m*(float)Cdyn + pidk_b)*peep_fac;
                        STEPPER_SPEED_MAX = float(Cdyn) * speed_m + speed_b;  //Originally was 250
                        STEPPER_ACC_INSUFFLATION = (accel_m*(float)Cdyn+accel_b); //WITHOUT MICROSTEPS (ALREADY DONE IN CALC)
                    }
                    #if TESTING_MODE_DISABLED
                    _pid->setGains(PID_KP,PID_KI, PID_KD);
                    _pid->setOutputRange(-STEPPER_SPEED_MAX,STEPPER_SPEED_MAX);
                    #endif //TESTING_MODE_DISABLED
                }
            } else {//no autopid
                PID_KP=700.01;
                PID_KI=20.01;
                PID_KD=100.01;
                STEPPER_ACC_INSUFFLATION = STEPPER_MICROSTEPS *  600;
                STEPPER_SPEED_MAX=14000;
                #if TESTING_MODE_DISABLED
                _pid->setGains(PID_KP,PID_KI, PID_KD);
                _pid->setOutputRange(-STEPPER_SPEED_MAX,STEPPER_SPEED_MAX);
                #endif //TESTING_MODE_DISABLED
            }
        }

    }// INIT INSUFFLATION
    break;
    case State_Insufflation:
    {
        if ( systemState.vent_mode == VENTMODE_VCL && _mlInsVol > _tidalVol) {
            #if TESTING_MODE_DISABLED
            _stepper->setTargetPositionToStop();
            #endif //TESTING_MODE_DISABLED
            wait_NoMove=true;
        }
        // time expired
        if( _msecTimerCnt > _timeoutIns ) {
            #if TESTING_MODE_DISABLED
            #ifdef ACCEL_STEPPER
            if (_stepper->distanceToGo() != 0 )
            #else
            if (!_stepper->motionComplete()) //LUCIANO: NEW
            #endif
            {
                Serial.println("ENDED TIME WHILE MOVING");
            }
            else {
                Serial.println("Motion Complete");
            }
            #endif //TESTING_MODE_DISABLED
            _setState(Init_Exsufflation);
        }
    }
    break;
    case Init_Exsufflation:
    {
        _msecTimerStartCycle=millis();
        totalCyclesInThisState = _timeoutEsp / TIME_BASE;

        /* Stepper control*/
        #if TESTING_MODE_DISABLED
        #ifdef ACCEL_STEPPER
        _stepper->setAcceleration(STEPPER_ACCEL_MAX);
        _stepper->setSpeed(STEPPER_SPEED_EXSUFF);
        _stepper->moveTo(STEPPER_LOWEST_POSITION);
        #else
        _stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_EXSUFF);
        _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACCEL_MAX);//EXSUFF NO SE UTILIZA MAS; ES LA MAXIMA
        _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
         #endif
        _pid->reset();
        #endif// TESTING_MODE_DISABLED

        /* Status update and reset timer, for next time */
        _setState(State_Exsufflation);
    }
    break;
    case State_Exsufflation:
    {
        if(_msecTimerCnt > _timeoutEsp) {
            #if TESTING_MODE_DISABLED
            #ifdef ACCEL_STEPPER
            if (_stepper->currentPosition()==STEPPER_LOWEST_POSITION)
            /// in steps. Positive is clockwise from the 0 position.
            #else
            if (_stepper->getCurrentPositionInSteps()==STEPPER_LOWEST_POSITION)
            #endif
            {
                _setState(Init_Insufflation);
                _msecTimerStartCycle=millis();
                _cyclenum++;  //THIS ALWAYS SGOULD BE PRESENT
            }
            #endif //TESTING_MODE_DISABLED
        }
    }
    break;
    case State_Homing:
    {
        // Open Solenoid Valve
        #ifdef ACCEL_STEPPER
        #else
        if (_sensor_error_detected) { // nunca es true?
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
        if (digitalRead(PIN_ENDSTOP)) {
            #if TESTING_MODE_DISABLED
            /* Stepper control: homing */
            if (_stepper->moveToHomeInSteps(
                    STEPPER_HOMING_DIRECTION,
                    STEPPER_HOMING_SPEED,
                    4000, //ATTENTION
                    PIN_ENDSTOP) != true)
            {
                #if DEBUG_UPDATE
                    Serial.println("Homing failed");
                #endif
            } else {
                #if DEBUG_UPDATE
                    Serial.println("Homing succeded");
                #endif
            }
            #endif //TESTING_MODE_DISABLED
        } else {
            #if DEBUG_UPDATE
                Serial.println("No end stop detected.");
            #endif
        }
        #endif//ACCEL_STEPPER
        
        /* Status update and reset timer, for next time */
        currentTime = 0;
        _setState(Init_Insufflation);
    }
    break;
    }
      
}//update

void MechVentilation::_init(
        #if TESTING_MODE_DISABLED
        #ifdef ACCEL_STEPPER
        AccelStepper *stepper,
        #else
        FlexyStepper *stepper,
        #endif
        AutoPID *pid, VentilationOptions_t options)
        #else
        VentilationOptions_t options)
        #endif//TESTING_MODE_DISABLED
{
    /* Set configuration parameters */
    #if TESTING_MODE_DISABLED
    _stepper = stepper;
    _pid = pid;
    #endif //TESTING_MODE_DISABLED
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
    #if TESTING_MODE_DISABLED
    #ifdef ACCEL_STEPPER
    #else
    _stepper->connectToPins(PIN_STEPPER_STEP, PIN_STEPPER_DIRECTION);
    _stepper->setStepsPerRevolution(1600);
    #endif
    #endif //TESTING_MODE_DISABLED

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
