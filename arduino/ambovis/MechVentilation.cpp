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
int STEPPER_SPEED_MAX=14000;

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

        //adding_vol=true;
        #ifdef DEBUG_UPDATE
          Serial.println("INSUFLACION ");        
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

        #if TESTING_MODE_DISABLED
        #ifdef ACCEL_STEPPER
        _stepper->setSpeed(STEPPER_SPEED_DEFAULT);
        _stepper->moveTo(STEPPER_HIGHEST_POSITION);
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
        _pid->reset();
        #endif //TESTING_MODE_DISABLED

        #if DEBUG_STATE_MACHINE
        debugMsg[debugMsgCounter++] = "State: InitInsuflation at " + String(millis());
        #endif

        /* Status update, reset timer, for next time, and reset PID integrator to zero */
        _setState(State_Insufflation);

        currentTime = millis();
        display_needs_update=true;

        if (vent_mode==VENTMODE_PCL) {
            if (autopid) {

                if (change_pid_params) {
                    speed_m =(float)STEPPER_MICROSTEPS*float(max_speed-min_speed)/float(max_cd-min_cd);
                    speed_b =(float)STEPPER_MICROSTEPS*(float)max_speed-speed_m*(float)max_cd;
                    accel_m =(float)STEPPER_MICROSTEPS*float(max_accel-min_accel)/float(max_cd-min_cd);
                    accel_b =(float)STEPPER_MICROSTEPS*(float)max_accel-accel_m*(float)max_cd;
                    pidk_m  =(float)(max_pidk-min_pidk)/float(max_cd-min_cd);
                    pidk_b  =(float)max_pidk - pidk_m*(float)max_cd;
                
                    pidi_m  =(float)(max_pidi-min_pidi)/float(max_cd-min_cd);
                    pidi_b  =(float)max_pidi - pidi_m*(float)max_cd;
                    pidd_m  =(float)(max_pidd-min_pidd)/float(max_cd-min_cd);
                    pidd_b  =(float)max_pidd - pidd_m*(float)max_cd;
                    //cdyn_m=
                    // max_acc,min_acc,max_speed,min_speed,max_cd,min_cd
                    change_pid_params=false;
                    //Serial.print("Speed m b:"); Serial.print(speed_m);Serial.print(" ");Serial.println(speed_b);
                    //Serial.print("Accel m b:"); Serial.print(accel_m);Serial.print(" ");Serial.println(accel_b);
                    //Serial.print("pidk m b:"); Serial.print(pidk_m);Serial.print(" ");Serial.println(pidk_b);
                }
                if ( abs ( last_pressure_max - _pip) >  dpip ) {

                    if ( Cdyn < min_cd ) {
                        PID_KP                   = min_pidk * peep_fac; //Orig 250
                        STEPPER_SPEED_MAX =        STEPPER_MICROSTEPS * min_speed;	//Originally 4000
                        STEPPER_ACC_INSUFFLATION = STEPPER_MICROSTEPS *  min_accel;
                    } else if ( Cdyn > max_cd ) {
                        PID_KP                   = max_pidk*peep_fac; //orig 1000
                        STEPPER_SPEED_MAX        = STEPPER_MICROSTEPS * max_speed; //Originally 12000
                        if (_pip>p_acc)
                            STEPPER_ACC_INSUFFLATION= STEPPER_MICROSTEPS *  max_accel * f_acc;//But the limit is calculated with range from 200 to 700
                        else
                            STEPPER_ACC_INSUFFLATION= STEPPER_MICROSTEPS *  max_accel;
                            //STEPPER_ACC_INSUFFLATION= STEPPER_MICROSTEPS *  600;
                    } else {
                        PID_KP=( pidk_m*(float)Cdyn + pidk_b)*peep_fac;
                        STEPPER_SPEED_MAX=float(Cdyn) * speed_m + speed_b;  //Originally was 250
                        STEPPER_ACC_INSUFFLATION=(accel_m*(float)Cdyn+accel_b); //WITHOUT MICROSTEPS (ALREADY DONE IN CALC)
      			    }
                    #if TESTING_MODE_DISABLED
                    _pid->setGains(PID_KP,PID_KI, PID_KD);
                    _pid->setOutputRange(-STEPPER_SPEED_MAX,STEPPER_SPEED_MAX);
                    #endif//TESTING_MODE_DISABLED
                }
            } else {//no autopid
                PID_KP=700.01;
                PID_KI=20.01;
                PID_KD=100.01;
                STEPPER_ACC_INSUFFLATION=STEPPER_MICROSTEPS *  600;
                STEPPER_SPEED_MAX=14000;
                #if TESTING_MODE_DISABLED
                _pid->setGains(PID_KP,PID_KI, PID_KD);
                _pid->setOutputRange(-STEPPER_SPEED_MAX,STEPPER_SPEED_MAX);
                #endif//TESTING_MODE_DISABLED
            }
        }//if pcl

    }// INIT INSUFFLATION
    break;
    case State_Insufflation:
    {
        /* Stepper control: set end position */
        if (vent_mode==VENTMODE_VCL && _mlInsVol>_tidalVol){
            #if TESTING_MODE_DISABLED
            _stepper->setTargetPositionToStop();
            #endif //TESTING_MODE_DISABLED
            //_setState(Init_Exsufflation); NOT BEGIN TO INSUFFLATE!
            wait_NoMove=true;
          }
        // time expired
        if(_msecTimerCnt > _timeoutIns)
        {
            #if TESTING_MODE_DISABLED
            if (!_stepper->motionComplete()) //LUCIANO: NEW
            {
                // motor not finished, force motor to stop in current position
                //_stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
                //MODIFIED
                _stepper->setTargetPositionToStop();
                #ifdef DEBUG_UPDATE
                  Serial.println("ENDED TIME WHILE MOVING");
                #endif
            }
            #endif //TESTING_MODE_DISABLED
            _setState(Init_Exsufflation);
        }
        else //Time has not expired (State Insufflation)
        {
            if (!wait_NoMove) {
                float rem_flux;
                if(_mlInsVol<0) //avoid first instance errors
                    rem_flux=_tidalVol/((float)(_timeoutIns-_msecTimerCnt) * DEFAULT_FRAC_CYCLE_VCL_INSUFF)*1000;//En [ml/s]
                else
                    rem_flux=(_tidalVol-_mlInsVol)/((float)(_timeoutIns-_msecTimerCnt) * DEFAULT_FRAC_CYCLE_VCL_INSUFF )*1000.;
                //#ifdef DEBUG_UPDATE
                // Serial.print("flux");Serial.println(_flux);Serial.print("rem flux");Serial.println(rem_flux);
                //#endif
               
                if (vent_mode==VENTMODE_VCL) {
                    #if TESTING_MODE_DISABLED
                    _pid->run(_flux,rem_flux,&_stepperSpeed);
                    #endif //TESTING_MODE_DISABLED
                    //_stepperSpeed=STEPPER_SPEED_DEFAULT;
                    if (_stepperSpeed>STEPPER_SPEED_MAX_VCL)
                        _stepperSpeed=STEPPER_SPEED_MAX_VCL;
                } else if (vent_mode==VENTMODE_PCL) {
                    #if TESTING_MODE_DISABLED
                    _pid->run(pressure_p, (float)_pip, &_stepperSpeed);
                    #endif //TESTING_MODE_DISABLED
                    //_stepperAccel=0.75*abs( _stepperSpeed - _stepper -> getCurrentVelocityInStepsPerSecond() ) / PID_TS * 1000.;
                    if (_stepperSpeed > STEPPER_SPEED_MAX)
                        _stepperSpeed=STEPPER_SPEED_MAX;
                    if (_stepperAccel > STEPPER_ACCEL_MAX)
                        _stepperAccel=STEPPER_ACCEL_MAX;
                }
                #ifdef DEBUG_UPDATE
                Serial.print("Req accel: ");Serial.println(_stepperAccel);
                Serial.print("Pres, pip: "); Serial.print(int(pressure_p)); Serial.print(" "); Serial.print(int(_pip));  Serial.print("reqSpeed: "); Serial.print(int(_stepperSpeed));  
                Serial.print("Curr Speed: "); Serial.print(int(_stepper->getCurrentVelocityInStepsPerSecond()));  Serial.print("Accel: "); Serial.println(int(_stepperAccel));                    
                #endif

                if (vent_mode !=VENTMODE_MAN){  //only if auto
                    // TODO: if _currentPressure > _pip + 5, trigger alarm
                    #if TESTING_MODE_DISABLED
                    #ifdef ACCEL_STEPPER  //LUCIANO
                    _stepper->setSpeed(_stepperSpeed);
                    _stepper->moveTo(STEPPER_HIGHEST_POSITION);
                    #else
                    _stepper->setSpeedInStepsPerSecond(abs(_stepperSpeed));
                    //_stepper->setAccelerationInStepsPerSecondPerSecond(abs(_stepperAccel));
                
                    if (_stepperSpeed == 0){
                        _stepper->setTargetPositionToStop();
                        //Serial.print("VELOCIDAD CERO!");
                    }
                    if (_stepperSpeed > 0){
                        _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
                    }
                    else {
                        _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
                        // if (!_stepper->motionComplete())
                        //   _stepper->setTargetPositionToStop();
                    }

                    if ( (pressure_p)>_pip)
                        _stepper->setTargetPositionToStop();
                    #endif
                    #endif //TESTING_MODE_DISABLED
                }//vent mode

                //Serial.println("CUrrtime");Serial.println(_msecTimerCnt);
                //Serial.println("timeout");Serial.println(_msecTimeoutInsufflation);
                //if (_stepper->getCurrentPositionInSteps()==STEPPER_HIGHEST_POSITION)
                //    _stepper->setTargetPositionToStop();
            }//!Wait no move!

        }
    }
    break;
    case Init_Exsufflation:
    {
        _msecTimerStartCycle=millis();
        totalCyclesInThisState = _timeoutEsp / TIME_BASE;

        #if DEBUG_STATE_MACHINE
        debugMsg[debugMsgCounter++] = "ExsuflationTime=" + String(totalCyclesInThisState);
        #endif

        #if TESTING_MODE_DISABLED
        /* Stepper control*/
        #ifdef ACCEL_STEPPER
        //Do nothing
        #else
        _stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_EXSUFF);
        _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_EXSUFFLATION);
        //LUCIANO
        //_stepper->setTargetPositionInSteps(
        //  STEPPER_DIR * (STEPPER_LOWEST_POSITION));
        _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
        //---------------------LUCIANO
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
            //////////////////// NEW //////////////////////////
            if (_stepper->getCurrentPositionInSteps()==STEPPER_LOWEST_POSITION) {
                _setState(Init_Insufflation);
                //_startWasTriggeredByPatient = false;
                _msecTimerStartCycle=millis();
                _cyclenum++;  //THIS ALWAYS SGOULD BE PRESENT
  		    }
            #endif //TESTING_MODE_DISABLED
            /////////////////// NEW ///////////////////////////
        } else    //Time hasnot expired
        {
            //LUCIANO
            #if TESTING_MODE_DISABLED
            _stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_EXSUFF);
            #endif //TESTING_MODE_DISABLED
        }
    }
    break;
    case State_Homing:
    {
        // Open Solenoid Valve

        if (_sensor_error_detected) {
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
            /* Stepper control: homming */
            #if TESTING_MODE_DISABLED
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
            #endif //TESTING_MODE_DISABLED
        } else {
            #if DEBUG_UPDATE
                Serial.println("No end stop detected.");
            #endif
        }


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
