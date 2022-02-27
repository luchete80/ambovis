#include "MechVentilation.h"

//float pressure_max;
//float pressure_min;

byte Cdyn_pass[3];

int STEPPER_ACC_INSUFFLATION = STEPPER_MICROSTEPS * 1500;
int STEPPER_SPEED_MAX        = STEPPER_MICROSTEPS * 1500;
int STEPPER_ACCEL_MAX        = STEPPER_MICROSTEPS * 1500;

//static
float speed_m,accel_m,speed_b,accel_b;
float pidk_m,pidk_b;
float pidi_m,pidi_b;
float pidd_m,pidd_b;
float dpip;
byte dpip_b;

float f_acc;
byte f_acc_b;
byte  p_acc;
bool ended_whilemov;

MechVentilation::MechVentilation(
        #if TESTING_MODE_DISABLED
        #ifdef ACCEL_STEPPER
        AccelStepper *stepper,
        #else
        FlexyStepper *stepper,
        #endif
        AutoPID *pid,
        VentilationOptions_t options) {
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
  #ifdef DEBUG_UPDATE
      Serial.print("Timeout Cycle");Serial.println(timeoutCycle);
      Serial.print("_timeoutIns");Serial.println(_timeoutIns);
      Serial.print("_timeoutEsp");Serial.println(_timeoutEsp);
  #endif
    
}

/**
 * It's called from timer1Isr
 */
void MechVentilation :: update (SystemState& systemState) {
    static int totalCyclesInThisState = 0;
    static int currentTime = 0;

    #if DEBUG_STATE_MACHINE
    extern volatile String debugMsg[];
    extern volatile byte debugMsgCounter;
    #endif

    _msecTimerCnt=(unsigned long)(millis()-_msecTimerStartCycle);
  
    int extra_time=0;
    if (_currentState == State_Exsufflation) {
      extra_time=_timeoutIns;
    }

    cycle_pos=byte( (float) ( (_msecTimerCnt+(float)extra_time)/(float)timeoutCycle * 127.0f) );

    switch (_currentState) {
    case Init_Insufflation:
    {
        last_pressure_max = pressure_max;
        last_pressure_min = pressure_min;
        pressure_max = 0;
        pressure_min = 60;

        // Close Solenoid Valve
        totalCyclesInThisState = (_timeoutIns) / TIME_BASE;

        _msecTimerStartCycle=millis();  //Luciano
        
        for(int i=0;i<2;i++) {
            Cdyn_pass[i]=Cdyn_pass[i+1];
        }
        Cdyn_pass[2]=_mllastInsVol/(last_pressure_max - last_pressure_min);
        Cdyn = (Cdyn_pass[0]+Cdyn_pass[1]+Cdyn_pass[2])/3.;

        _mllastInsVol=int(_mlInsVol);
        _mllastExsVol=int(fabs(_mlExsVol));

        _mlInsVol=0.;
        _mlExsVol=0.;

        #ifdef ACCEL_STEPPER
        #if TESTING_MODE_DISABLED
        _stepper->setSpeed(STEPPER_SPEED_MAX);
        _stepper->moveTo(-STEPPER_HIGHEST_POSITION);
        _stepper->setAcceleration(STEPPER_ACCEL_MAX);
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
            _stepperSpeed = 1.1 * STEPPER_HIGHEST_POSITION*(float(_percVol)*0.01)/( (float)(_timeoutIns*0.001) * DEFAULT_FRAC_CYCLE_VCL_INSUFF);//En [ml/s]

        #ifdef DEBUG_UPDATE
          Serial.print("Manual mode Timeout ins , speed: ");Serial.print(_timeoutIns);Serial.print(" ");Serial.println(_stepperSpeed);
        #endif
        #if TESTING_MODE_DISABLED
          _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACCEL_MAX);
        #endif //TESTING_MODE_DISABLED
          if (_stepperSpeed>STEPPER_SPEED_MAX){
                _stepperSpeed=STEPPER_SPEED_MAX;
            }
            #if TESTING_MODE_DISABLED
            _stepper->setSpeedInStepsPerSecond(_stepperSpeed);
            #endif //TESTING_MODE_DISABLED
        }
        #endif //ACCEL_STEPPER

//        _pid->reset();

        /* Status update, reset timer, for next time, and reset PID integrator to zero */
        _setState(State_Insufflation);

        currentTime = millis();
        systemState.display_needs_update = true;

    curr_ended_whilemov = false;

    }// INIT INSUFFLATION
    break;
    case State_Insufflation:
    {
//        /* Stepper control: set end position */
//        if (vent_mode==VENTMODE_VCL && _mlInsVol>_tidalVol){
//            #ifdef ACCEL_STEPPER
//              _stepper->stop(); 
//            #else
//              _stepper->setTargetPositionToStop();
//            #endif
//            //_setState(Init_Exsufflation); NOT BEGIN TO INSUFFLATE!
//            wait_NoMove=true;
//          }
        
        // time expired
        //if (currentTime > totalCyclesInThisState)
        if(_msecTimerCnt > _timeoutIns)
        {
            #if TESTING_MODE_DISABLED
            #ifdef ACCEL_STEPPER
            if (_stepper->distanceToGo() != 0 )
            #else
            if (!_stepper->motionComplete()) //LUCIANO: NEW
            #endif
            {
                curr_ended_whilemov = true;
                Serial.println("ENDED TIME WHILE MOVING");
            }
            else {
              Serial.println("Motion Complete");
            }
            #endif //TESTING_MODE_DISABLED
            _setState(Init_Exsufflation);
        }
//        else //Time has not expired (State Insufflation)
//        {
//            if (!wait_NoMove){  
//               float rem_flux;
//               if(_mlInsVol<0) //avoid first instance errors
//                rem_flux=_tidalVol/((float)(_timeoutIns-_msecTimerCnt) * DEFAULT_FRAC_CYCLE_VCL_INSUFF)*1000;//En [ml/s]
//               else
//                rem_flux=(_tidalVol-_mlInsVol)/((float)(_timeoutIns-_msecTimerCnt) * DEFAULT_FRAC_CYCLE_VCL_INSUFF )*1000.;
//               //#ifdef DEBUG_UPDATE
//               // Serial.print("flux");Serial.println(_flux);Serial.print("rem flux");Serial.println(rem_flux);
//               //#endif
//               
//               if (vent_mode==VENTMODE_VCL){
//                _pid->run(_flux,rem_flux,&_stepperSpeed);
//                //_stepperSpeed=STEPPER_SPEED_DEFAULT;
//                if (_stepperSpeed>STEPPER_SPEED_MAX_VCL)
//                  _stepperSpeed=STEPPER_SPEED_MAX_VCL;
//               } else if (vent_mode==VENTMODE_PCL) {
//
//                  _pid->run(pressure_p, (float)_pip, &_stepperSpeed);
//                  //_stepperAccel=0.75*abs( _stepperSpeed - _stepper -> getCurrentVelocityInStepsPerSecond() ) / PID_TS * 1000.;
//                  if (_stepperSpeed > STEPPER_SPEED_MAX)
//                    _stepperSpeed=STEPPER_SPEED_MAX;
//                  if (_stepperAccel > STEPPER_ACCEL_MAX)
//                    _stepperAccel=STEPPER_ACCEL_MAX;
//               }                
//               #ifdef DEBUG_UPDATE
//                Serial.print("Req accel: ");Serial.println(_stepperAccel);
//                Serial.print("Pres, pip: "); Serial.print(int(pressure_p)); Serial.print(" "); Serial.print(int(_pip));  Serial.print("reqSpeed: "); Serial.print(int(_stepperSpeed));  
//                Serial.print("Curr Speed: "); Serial.print(int(_stepper->getCurrentVelocityInStepsPerSecond()));  Serial.print("Accel: "); Serial.println(int(_stepperAccel));                    
//               #endif
//
//              if (vent_mode !=VENTMODE_MAN){  //only if auto
//                // TODO: if _currentPressure > _pip + 5, trigger alarm
//                #ifdef ACCEL_STEPPER  //LUCIANO
//                  _stepper->setSpeed(_stepperSpeed);
//                #else
//                _stepper->setSpeedInStepsPerSecond(abs(_stepperSpeed));
//                //_stepper->setAccelerationInStepsPerSecondPerSecond(abs(_stepperAccel));
//                
//                if (_stepperSpeed == 0){
//                #ifdef ACCEL_STEPPER  //LUCIANO
//
//                #else
//                  _stepper->setTargetPositionToStop();
//                #endif
//                  //Serial.print("VELOCIDAD CERO!");
//                }                  
//                if (_stepperSpeed > 0){
//                  #ifdef ACCEL_STEPPER  //LUCIANO
//                    _stepper->moveTo(STEPPER_HIGHEST_POSITION);
//                  #else
//                    _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
//                  #endif
//                }
//                else{
//                    #ifdef ACCEL_STEPPER  //LUCIANO
//                      _stepper->moveTo(STEPPER_LOWEST_POSITION);
//                    #else
//                      _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
//                    #endif
//                   // if (!_stepper->motionComplete())
//                   //   _stepper->setTargetPositionToStop();
//                }
//
//                    if ( (pressure_p)>_pip)
//                      _stepper->setTargetPositionToStop();
//              #endif
//              }//vent mode
//              //Serial.println("CUrrtime");Serial.println(_msecTimerCnt);
//              //Serial.println("timeout");Serial.println(_msecTimeoutInsufflation);
//  
//  //            if (_stepper->getCurrentPositionInSteps()==STEPPER_HIGHEST_POSITION)
//  //              _stepper->setTargetPositionToStop();
//            }//!Wait no move!
//
//        }
    }
    break;
    case Init_Exsufflation:
    {
      ended_whilemov = curr_ended_whilemov;
      Serial.println("ended_whilemov: " + String(ended_whilemov ));
      
      _msecTimerStartCycle=millis();
        totalCyclesInThisState = _timeoutEsp / TIME_BASE;

#if DEBUG_STATE_MACHINE
        debugMsg[debugMsgCounter++] = "ExsuflationTime=" + String(totalCyclesInThisState);
#endif

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
              
//      if ( _flux < 10.)
//        adding_vol=false;
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
        if(_msecTimerCnt > _timeoutEsp) 
        { 
            // Serial.println("Tiempo de volver");
        /////////////////////////// ********** ORIGINAL 
            // if (!_stepper->motionComplete())
            // {
              // // motor not finished, force motor to stop in current position
              // //BUG
              // //_stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
              // _stepper->setTargetPositionToStop();
            // }
            // /* Status update and reset timer, for next time */
            // _setState(Init_Insufflation);
            // _startWasTriggeredByPatient = false;
            // _msecTimerStartCycle=millis();
           //_cyclenum++;  //THIS ALWAYS SGOULD BE PRESENT
        ////////////////////////////////*** ORIGINAL

        //////////////////// NEW //////////////////////////
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
            #if TESTING_MODE_DISABLED
            /* Stepper control: homming */
            if (_stepper->moveToHomeInSteps(
                    STEPPER_HOMING_DIRECTION,
                    STEPPER_HOMING_SPEED,
                    4000, //ATTENTION
                    PIN_ENDSTOP) != true)
            {
#if DEBUG_UPDATE
                    Serial.println("Homing failed");
#endif
            } else{
              //_stepper->setCurrentPositionInSteps(int(STEPPER_HIGHEST_POSITION*0.12));
              
              }
            #endif//TESTING_MODE_DISABLED
        }
        else{
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

    AutoPID *pid,
    VentilationOptions_t options)
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
    _percIE= options.percInspEsp;
    _percVol=options.percVolume;
    
    setRPM(_rpm);
    _setInspiratoryCycle();
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

float MechVentilation::getInsVol() {
    return (_mllastInsVol+_mllastExsVol)/2.;
}

void MechVentilation::change_config(VentilationOptions_t options) {
    _rpm = options.respiratoryRate;
    _pip = options.peakInspiratoryPressure;
    _peep = options.peakEspiratoryPressure;
    _percIE= options.percInspEsp;
    setRPM(_rpm); //Include set inspiratory cycle
    _setInspiratoryCycle();
    _percVol=options.percVolume;
}
