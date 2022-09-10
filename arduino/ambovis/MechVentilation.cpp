#include "MechVentilation.h"

float pressure_max;
float pressure_min;

unsigned long _msecTimerStartCycle;

byte Cdyn_pass[3];

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
    AutoPID *pid)
{

    _init(
        stepper,
        pid);
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
    //_timeoutIns = timeoutCycle * DEFAULT_POR_INSPIRATORIO / 100;
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
void MechVentilation :: update()
{

    static int totalCyclesInThisState = 0;
    static int currentTime = 0;

    #if DEBUG_STATE_MACHINE
    extern volatile String debugMsg[];
    extern volatile byte debugMsgCounter;
    #endif

  _msecTimerCnt=(unsigned long)(millis()-_msecTimerStartCycle);
  
  cycle_pos=byte( (float) ( (_msecTimerCnt)/(float)timeoutCycle * 127.0f) );
  int extra_time=0;
  if (_currentState == State_Exsufflation) extra_time=_timeoutIns;
  cycle_pos=byte( (float) ( (_msecTimerCnt+(float)extra_time)/(float)timeoutCycle * 127.0f) );

//    if (pressures.state != SensorStateOK)
//    {                                  // Sensor error detected: return to zero position and continue from there
//        _sensor_error_detected = true; //An error was detected in sensors
//        /* Status update, for this time */
//        // TODO: SAVE PREVIOUS CYCLE IN MEMORY AND RERUN IT
//        Serial.println("fail sensor");
//        _setState(State_Exsufflation);
//    }
//    else {
//        _sensor_error_detected = false; //clear flag
//    }
    if (force_start) {
      stopped=false;
      force_start = false;
    }

    if (!stopped){
    switch (_currentState)
    {
    case Init_Insufflation:
    {
        if (force_stop){
          force_stop = false;
          stopped = true;
          digitalWrite(PIN_STEPPER, LOW);
          return;
        }

        last_pressure_max=pressure_max;
        last_pressure_min=pressure_min;
        pressure_max=0;
        pressure_min=60;

        // Close Solenoid Valve

        totalCyclesInThisState = (_timeoutIns) / TIME_BASE;

        _msecTimerStartCycle=millis();  //Luciano
        
        for (int i=0;i<2;i++) Cdyn_pass[i]=Cdyn_pass[i+1];
        Cdyn_pass[2]=_mllastInsVol/(last_pressure_max - last_pressure_min);
        Cdyn = (Cdyn_pass[0]+Cdyn_pass[1]+Cdyn_pass[2])/3.;
        _mllastInsVol=int(_mlInsVol);
        _mllastExsVol=int(fabs(_mlExsVol));
        
        _mlInsVol=0.;
        _mlExsVol=0.;
        
        wait_NoMove=false;
        /* Stepper control: set acceleration and end-position */

        #ifdef ACCEL_STEPPER
        _stepper->setSpeed(STEPPER_SPEED_MAX);
        _stepper->moveTo(-STEPPER_HIGHEST_POSITION);
        _stepper->setAcceleration(STEPPER_ACCEL_MAX);
        #else
        // Note: this can only be called when the motor is stopped
        //IMPORTANT FROM https://github.com/Stan-Reifel/FlexyStepper/blob/master/Documentation.md
        _stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_DEFAULT);
        _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_INSUFFLATION);

        if (this->variableParameters->vent_mode!=VENTMODE_MAN)  //VCL && PCL
          _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
        else { //MANUAL MODE
          _stepper->setTargetPositionInSteps(int (STEPPER_HIGHEST_POSITION*(float)_percVol/100.));
          _stepperSpeed = 1.1 * STEPPER_HIGHEST_POSITION*(float(_percVol)*0.01)/( (float)(_timeoutIns*0.001) * DEFAULT_FRAC_CYCLE_VCL_INSUFF);//En [ml/s]

          _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACCEL_MAX);
          if (_stepperSpeed>STEPPER_SPEED_MAX)
            _stepperSpeed=STEPPER_SPEED_MAX;
          _stepper->setSpeedInStepsPerSecond(_stepperSpeed);
        } 
        #endif

        /* Status update, reset timer, for next time, and reset PID integrator to zero */
        _setState(State_Insufflation);

        currentTime = millis();
        display_needs_update=true;
//      
//      if (vent_mode==VENTMODE_PCL){
//      if (autopid) {
//      
//          if (change_pid_params) {
//                speed_m =(float)STEPPER_MICROSTEPS*float(max_speed-min_speed)/float(max_cd-min_cd);
//                speed_b =(float)STEPPER_MICROSTEPS*(float)max_speed-speed_m*(float)max_cd;
//                accel_m =(float)STEPPER_MICROSTEPS*float(max_accel-min_accel)/float(max_cd-min_cd);
//                accel_b =(float)STEPPER_MICROSTEPS*(float)max_accel-accel_m*(float)max_cd;
//                pidk_m  =(float)(max_pidk-min_pidk)/float(max_cd-min_cd);
//                pidk_b  =(float)max_pidk - pidk_m*(float)max_cd;
//                
//                pidi_m  =(float)(max_pidi-min_pidi)/float(max_cd-min_cd);
//                pidi_b  =(float)max_pidi - pidi_m*(float)max_cd;
//                pidd_m  =(float)(max_pidd-min_pidd)/float(max_cd-min_cd);
//                pidd_b  =(float)max_pidd - pidd_m*(float)max_cd;
//                //cdyn_m=
//                // max_acc,min_acc,max_speed,min_speed,max_cd,min_cd
//                change_pid_params=false;
//                //Serial.print("Speed m b:"); Serial.print(speed_m);Serial.print(" ");Serial.println(speed_b);
//                //Serial.print("Accel m b:"); Serial.print(accel_m);Serial.print(" ");Serial.println(accel_b);
//                //Serial.print("pidk m b:"); Serial.print(pidk_m);Serial.print(" ");Serial.println(pidk_b);
//          }
//          if ( abs ( last_pressure_max - _pip) >  dpip ){
//                  
//                  if ( Cdyn < min_cd ) {
//                       PID_KP                   = min_pidk * peep_fac; //Orig 250
//                       STEPPER_SPEED_MAX =        STEPPER_MICROSTEPS * min_speed; //Originally 4000
//                       STEPPER_ACC_INSUFFLATION = STEPPER_MICROSTEPS *  min_accel;            
//                  } else if ( Cdyn > max_cd ) {
//                       PID_KP                   = max_pidk*peep_fac; //orig 1000
//                       STEPPER_SPEED_MAX        = STEPPER_MICROSTEPS * max_speed; //Originally 12000
//                       if (_pip>p_acc) 
//                        STEPPER_ACC_INSUFFLATION= STEPPER_MICROSTEPS *  max_accel * f_acc;//But the limit is calculated with range from 200 to 700
//                       else         
//                        STEPPER_ACC_INSUFFLATION= STEPPER_MICROSTEPS *  max_accel;
//                        //STEPPER_ACC_INSUFFLATION= STEPPER_MICROSTEPS *  600;
//                       
//                  }
//                  else {
//
//                      PID_KP=( pidk_m*(float)Cdyn + pidk_b)*peep_fac;
//                      STEPPER_SPEED_MAX=float(Cdyn) * speed_m + speed_b;  //Originally was 250
//                      STEPPER_ACC_INSUFFLATION=(accel_m*(float)Cdyn+accel_b); //WITHOUT MICROSTEPS (ALREADY DONE IN CALC)
//              }
//              _pid->setGains(PID_KP,PID_KI, PID_KD);
//              _pid->setOutputRange(-STEPPER_SPEED_MAX,STEPPER_SPEED_MAX);
//          }
//      } else {//no autopid
//              PID_KP=700.01;
//              PID_KI=20.01;
//              PID_KD=100.01;
//              STEPPER_ACC_INSUFFLATION=STEPPER_MICROSTEPS *  600;
//              STEPPER_SPEED_MAX=14000;              
//              _pid->setGains(PID_KP,PID_KI, PID_KD);
//              _pid->setOutputRange(-STEPPER_SPEED_MAX,STEPPER_SPEED_MAX);     
//      }
//      }//if pcl

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
            #ifdef ACCEL_STEPPER
            if (_stepper->distanceToGo() != 0 )
            #else
            if (!_stepper->motionComplete()) //LUCIANO: NEW
            #endif
            {
                curr_ended_whilemov = true;
                // motor not finished, force motor to stop in current position
                //_stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
                //MODIFIED
//                #ifdef ACCEL_STEPPER
//                _stepper->stop(); 
//                #else 
//                _stepper->setTargetPositionToStop();
//                #endif
                
//                
                //#ifdef DEBUG_UPDATE
                #ifdef DEBUG_STEPPER
                Serial.println("ENDED TIME WHILE MOVING");
                #endif
                //#endif
            }
            else {
              Serial.println("Motion Complete");
            }
            _setState(Init_Exsufflation);
            if (_recruitmentMode) {
                deactivateRecruitment();
            }
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
      
      _msecTimerStartCycle=millis();
      //Serial.print("Current pressure");Serial.println(_currentPressure);
      
//#if DEBUG_UPDATE
//        Serial.println("Starting exsuflation");
//#endif

        totalCyclesInThisState = _timeoutEsp / TIME_BASE;

#if DEBUG_STATE_MACHINE
        debugMsg[debugMsgCounter++] = "ExsuflationTime=" + String(totalCyclesInThisState);
#endif

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
          
#if DEBUG_STATE_MACHINE
        debugMsg[debugMsgCounter++] = "Motor: to exsuflation at " + String(millis());
#endif

        _pid->reset();

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
        #ifdef ACCEL_STEPPER
         if (_stepper->currentPosition()==STEPPER_LOWEST_POSITION) 
            /// in steps. Positive is clockwise from the 0 position.
        #else
        if (_stepper->getCurrentPositionInSteps()==STEPPER_LOWEST_POSITION) 
        #endif
        {
        
            _setState(Init_Insufflation);
            //_startWasTriggeredByPatient = false;
      #ifdef DEBUG_STEPPER
      unsigned long reltime = ventilation->getMSecTimerCnt();
      Serial.print("End Exsuff. Rel Msec: ");Serial.print(reltime);Serial.print(", Abs: ");
      Serial.print(time);Serial.print(" Exsuff time: ");Serial.println(_timeoutEsp);
      #endif
      
            _msecTimerStartCycle=millis();
            _cyclenum++;  //THIS ALWAYS SGOULD BE PRESENT
          }
        /////////////////// NEW ///////////////////////////
        } else    //Time hasnot expired
        {
//            _pid->run(pressure_p, (float)_peep, &_stepperSpeed);
//            _pid->run(float(pressure_p-pressure_p0), (float)_peep, &_stepperSpeed);
//LUCIANO
//              _stepper->setSpeedInStepsPerSecond(4000);
             // Serial.println(_stepperSpeed);
//            if (_stepperSpeed >= 0)
//                _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
//            else
//                _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);


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
    
      #endif//ACCEL_STEPPER
        
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

    }//!stopped
      
}//update

void MechVentilation::_init(
#ifdef ACCEL_STEPPER
    AccelStepper *stepper,
#else
    FlexyStepper *stepper,
#endif

    AutoPID *pid)
{
    /* Set configuration parameters */
    _stepper = stepper;
    _pid = pid;
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

void MechVentilation::updateParameters() {
    _rpm = variableParameters->respiratoryRate;
    _percIE= byte(variableParameters->percInspEsp);
    _percVol= byte(variableParameters->percVolume);
    _setInspiratoryCycle();
    Serial.println("params updated");
}
