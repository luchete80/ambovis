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

float pressure_max;
float pressure_min;

static int highest_man_pos;

MechVentilation::MechVentilation(
        #ifdef ACCEL_STEPPER
        AccelStepper *stepper,
      #else
        FlexyStepper *stepper,
      #endif
    
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
    //_timeoutIns = timeoutCycle * DEFAULT_POR_INSPIRATORIO / 100;
    _timeoutIns = timeoutCycle / (float(_percIE+1));
	_timeoutEsp = (timeoutCycle) - _timeoutIns;    
	#ifdef DEBUG_UPDATE
      Serial.print("Timeout Cycle");Serial.println(timeoutCycle);
      Serial.print("_timeoutIns");Serial.println(_timeoutIns);
      Serial.print("_timeoutEsp");Serial.println(_timeoutEsp);
	#endif
    
}

//LUCIANO 
float MechVentilation::getCurrentPressure(){
  return _currentPressure;}
      
void MechVentilation::evaluatePressure(void)
{
    if (_currentPressure > ALARM_MAX_PRESSURE)
    {
        digitalWrite(PIN_BUZZ, HIGH);
        _currentAlarm = Alarm_Overpressure;
        //Serial.println("Overpressure");
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

    if (pressures.state != SensorStateOK)
    {                                  // Sensor error detected: return to zero position and continue from there
        _sensor_error_detected = true; //An error was detected in sensors
        /* Status update, for this time */
        // TODO: SAVE PREVIOUS CYCLE IN MEMORY AND RERUN IT
        Serial.println("fail sensor");
        _setState(State_Exsufflation);
    }
    else {
        _sensor_error_detected = false; //clear flag
    }

    //CHECK PIP AND PEEP (OUTSIDE ANY CYCLE!!)
    if (pressure_p>pressure_max) {
          pressure_max=pressure_p;
    }
    if (pressure_sec>psec_max) {
          psec_max=pressure_sec;
    }
    if (pressure_p < pressure_min){
        pressure_min=pressure_p;
    }

    // Check pressures
    evaluatePressure();

    refreshWatchDogTimer();

    switch (_currentState)
    {
    case Init_Insufflation:
    {
      #ifdef DEBUG_UPDATE
        Serial.print("max pressure: ");Serial.println(pressure_max);
        Serial.print("min pressure: ");Serial.println(pressure_min);        
      #endif
          
      pressure_max=0;
      psec_max=0;
      pressure_min=60;

#if DEBUG_UPDATE
        Serial.println("Starting insuflation");
        Serial.print("pressure_max");
#endif
        // Close Solenoid Valve
        digitalWrite(PIN_SOLENOID, SOLENOID_CLOSED);

        // Reset volume
        _sensors->resetVolumeIntegrator();

        totalCyclesInThisState = (_timeoutIns) / TIME_BASE;

        _msecTimerStartCycle=millis();  //Luciano
        _mlInsVol=0.;
        wait_NoMove=false;
        /* Stepper control: set acceleration and end-position */

        #ifdef ACCEL_STEPPER
        stepper->setSpeed(STEPPER_SPEED_DEFAULT);
        stepper->moveTo(STEPPER_HIGHEST_POSITION);
        #else
        // Note: this can only be called when the motor is stopped
        //IMPORTANT FROM https://github.com/Stan-Reifel/FlexyStepper/blob/master/Documentation.md
        _stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_DEFAULT);
        _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_INSUFFLATION);

        if (vent_mode!=VENTMODE_MAN)  //VCL && PCL
          _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
        else { //MANUAL MODE
          _stepper->setTargetPositionInSteps(int (STEPPER_HIGHEST_POSITION*(float)_percVol/100.));
          _stepperSpeed=STEPPER_HIGHEST_POSITION*(float(_percVol)/100.)/( (float)(_timeoutIns/1000) * DEFAULT_FRAC_CYCLE_VCL_INSUFF);//En [ml/s]
          if (_stepperSpeed>STEPPER_SPEED_MAX)
            _stepperSpeed=STEPPER_SPEED_MAX;
          #ifdef DEBUG_UPDATE
            Serial.print("Speed Man:");Serial.print(_stepperSpeed);
          #endif
          _stepper->setSpeedInStepsPerSecond(_stepperSpeed);
 
        #endif
        }

        if (vent_mode==VENTMODE_PCL){
            max_accel=(_pip-20)/20.*(5000-2000)+2000;
            max_speed=(_pip-20)/20.*(5000-2000)+2000;
            _stepper->setAccelerationInStepsPerSecondPerSecond(max_accel);          
        }
        
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
            
        if (vent_mode==VENTMODE_VCL && _mlInsVol>_tidalVol){
            _stepper->setTargetPositionToStop();
            //_setState(Init_Exsufflation); NOT BEGIN TO INSUFFLATE!
            wait_NoMove=true;
            _mllastInsVol=_mlInsVol;
          }
        
        // time expired
        //if (currentTime > totalCyclesInThisState)
        if(_msecTimerCnt > _timeoutIns)
        {
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
            _setState(Init_Exsufflation);
            _mllastInsVol=_mlInsVol;

            if (_recruitmentMode) {
                deactivateRecruitment();
            }
        }
        else //Time has not expired (State Insufflation)
        {
            if (!wait_NoMove){

              #ifdef DEBUG_UPDATE
                Serial.print("volume:");Serial.println(_mlInsVol);
              #endif
              //_mlInsVol+=float(_flux*(TIME_BASE));//flux in l and time in msec, results in ml
              //_mlInsVol+=float((_flux-_flux_0)*(millis()-last_vent_time));//flux in l and time in msec, results in ml 
              _mlInsVol+=_flux*float((millis()-last_vent_time))*0.001;//flux in l and time in msec, results in ml                  
              //#endif
                //flujo remanente   
                float rem_flux;
               if(_mlInsVol<0) //avoid first instance errors
                rem_flux=_tidalVol/((float)(_timeoutIns-_msecTimerCnt) * DEFAULT_FRAC_CYCLE_VCL_INSUFF)*1000;//En [ml/s]
               else
                rem_flux=(_tidalVol-_mlInsVol)/((float)(_timeoutIns-_msecTimerCnt) * DEFAULT_FRAC_CYCLE_VCL_INSUFF )*1000.;
               //#ifdef DEBUG_UPDATE
               // Serial.print("flux");Serial.println(_flux);Serial.print("rem flux");Serial.println(rem_flux);
               //#endif
               
               if (vent_mode==VENTMODE_VCL){
                _pid->run(_flux,rem_flux,&_stepperSpeed);
                //_stepperSpeed=STEPPER_SPEED_DEFAULT;
                if (_stepperSpeed>STEPPER_SPEED_MAX_VCL)
                  _stepperSpeed=STEPPER_SPEED_MAX_VCL;
               } else if (vent_mode==VENTMODE_PCL) {
                  if ( (pressure_p)<0)
                    _stepperSpeed=STEPPER_SPEED_DEFAULT;
                  else
                    _pid->run(pressure_p, (float)_pip, &_stepperSpeed);
                    if (_stepperSpeed > max_speed)
                      _stepperSpeed=max_speed;
               }                
//               #ifdef DEBUG_UPDATE
//                Serial.print("Speed: "); Serial.println(int(_stepperSpeed));       
//               // Serial.print("pip 30, dp");Serial.println(pressure_p - pressure_p0);                
//               #endif
               
                
//               Serial.print("Speed");Serial.println(_stepperSpeed);
                

              if (vent_mode !=VENTMODE_MAN){  //only if auto
                // TODO: if _currentPressure > _pip + 5, trigger alarm
                #ifdef ACCEL_STEPPER  //LUCIANO
                  stepper->setSpeed(_stepperSpeed);
                  stepper->moveTo(STEPPER_HIGHEST_POSITION);
                #else
                _stepper->setSpeedInStepsPerSecond(abs(_stepperSpeed));
                if (_stepperSpeed >= 0){
                    _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
                }
                else{
                    //_stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
                    if (!_stepper->motionComplete())
                      _stepper->setTargetPositionToStop();
                }
              }//vent mode
              #endif
              //Serial.println("CUrrtime");Serial.println(_msecTimerCnt);
              //Serial.println("timeout");Serial.println(_msecTimeoutInsufflation);
  
  //            if (_stepper->getCurrentPositionInSteps()==STEPPER_HIGHEST_POSITION)
  //              _stepper->setTargetPositionToStop();
            }//!Wait no move!

        }
    }
    break;
    case Init_Exsufflation:
    {
      
      _msecTimerStartCycle=millis();
      //Serial.print("Current pressure");Serial.println(_currentPressure);
      
#if DEBUG_UPDATE
        //Serial.println("Starting exsuflation");
#endif
        // Open Solenoid Valve
        digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);

        totalCyclesInThisState = _timeoutEsp / TIME_BASE;
        //Serial.println("Ciclos exsuff"+String(totalCyclesInThisState));
        _sensors->saveVolume();
        _sensors->resetVolumeIntegrator();


#if DEBUG_STATE_MACHINE
        debugMsg[debugMsgCounter++] = "ExsuflationTime=" + String(totalCyclesInThisState);
#endif


        /* Stepper control*/
        #ifdef ACCEL

        #else
        _stepper->setSpeedInStepsPerSecond(STEPPER_ACC_EXSUFFLATION);
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
    }
    break;
    case State_Exsufflation:
    {
    
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
  			//if (_stepper->motionComplete()&& _stepper->getCurrentPositionInSteps()==STEPPER_LOWEST_POSITION) {
        if (_stepper->getCurrentPositionInSteps()==STEPPER_LOWEST_POSITION) {
  				  _setState(Init_Insufflation);
  				  //_startWasTriggeredByPatient = false;
  				  _msecTimerStartCycle=millis();
            _cyclenum++;  //THIS ALWAYS SGOULD BE PRESENT
  		    }
        /////////////////// NEW ///////////////////////////
        } else    //Time hasnot expired
        {
//            _pid->run(pressure_p, (float)_peep, &_stepperSpeed);
//            _pid->run(float(pressure_p-pressure_p0), (float)_peep, &_stepperSpeed);
//LUCIANO
              _stepper->setSpeedInStepsPerSecond(1200);
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
        //digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);

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
                    2000, //ATTENTION
                    PIN_ENDSTOP) != true)
            {
#if DEBUG_UPDATE
                    Serial.println("Homing failed");
#endif
            } else{
              //_stepper->setCurrentPositionInSteps(int(STEPPER_HIGHEST_POSITION*0.12));
              
              }
            
        }
        else{
#if DEBUG_UPDATE
           Serial.println("No end stop detected.");
#endif
        }
        /* Status update and reset timer, for next time */
        currentTime = 0;
        _setState(Init_Insufflation);
    }
    break;

    case State_Error:
        break;
    default:
        //TODO
        break;
    }
      
}//update

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
    _cyclenum=0;
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

float MechVentilation::getInsVol()
{
    return _mllastInsVol;
}

void MechVentilation::change_config(VentilationOptions_t options)
{
    _rpm = options.respiratoryRate;
    _pip = options.peakInspiratoryPressure;
    _peep = options.peakEspiratoryPressure;
    _tidalVol=options.tidalVolume;
    _percIE= options.percInspEsp;
    setRPM(_rpm); //Include set inspiratory cycle
    _percVol=options.percVolume;

    _mode = options.modeCtl;
}
