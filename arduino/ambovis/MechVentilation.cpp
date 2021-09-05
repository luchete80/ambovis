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

MechVentilation::MechVentilation(AccelStepper *stepper, AutoPID *pid, VentilationOptions_t options) {
    _init(stepper, pid, options);
}

//TODO: use this method to play a beep in main loop, 1 second long for example.
boolean MechVentilation::getStartWasTriggeredByPatient()
{ //returns true if last respiration cycle was started by patient trigger. It is cleared when read.
    return _startWasTriggeredByPatient;
}

//TODO: use this method to play a beep in main loop, 2 second long for example.
boolean MechVentilation::getSensorErrorDetected() { //returns true if there was an sensor error detected. It is cleared when read.
    return _sensor_error_detected;
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

    switch (_currentState) {
        case Init_Insufflation: {
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
        Cdyn = (Cdyn_pass[0]+Cdyn_pass[1]+Cdyn_pass[2])/3.;
        _mllastInsVol=int(_mlInsVol);
        _mllastExsVol=int(fabs(_mlExsVol));
        
        //_mlInsVol2=0;
        _mlInsVol=0.;
        _mlExsVol=0.;
        
        wait_NoMove=false;

        /* Stepper control: set acceleration and end-position */
        _stepper->setSpeed(STEPPER_SPEED_MAX);
        _stepper->moveTo(-STEPPER_HIGHEST_POSITION);
        _stepper->setAcceleration(STEPPER_ACCEL_MAX);

        /* Status update, reset timer, for next time, and reset PID integrator to zero */
        _setState(State_Insufflation);

        currentTime = millis();
        display_needs_update=true;
        break;
    }// INIT INSUFFLATION
    case State_Insufflation: {
        // time expired
        if(_msecTimerCnt > _timeoutIns) {

            if (_stepper->distanceToGo() != 0 ) {
                // motor not finished, force motor to stop in current position
                //_stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
                //MODIFIED
                _stepper->stop();
            }
            else {
              Serial.println("Motion Complete");
            }

            _setState(Init_Exsufflation);
            if (_recruitmentMode) {
                deactivateRecruitment();
            }
        }
        break;
    }
    case Init_Exsufflation: {
        _msecTimerStartCycle=millis();
        totalCyclesInThisState = _timeoutEsp / TIME_BASE;

        /* Stepper control*/
        _stepper->setAcceleration(STEPPER_ACCEL_MAX);
        _stepper->setSpeed(STEPPER_SPEED_EXSUFF);
        _stepper->moveTo(STEPPER_LOWEST_POSITION);
        #ifdef DEBUG_STEPPER
        unsigned long reltime = ventilation->getMSecTimerCnt();
        Serial.print("Exsuff. Rel Msec: ");Serial.print(reltime);Serial.print(", Abs: ");
        Serial.println(time);
        #endif

        _pid->reset();

        /* Status update and reset timer, for next time */
        _setState(State_Exsufflation);
    
        //display_needs_update=true;
        //last_pressure_max=pressure_max;
        break;
    }
    case State_Exsufflation: {
        if(_msecTimerCnt > _timeoutEsp) {
            //////////////////// NEW //////////////////////////
            if (_stepper->currentPosition()==STEPPER_LOWEST_POSITION) {
                /// in steps. Positive is clockwise from the 0 position.
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
        /////////////////// NEW //////////////////////////
        }
        break;
    }
    case State_Homing: {
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
        *
        * Now is homing without setting
        */
        if (digitalRead(PIN_ENDSTOP)) {
            /* Stepper control: homming */
            #if DEBUG_UPDATE
            //Serial.println("Attempting homing...");
            #endif
            if (_stepper->moveToHomeInSteps( STEPPER_HOMING_DIRECTION, STEPPER_HOMING_SPEED,
                    4000, //ATTENTION
                    PIN_ENDSTOP) != true) {
                #if DEBUG_UPDATE
                Serial.println("Homing failed");
                #endif
            }
        } else {
            #if DEBUG_UPDATE
            Serial.println("No end stop detected.");
            #endif
        }

        /* Status update and reset timer, for next time */
        currentTime = 0;
        _setState(Init_Insufflation);
        break;
    }
    case State_Error:
        break;
//    default:
//        //TODO
//        break;
    }
      
}//update

void MechVentilation::_init(AccelStepper *stepper, AutoPID *pid, VentilationOptions_t options)
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
