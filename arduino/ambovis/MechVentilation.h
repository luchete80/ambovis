/** Mechanical ventilation.
 *
 * @file MechVentilation.h
 *
 * It handles the mechanical ventilation control loop.
 */
#ifndef INC_MECHANICAL_VENTILATION_H
#define INC_MECHANICAL_VENTILATION_H

#include <float.h>
#include <inttypes.h>
#include "pinout.h"
#include "defaults.h"
#include "src/AutoPID/AutoPID.h"
#include "Sensors.h"

#ifdef ACCEL_STEPPER
#include "src/AccelStepper/AccelStepper.h"
#else
#include "src/FlexyStepper/FlexyStepper.h"
#endif
/** States of the mechanical ventilation. */
enum State
{
    Init_Insufflation = 1,
    State_Insufflation = 2, /**< Insufflating (PID control). */
    Init_Exsufflation = 3,
    State_Exsufflation = 4, /**< Return to position 0 and wait for the patient to exsufflate. */
    State_Homing = 0,
    State_Error = -1
};

enum Alarm
{
    No_Alarm = 0,
    Alarm_Overpressure = 1,
    Alarm_Underpressure = 2,
    Alarm_No_Flux = 3
};

typedef struct {
    float pip;
    unsigned short timeoutIns;
} Configuration_t;

/**
 * This is the mechanical ventilation class.
 */
class MechVentilation
{
public:
    /**
	 * @brief Construct a new Mech Ventilation object
	 *
	 * @param stepper
	 * @param sensors
	 * @param pid
	 * @param options
	 */
    MechVentilation(
      #ifdef ACCEL_STEPPER
        AccelStepper *_stepper,
      #else
        FlexyStepper *_stepper,
      #endif
        AutoPID *pid,
        VentilationOptions_t options);

    boolean getStartWasTriggeredByPatient();
    void setVentilationCyle_WaitTime(float speedExsufflation);
    /** Start mechanical ventilation. */
    void start(void);
    /** Stop mechanical ventilation. */
    void stop(void);
    void update(void);

    /** Recruitment */
    void activateRecruitment(void);
    void deactivateRecruitment(void);
    byte _mode;
    /**
     * getters
     */
    bool getSensorErrorDetected();
    uint8_t getRPM(void);
    short getExsuflationTime(void);
    short getInsuflationTime(void);
    short getPeakInspiratoryPressure(void);
    short getPeakEspiratoryPressure(void);
    State getState(void);
    /**
     * setters
     */
    void setRPM(uint8_t rpm);
    void setPeakInspiratoryPressure(float pip);
    void setPeakEspiratoryPressure(float peep);

    float getInsVol(void);

    unsigned long getCycleNum(){return _cyclenum;};
    void setCycleNum(unsigned long cyc){_cyclenum=cyc;}
    void change_config(VentilationOptions_t);


    //LUCIANO 
    float getCurrentPressure();
    //
    
private:
    /** Initialization. */
    void _init(
        #ifdef ACCEL_STEPPER
        AccelStepper *_stepper,
        #else
        FlexyStepper *_stepper,
        #endif
        AutoPID *pid,
        VentilationOptions_t options);
#if 0
    int _calculateInsuflationPosition (void);
#endif

    /** Set state. */
    void _setState(State state);
    void _setAlarm(Alarm alarm);
#if 0
    void _increaseInsuflationSpeed (byte factor);
    void _decreaseInsuflationSpeed (byte factor);
    void _increaseInsuflation (byte factor);
    void _decreaseInsuflation (byte factor);
#endif
    void _setInspiratoryCycle(void);

    /* Configuration parameters */
    #ifdef ACCEL_STEPPER
    AccelStepper *_stepper;
    #else
    FlexyStepper *_stepper;
    #endif
    AutoPID *_pid;
    /** Flow trigger activation. */
    bool _hasTrigger;
    /** Flow trigger value in litres per minute. */
    float _triggerThreshold;
    /**  Insufflation timeout in seconds. */
    unsigned int _timeoutIns;
    /** Exsufflation timeout in seconds. */
    unsigned int _timeoutEsp;
    /** Breaths per minute */
    uint8_t _rpm;
    /** Peak inspiratory pressure */
    short volatile _pip;
    /** Peak espiratory pressure */
    short _peep;
    /** Recruitment */
    bool volatile _recruitmentMode = false;

    byte _percIE;
    byte _percVol;  //MANUAL MODE, 1 TO 10

    short _tidalVol;
    bool wait_NoMove;

    /* Configuration */
    Configuration_t _nominalConfiguration;

    /* Internal state */
    /** Current state. */
    State _currentState = State_Homing;
    Alarm _currentAlarm = No_Alarm;


        /** Timer counter in seconds. */
    //Este tambien es mio
   
    unsigned long _msecTimerCnt; //esteno necesita ser tan grande
    /**  Insufflation timeout in seconds. */
    unsigned long _cyclenum;    //Not important value, only for printing control
    

    /** Stepper speed. Steps per seconds. */
    float _stepperSpeed;
    float _stepperAccel;
    
    bool _running = false;
    bool _sensor_error_detected;
    bool _startWasTriggeredByPatient = false;
    //float _currentFlow = 0.0;
    //float _currentVolume = 0.0;
    float timeoutCycle;
};

extern byte stepper_time;
extern unsigned long last_vent_time;
extern float _mlInsVol,_mlExsVol;
extern int _mllastInsVol,_mllastExsVol;
//_mlInsVol2;
//extern float _stepperSpeed;
extern float pressure_sec,psec_max,last_psec_max;
extern unsigned long _msecTimerStartCycle; //CADA semiciclo
extern bool display_needs_update;
extern byte flux_count;
extern unsigned long flux_filter_time;
extern float flux_sum;
extern VentilationOptions_t options;
extern MechVentilation * ventilation;
extern unsigned long last_cycle;
extern byte alarm_max_pressure,alarm_peep_pressure;
extern byte cycle_pos; //0 to 127
extern byte Cdyn;
extern bool autopid;

#endif /* INC_MECHANICAL_VENTILATION_H */
