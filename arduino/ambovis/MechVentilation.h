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
#include "sensorcalculation.h"
#include "src/AutoPID/AutoPID.h"

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
    void update( SensorData& sensorData );

    /** Recruitment */
    void activateRecruitment(void);
    void deactivateRecruitment(void);
    /**
     * getters
     */
    bool getSensorErrorDetected();
    uint8_t getRPM(void);
    short getExsuflationTime(void);
    short getInsuflationTime(void);
    short getPeakInspiratoryPressure(void);
    short getPeakEspiratoryPressure(void);
    void forceStop(){force_stop = true;};
    void forceStart(){force_start = true;};
    bool  force_stop, stopped,force_start;
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
    const unsigned long & getMSecTimerCnt()const {return _msecTimerCnt;}
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

    bool curr_ended_whilemov;
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
    float timeoutCycle;
};

extern unsigned int _timeoutIns;
extern unsigned int _timeoutEsp;

extern int _mllastInsVol,_mllastExsVol;
extern bool display_needs_update;
extern VentilationOptions_t options;
extern MechVentilation * ventilation;
extern unsigned long last_cycle;
extern byte alarm_max_pressure,alarm_peep_pressure;
extern int alarm_vt;
extern byte cycle_pos; //0 to 127
extern byte Cdyn;
extern bool autopid;
extern bool filter;
extern byte pfmin,pfmax;
extern float peep_fac;
extern float pf_min,pf_max;
extern float dpip;
extern byte dpip_b;
extern int max_accel,min_accel,max_speed,min_speed,max_cd,min_cd,max_pidk,min_pidk;
extern int max_pidi,min_pidi;
extern int max_pidd,min_pidd;

extern float f_acc;
extern byte f_acc_b;
extern byte p_acc;

extern bool ended_whilemov;
extern float pressure_max,pressure_min;
//extern float pressure_p;
extern float last_pressure_max,last_pressure_min;

#endif /* INC_MECHANICAL_VENTILATION_H */
