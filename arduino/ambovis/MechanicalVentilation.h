#ifndef MECHANICAL_VENTILATION_H
#define MECHANICAL_VENTILATION_H

#include "pinout.h"
#include "defaults.h"
#include "sensorcalculation.h"
#include "src/AccelStepper/AccelStepper.h"

enum State {
    Init_Insufflation = 1,
    State_Insufflation = 2,
    Init_Exufflation = 3,
    State_Exufflation = 4,
    State_Homing = 0,
    State_Error = -1
};

typedef struct ventilation_config {
    byte vent_mode;
    byte respiratoryRate;
    short peakInspiratoryPressure;
    short peakExpiratoryPressure;
    byte percIE;
    byte percVolume;
    int stepper_speed_max;
    int stepper_accel_max;
} VentilationConfig;

typedef struct ventilation_status {
    volatile State currentState;
    volatile unsigned long msTimerCnt;
    unsigned long cycleNum;
    unsigned long last_cycle;
    volatile byte cyclePosition;
    bool running;
    volatile float timeoutCycle;
    unsigned int timeIns;
    unsigned int timeExp;
    volatile unsigned long startCycleTimeInMs;
    int cDynPass[3];
    int mlLastInsVol;
    int mlLastExpVol;
    float pressure_max;
    float pressure_min;
    int lastMaxPressure;
    int lastMinPressure;
    float cDyn;
    volatile bool endingWhileMoving;
    volatile bool endedWhileMoving;
    volatile bool updateDisplay;
} VentilationStatus;

typedef struct mechanical_ventilation {
    AccelStepper * stepper;
    VentilationConfig ventilationConfig;
    VentilationStatus ventilationStatus;
} MechanicalVentilation;

void start(MechanicalVentilation& mechanicalVentilation);
void stop(MechanicalVentilation& mechanicalVentilation);
void update(MechanicalVentilation& mechanicalVentilation, SensorData& sensor);
void update_config(MechanicalVentilation& mechanicalVentilation);
void search_home_position(AccelStepper* stepper);

#endif /* MECHANICAL_VENTILATION_H */
