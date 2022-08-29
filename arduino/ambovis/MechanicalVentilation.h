#ifndef MECHANICAL_VENTILATION_H
#define MECHANICAL_VENTILATION_H

#include "pinout.h"
#include "defaults.h"
#include "src/AccelStepper/AccelStepper.h"

enum State {
    Init_Insufflation = 1,
    State_Insufflation = 2,
    Init_Exufflation = 3,
    State_Exufflation = 4,
    State_Homing = 0,
    State_Error = -1
};

typedef struct variable_parameters {
    int alarm_max_pressure;
    int alarm_peep_pressure;
    int alarm_vt;
} AlarmConfig;

typedef struct ventilation_config {
    short respiratoryRate;
    short peakInspiratoryPressure;
    short peakExpiratoryPressure;
    byte percIE;
    byte percVolume;
    AlarmConfig alarmConfig;
} VentilationConfig;

typedef struct ventilation_status {
    volatile State currentState;
    volatile unsigned long msTimerCnt;
    unsigned long cycleNum;
    volatile byte cyclePosition;
    bool running;
    volatile float timeoutCycle;
    unsigned int timeIns;
    unsigned int timeExp;
    volatile unsigned long startCycleTimeInMs;
    int cDynPass[3];
    int mlLastInsVol;
    int mlLastExpVol;
    int lastMaxPressure;
    int lastMinPressure;
    float cDyn;
    volatile bool newInsufflation;
    volatile bool endingWhileMoving;
    volatile bool endedWhileMoving;
    volatile bool updateDisplay;
} VentilationStatus;

typedef struct mechanical_ventilation {
    AccelStepper * volatile stepper = new AccelStepper(AccelStepper::DRIVER, PIN_STEPPER_STEP, PIN_STEPPER_DIRECTION);
    VentilationConfig ventilationConfig;
    VentilationStatus ventilationStatus;
} MechanicalVentilation;

typedef struct sensor_data {
    int pressure_p;
    int max_pressure;
    int min_pressure;
    float mlInsVol;
    float mlExsVol;
} SensorData;

void start(MechanicalVentilation& mechanicalVentilation);
void stop(MechanicalVentilation& mechanicalVentilation);
void update(MechanicalVentilation& mechanicalVentilation);
void newInsufflationActions(VentilationStatus& status, SensorData& sensorData);
void newExufflationActions(VentilationStatus& status);
void update_config(MechanicalVentilation& mechanicalVentilation);

#endif /* MECHANICAL_VENTILATION_H */
