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
    byte respiratory_rate;
    short peak_ins_pressure;
    short peak_exp_pressure;
    byte perc_IE;
    byte perc_volume;
    int stepper_speed_max;
    int stepper_accel_max;
} Ventilation_Config_t;

typedef struct ventilation_status {
    volatile State current_state;
    volatile unsigned long ms_timer_cnt;
    unsigned long cycle;
    unsigned long last_cycle;
    volatile byte cycle_pos;
    bool running;
    volatile float timeout_cycle;
    unsigned int time_ins;
    unsigned int time_exp;
    volatile unsigned long start_cycle_time_ms;
    int c_dyn_pass[3];
    int ml_last_ins_vol;
    int ml_last_exp_vol;
    int last_max_pressure;
    int last_min_pressure;
    float c_dyn;
    volatile bool ending_while_moving;
    volatile bool ended_while_moving;
    volatile bool update_display;
} Ventilation_Status_t;

typedef struct mechanical_ventilation {
    AccelStepper * stepper;
    Ventilation_Config_t config;
    Ventilation_Status_t status;
} Mechanical_Ventilation_t;

void start(Mechanical_Ventilation_t& mech_vent);
void stop(Mechanical_Ventilation_t& mech_vent);
void update(Mechanical_Ventilation_t& mech_vent, SensorData& sensor);
void update_config(Mechanical_Ventilation_t& mech_vent);
void ccw_search_home(AccelStepper* stepper);
void cw_search_home(AccelStepper* stepper);

#endif /* MECHANICAL_VENTILATION_H */
