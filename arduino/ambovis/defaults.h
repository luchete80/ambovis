#ifndef _DEFAULTS_H
#define _DEFAULTS_H

#include "Arduino.h"
//#define LCD_I2C 1 //IT DOES NOT WORK WITH ENCODER
//Connections and Debug Defs
#define ACCEL_STEPPER 1
//#define DEBUG_PID 1
//#define DEBUG_UPDATE  1
//#define DEBUG_STEPPER     1
//#define FILTER_FLUX 1
#define DEBUG_OFF 1 //Release version
#define BUZZER_LOW 0

// Base de tiempos. Periodo de llamada a mechVentilation.update
#define TIME_BASE                 20                                          // msec
#define TIME_BASE_MICROS         (TIME_BASE * 1000) // microsec PID refresh (for ISR)
#define TIME_STEPPER_ISR_MICROS  50                 // microsec Stepper refresh (for ISR)

#define TIME_SENSOR 10                // msec
#define TIME_SHOW 	50                  //IF OLED DISPLAY IS USED FASTER THAN 50ms GIVES ERRORS IN THE PLOTS 
#define TIME_SAVE 	5000
#define TIME_BUZZER 500
#define TIME_MUTE   60000             //msec
#define TIME_UPDATE_DISPLAY 20

#define STEPPER_MICROSTEPS 4
#define STEPPER_STEPS_PER_REVOLUTION 200
#define STEPPER_MICROSTEPS_PER_REVOLUTION (STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS)

#define STEPPER_HOMING_DIRECTION    (1)
#define STEPPER_HOMING_SPEED        (STEPPER_MICROSTEPS * 600)   // Steps/s
#define STEPPER_LOWEST_POSITION     (-5)   // Steps
#define STEPPER_HIGHEST_POSITION    ( 183 * STEPPER_MICROSTEPS)   //270º ,2500 for 270º, 2850 for 220º, 2930 for 330º
#define STEPPER_SPEED_DEFAULT       (STEPPER_MICROSTEPS *  1500)   // Steps/s
extern int STEPPER_SPEED_MAX;       //(14000)   // Steps/s  //THIS IS FOR 1600 steps in a revolution. DO NOT GO BEYOND THIS!
extern int STEPPER_ACCEL_MAX;       //(1500 * STEPPER_MICROSTEPS)
#define STEPPER_SPEED_MAX_VCL       (75 * STEPPER_MICROSTEPS)   // Steps/s  //THIS IS FOR 1600 steps in a revolution. DO NOT GO BEYOND THIS!
#define STEPPER_SPEED_EXSUFF        (450 * STEPPER_MICROSTEPS)
//#define STEPPER_ACC_EXSUFFLATION    (STEPPER_MICROSTEPS *  2000)   // Steps/s2

// Valores por defecto
#define DEFAULT_FRAC_CYCLE_VCL_INSUFF 0.75
#define DEFAULT_TRIGGER_THRESHOLD 3.0
#define DEFAULT_RPM 30
#define DEFAULT_MAX_RPM 30
#define DEFAULT_MIN_RPM 12
#define DEFAULT_POR_INSPIRATORIO 33.3333F // %
#define DEFAULT_PEAK_INSPIRATORY_PRESSURE 20.
#define DEFAULT_PEAK_ESPIRATORY_PRESSURE 5

// Presión
#define DEFAULT_PA_TO_CM_H20 0.0102F
#define DEFAULT_PSI_TO_CM_H20 70.306957F

// Recruitment
#define DEFAULT_RECRUITMENT_TIMEOUT 40000 // msec
#define DEFAULT_RECRUITMENT_PIP 40

// Válvula de emergencia
#define VALVE_MAX_PRESSURE 60 // cm H2O

// PID constants
// PID settings and gains
#define PID_MIN -20000 // TODO: check direction implementation
#define PID_MAX 20000

extern int PID_KP,PID_KI,PID_KD;

#define PID_TS TIME_BASE
#define PID_BANGBANG 8

typedef struct ventilation_options_t {
    short respiratoryRate = DEFAULT_RPM;
    short peakInspiratoryPressure = DEFAULT_PEAK_INSPIRATORY_PRESSURE;
    short peakEspiratoryPressure = DEFAULT_PEAK_ESPIRATORY_PRESSURE;
    float triggerThreshold = DEFAULT_TRIGGER_THRESHOLD;
    byte percInspEsp = 2;
    bool hasTrigger = false;
    short tidalVolume = 300;  //in ml
    byte percVolume = 100;
} VentilationOptions_t;

#define MODE_VOL_CTL 0
#define MODE_VOL_CTL 1
#define MODE_MANUAL  2

#define VENTMODE_VCL 0
#define VENTMODE_PCL 1
#define VENTMODE_MAN 2

//general variables
extern byte vent_mode;
extern bool sleep_mode;
extern bool put_to_sleep, wake_up;
extern unsigned long time;

// 5v to 1.1v dividiver, in order to use 1.1 arduino vref (more stable)
// Vo = V1 x R2/(R1 + R2)
// -----X
//     R1
// V1   X----- 
//     R2  Vo
// -----X-----
//R1: 120, R2: 470
// Amp: (120 + 470)/120 = 4.9166666
#define VOLTAGE_CONV   4.916666
#define USING_1v1_4PRESS

//Battery level voltage dividers
#define BATDIV_R1           12000
#define BATDIV_R2           470
#define BATTERY_READ 5
#define BAT_TEST
#define TIME_SHOW_BAT   15000 //MSECS
//#define TEMP_TEST
#define TIME_READ_TEMP  15000 //MSECS
#define CALIB_CYCLES  5

#endif // DEFAULTS_H
