
#ifndef _DEFAULTS_H
#define _DEFAULTS_H

#include "Arduino.h"

//Connections and Debug Defs
//#define ACCEL_STEPPER 1
#define DEBUG_UPDATE 1 // 
#define DEBUG_OFF 1 //Release version
//#define P_HONEYWELL 1
#define RANGE_DPT 250

#define BMP_I2C 1  //Pressure Sensor
//#define LCD_I2C 1 //IT DOES NOT WORK WITH ENCODER


//#define DEBUG_STATE_MACHINE 1
//#define PRUEBAS 1 // testing over arduino without sensors

// Base de tiempos. Periodo de llamada a mechVentilation.update
#define TIME_BASE 10                 // msec
#define TIME_SENSOR 100               // msec
#define TIME_SEND_CONFIGURATION 2000 // msec
#define V_HONEY_P0 0.5145

// Sensores
#define ENABLED_SENSOR_VOLUME 1
#if ENABLED_SENSOR_VOLUME
//#define ENABLED_SENSOR_VOLUME_SFM3300 1
#endif

// Valores motor
#define STEPPER_MICROSTEPS 4
#define STEPPER_STEPS_PER_REVOLUTION 200

#define STEPPER_MICROSTEPS_PER_REVOLUTION (STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS)
//#define STEPPER_DIR 1
#define STEPPER_HOMING_DIRECTION    (1)
#define STEPPER_HOMING_SPEED        (STEPPER_MICROSTEPS * 1000)   // Steps/s
//#define STEPPER_LOWEST_POSITION     (STEPPER_MICROSTEPS *  -100)   // Steps
//#define STEPPER_HIGHEST_POSITION    (STEPPER_MICROSTEPS *   100)   // Steps
#define STEPPER_LOWEST_POSITION     (-10)   // Steps
#define STEPPER_HIGHEST_POSITION    ( 1200)   //270º
//#define STEPPER_HIGHEST_POSITION    ( 1280)   //330º
#define STEPPER_SPEED_DEFAULT       (400)   // Steps/s
#define STEPPER_SPEED_MAX           (1600*6)   // Steps/s  //THIS IS FOR 1600 steps in a revolution. DO NOT GO BEYOND THIS!
#define STEPPER_SPEED_MAX_VCL       (1200)   // Steps/s  //THIS IS FOR 1600 steps in a revolution. DO NOT GO BEYOND THIS!
#define STEPPER_SPEED_MAX_EXSUFF    (2000)
#define STEPPER_ACC_EXSUFFLATION    (STEPPER_MICROSTEPS *  1200)   // Steps/s2
#define STEPPER_ACC_INSUFFLATION    (STEPPER_MICROSTEPS *  1200)   // Steps/s2

// Valores por defecto
#define DEFAULT_HEIGHT 170 // cm
#define DEFAULT_SEX 0 // 0: varón, 1: mujer
#define DEFAULT_ML_POR_KG_DE_PESO_IDEAL 7
#define DEFAULT_MAX_VOLUMEN_TIDAL 800
#define DEFAULT_MIN_VOLUMEN_TIDAL 240
#define DEFAULT_FRAC_CYCLE_VCL_INSUFF 0.75
#define DEFAULT_TRIGGER_THRESHOLD 3.0
#define DEFAULT_RPM 14
#define DEFAULT_MAX_RPM 24
#define DEFAULT_MIN_RPM 3
#define DEFAULT_POR_INSPIRATORIO 33.3333F // %
#define DEFAULT_PEAK_INSPIRATORY_PRESSURE 25
#define DEFAULT_PEAK_ESPIRATORY_PRESSURE 5

#define V_SUPPLY_HONEY 5.23F
// #define DEFAULT_PEAK_ESPIRATORY_PRESSURE 6

#define DEFAULT_PRESSURE_V_FLUX_K1 1.

// Presión
#define DEFAULT_PA_TO_CM_H20 0.0102F
#define DEFAULT_PSI_TO_CM_H20 70.306957F

// Recruitment
#define DEFAULT_RECRUITMENT_TIMEOUT 40000 // msec
#define DEFAULT_RECRUITMENT_PIP 40

// Alarmas
#define ALARM_MAX_PRESSURE 35 // cm H2O
#define ALARM_MIN_PRESSURE 1  // cm H2O

// Válvula de emergencia
#define VALVE_MAX_PRESSURE 60 // cm H2O

// PID constants
// PID settings and gains
#define PID_MIN -20000 // TODO: check direction implementation
#define PID_MAX 20000
#define PID_KP 0.01
#define PID_KI 0.01
#define PID_KD 0.01
#define PID_TS TIME_BASE
#define PID_BANGBANG 8

// Solenoid
#define SOLENOID_CLOSED 0
#define SOLENOID_OPEN 1

typedef struct {
    short respiratoryRate;
    short peakInspiratoryPressure;
    short peakEspiratoryPressure;
    float triggerThreshold;
    byte percInspEsp;
    bool hasTrigger;
    unsigned short tidalVolume;  //in ml
    byte modeCtl;
    byte percVolume;   //For manual mode: 1 to 10
} VentilationOptions_t;

#define MODE_VOL_CTL 0
#define MODE_VOL_CTL 1
#define MODE_MANUAL  2

#define VENTMODE_VCL 0
#define VENTMODE_PCL 1
#define VENTMODE_MAN 2

//general variables
extern byte vent_mode;
extern bool send_data;

extern int max_speed,max_accel;

#endif // DEFAULTS_H
