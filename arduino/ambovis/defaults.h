#ifndef _DEFAULTS_H
#define _DEFAULTS_H

#include "Arduino.h"
//#define LCD_I2C 1 //IT DOES NOT WORK WITH ENCODER
//Connections and Debug Defs
#define ACCEL_STEPPER 1
//#define DEBUG_PID 1
//#define DEBUG_UPDATE  1
#define DEBUG_STEPPER     1
//#define FILTER_FLUX 1
#define DEBUG_OFF 1 //Release version
#define BUZZER_LOW 0

#define BMP_I2C 1  //Pressure Sensor

//#define PRUEBAS 1 // testing over arduino without sensors

// Base de tiempos. Periodo de llamada a mechVentilation.update
#define TIME_BASE                 20                                          // msec
#define TIME_BASE_MICROS         (TIME_BASE * 1000) // microsec PID refresh (for ISR)
#define TIME_STEPPER_ISR_MICROS  50                 // microsec Stepper refresh (for ISR)

#define TIME_SENSOR 10                // msec
#define TIME_SHOW 	50                  //IF OLED DISPLAY IS USED FASTER THAN 50ms GIVES ERRORS IN THE PLOTS 
#define TIME_SAVE 	5000
#define TIME_BUZZER 500
#define TIME_MUTE   60000             //msec

#define TIME_SEND_CONFIGURATION 2000 // msec
#define V_HONEY_P0 	0.49874F //Analog/1023

#define TIME_BUZZER 500

// Sensores
#define ENABLED_SENSOR_VOLUME 1
#if ENABLED_SENSOR_VOLUME
//#define ENABLED_SENSOR_VOLUME_SFM3300 1
#endif


#define STEPPER_MICROSTEPS 4
#define STEPPER_STEPS_PER_REVOLUTION 200
#define STEPPER_MICROSTEPS_PER_REVOLUTION (STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS)
//#define STEPPER_DIR 1
#define STEPPER_HOMING_DIRECTION    (1)
#define STEPPER_HOMING_SPEED        (STEPPER_MICROSTEPS * 600)   // Steps/s
//#define STEPPER_LOWEST_POSITION     (STEPPER_MICROSTEPS *  -100)   // Steps
//#define STEPPER_HIGHEST_POSITION    (STEPPER_MICROSTEPS *   100)   // Steps
#define STEPPER_LOWEST_POSITION     (-5)   // Steps
#define STEPPER_HIGHEST_POSITION    ( 183 * STEPPER_MICROSTEPS)   //270º ,2500 for 270º, 2850 for 220º, 2930 for 330º
#define STEPPER_SPEED_DEFAULT       (STEPPER_MICROSTEPS *  1500)   // Steps/s
extern int STEPPER_SPEED_MAX;       //(14000)   // Steps/s  //THIS IS FOR 1600 steps in a revolution. DO NOT GO BEYOND THIS!
extern int STEPPER_ACCEL_MAX;       //(1500 * STEPPER_MICROSTEPS)
#define STEPPER_SPEED_MAX_VCL       (75 * STEPPER_MICROSTEPS)   // Steps/s  //THIS IS FOR 1600 steps in a revolution. DO NOT GO BEYOND THIS!
#define STEPPER_SPEED_EXSUFF        (450 * STEPPER_MICROSTEPS)
//#define STEPPER_ACC_EXSUFFLATION    (STEPPER_MICROSTEPS *  2000)   // Steps/s2

// Valores por defecto
#define DEFAULT_HEIGHT 170 // cm
#define DEFAULT_SEX 0 // 0: varón, 1: mujer
#define DEFAULT_ML_POR_KG_DE_PESO_IDEAL 7
#define DEFAULT_MAX_VTIDAL 800
#define DEFAULT_MIN_VTIDAL 240
#define DEFAULT_FRAC_CYCLE_VCL_INSUFF 0.75
#define DEFAULT_TRIGGER_THRESHOLD 3.0
#define DEFAULT_RPM 30
#define DEFAULT_MAX_RPM 30
#define DEFAULT_MIN_RPM 12
#define DEFAULT_POR_INSPIRATORIO 33.3333F // %
#define DEFAULT_PEAK_INSPIRATORY_PRESSURE 25
#define DEFAULT_PEAK_ESPIRATORY_PRESSURE 5

#define V_SUPPLY_HONEY 5.0F
// #define DEFAULT_PEAK_ESPIRATORY_PRESSURE 6

#define DEFAULT_PRESSURE_V_FLUX_K1 1.
// Presión
#define DEFAULT_PA_TO_CM_H20 0.0102F
#define DEFAULT_PSI_TO_CM_H20 70.306957F

// Recruitment
#define DEFAULT_RECRUITMENT_TIMEOUT 40000 // msec
#define DEFAULT_RECRUITMENT_PIP 40

// Alarmas
#define ALARM_MIN_PRESSURE 1  // cm H2O

// Válvula de emergencia
#define VALVE_MAX_PRESSURE 60 // cm H2O

// PID constants
// PID settings and gains
#define PID_MIN -20000 // TODO: check direction implementation
#define PID_MAX 20000

extern int PID_KP,PID_KI,PID_KD;

#define PID_TS TIME_BASE
#define PID_BANGBANG 8

// Solenoid
#define SOLENOID_CLOSED 0
#define SOLENOID_OPEN 1

class VentilationOptions_t {

  public:
  short respiratoryRate;
  short peakInspiratoryPressure;
  short peakEspiratoryPressure;
  float triggerThreshold;
  byte percInspEsp;
  bool hasTrigger;
  short tidalVolume;  //in ml
  byte modeCtl;
  byte percVolume;   //For manual mode: 1 to 10

  VentilationOptions_t(){}
  ~VentilationOptions_t(){}
};

#define MODE_VOL_CTL 0
#define MODE_VOL_CTL 1
#define MODE_MANUAL  2

#define VENTMODE_VCL 0
#define VENTMODE_PCL 1
#define VENTMODE_MAN 2

//general variables
extern byte vent_mode;
extern bool send_data;
extern bool sleep_mode;

extern byte alarm_state;

extern bool put_to_sleep,wake_up;
//PID LIMITS
extern bool change_pid_params;
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
#define TIME_VOLTAGE_SENSOR 50                // msec

#define VMIN_MPX            4.5               //For battery charge reference
//Battery level voltage dividers
#define BATDIV_R1           12000
#define BATDIV_R2           470
extern float vlevel;


#endif // DEFAULTS_H
