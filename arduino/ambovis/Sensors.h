#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <stdint.h>
//#include "src/Adafruit_Sensor/Adafruit_Sensor.h"
//#include "src/Adafruit_BMP280/Adafruit_BMP280.h"
#include "src/Pressure_Sensor/Pressure_Sensor.h"
#include "defaults.h"

extern float pressure_max,pressure_min,pressure_peep;
extern float pressure_p;  //DIFFERENTIALS!
extern float last_pressure_max,last_pressure_min,last_pressure_peep;

          
extern float _flux;
//extern Adafruit_BMP280 _pres1Sensor;

#ifdef DEBUG_FLUX
extern float err_sum;
extern unsigned long ciclo;
extern float ins_prom;
#endif
extern float verror;

#endif
