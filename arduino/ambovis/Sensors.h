#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <stdint.h>
#include "src/Pressure_Sensor/Pressure_Sensor.h"
#include "defaults.h"

typedef struct {
    float pressure_max;
    float pressure_min;
    float pressure_p;  //DIFFERENTIALS!
    float last_pressure_max;
    float last_pressure_min;
//    float _flux;
    float flow_f;
    float verror = 0;
} SensorParams;

//extern float pressure_max,pressure_min,pressure_peep;
//extern float pressure_p;  //DIFFERENTIALS!
//extern float last_pressure_max,last_pressure_min,last_pressure_peep;
//
//extern float _flux,flow_f;
////extern Adafruit_BMP280 _pres1Sensor;
//
//extern float verror;

#endif
