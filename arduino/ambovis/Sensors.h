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
    float flow_f;
} SensorParams;

#endif
