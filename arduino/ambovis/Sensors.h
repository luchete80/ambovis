#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <stdint.h>
//#include "src/Adafruit_Sensor/Adafruit_Sensor.h"
//#include "src/Adafruit_BMP280/Adafruit_BMP280.h"
#include "src/Pressure_Sensor/Pressure_Sensor.h"
#include "defaults.h"

extern float pressure_max;
extern float pressure_min;
extern float pressure_p;  //DIFFERENTIALS!
extern float pressure_p0;
extern float last_pressure_max,last_pressure_min;

          
extern float _flux,_flux_0;
extern Pressure_Sensor _dpsensor;  //Used for venturi
//extern Adafruit_BMP280 _pres1Sensor;

#endif
