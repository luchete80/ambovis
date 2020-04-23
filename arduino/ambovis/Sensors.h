#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <stdint.h>
#include "src/Adafruit_Sensor/Adafruit_Sensor.h"
#include "src/Adafruit_BMP280/Adafruit_BMP280.h"
#include "src/Pressure_Sensor/Pressure_Sensor.h"
#include "calc.h"

#include "defaults.h"
#ifdef ENABLED_SENSOR_VOLUME_SFM3300
//#include "src/SFM3200/sfm3000wedo.h"
#endif
#include "pinout.h"

#define SENSORS_MAX_ERRORS 5

#if ENABLED_SENSOR_VOLUME_SFM3300
#define SFM3300_OFFSET 32768
#define SFM3300_SCALE   120
#endif
    
enum SensorState {
    SensorStateOK = 0,
    SensorStateFailed = 1
};

typedef struct {
    uint8_t minPressure;
    uint8_t maxPressure;
} SensorLastPressure_t;

typedef struct {
    SensorState state;
    float pressure1;
    float pressure2;
} SensorPressureValues_t;

typedef struct {
    SensorState state;
    short volume; // ml
} SensorVolumeValue_t;

class Sensors
{
    public:
    Sensors();
    unsigned int begin(void);
    void readPressure();
    SensorLastPressure_t getLastPressure(void);
    
    SensorPressureValues_t getRelativePressureInPascals();
    SensorPressureValues_t getAbsolutePressureInPascals();
    SensorPressureValues_t getAbsolutePressureInCmH20();
    SensorPressureValues_t getRelativePressureInCmH20();
    
    SensorVolumeValue_t getVolume();
    void saveVolume(void);
    void getOffsetBetweenPressureSensors(int samples = 100);
#if ENABLED_SENSOR_VOLUME
    void readVolume(void);
    void resetVolumeIntegrator(void);
    float getFlow(void);
#endif
    private:
    void _init(void);
    //Adafruit_BME280 _pres1Sensor;
    //Adafruit_BMP280 _pres1Sensor;    
    Adafruit_BMP280 _pres2Sensor;
    //Adafruit_BMP280 _pres1Sensor;   //LUCINO
    //Adafruit_BMP280 _pres1Sensor;
    
    uint8_t _minPressure;
    uint8_t _maxPressure;
    float _pressure1;
    float _pressure2;
    float _pressureSensorsOffset = 0.0;
    SensorState _state;
    byte _errorCounter;
    volatile uint8_t _lastMinPressure;
    volatile uint8_t _lastMaxPressure;
#if ENABLED_SENSOR_VOLUME
    float _volume_ml;

    volatile float _lastVolume;
    unsigned long _lastReadFlow;
#endif


};

extern float pressure_max;
extern float pressure_min;
extern float pressure_p;  //DIFFERENTIALS!
extern float pressure_p0;
extern float _flux,_flux_0;
extern Pressure_Sensor _dpsensor;  //Used for venturi
extern Adafruit_BMP280 _pres1Sensor;

#endif
