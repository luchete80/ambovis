//
// Created by Mirian Quinteros on 26/02/2022.
//

#ifndef LOOPTIMEACTIONS_H
#define LOOPTIMEACTIONS_H

#include "defaults.h"
#include "DpFluxCalculator.h"

typedef struct {
    float pressure_p;
    float vlevel;
    float p_dpt;
    float flux;
} SensorData;

typedef struct {
   float flux_fil[5];
   float flow_f;
   float mlInsVol;
   float mlExsVol;
   float pressure_max;
   float pressure_min;
} SensorProcessedData;

SensorData readSensorData(int16_t pressureFromPin, int16_t mpxLevelFromPin, int16_t adc0, float vzero) {
    SensorData sensorData;
    sensorData.pressure_p = ( pressureFromPin / (1023.) - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20;//MPX5010
    sensorData.vlevel = float(mpxLevelFromPin)/1024.*1.1*VOLTAGE_CONV;
    float voltage = (adc0 * 0.1875) * 0.001; //Volts
    //With constant correction
    sensorData.p_dpt = ( (voltage - vzero)/sensorData.vlevel - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20; //WITH TRIM
    sensorData.flux = findFlux(sensorData.p_dpt);
    return sensorData;
}

SensorProcessedData updateProcessedSensorData(SensorData sensorData, bool filter, float time, float lastReadSensor, SensorProcessedData oldData) {
    SensorProcessedData newSensorProcessedData;
    newSensorProcessedData.mlInsVol = oldData.mlInsVol;
    newSensorProcessedData.mlExsVol = oldData.mlExsVol;
    newSensorProcessedData.pressure_max = oldData.pressure_max;
    newSensorProcessedData.pressure_min = oldData.pressure_min;

    newSensorProcessedData.flow_f = sensorData.flux;
    if (filter) {
        for (int i = 0; i < 4; i++) {
            newSensorProcessedData.flux_fil[i] = oldData.flux_fil[i + 1];
        }
        newSensorProcessedData.flux_fil[4] = sensorData.flux;
        float flux_sum = 0.;
        for (int i = 0; i < 5; i++) {
            flux_sum += newSensorProcessedData.flux_fil[i];
        }
        newSensorProcessedData.flow_f = flux_sum / 5.;
    }

    float fluxInMl = newSensorProcessedData.flow_f * float((time - lastReadSensor)) * 0.001;//flux in l and time in msec, results in ml
    if (sensorData.flux > 0) {
        newSensorProcessedData.mlInsVol += fluxInMl;
    } else {
        newSensorProcessedData.mlInsVol -= fluxInMl;
    }

    if (sensorData.pressure_p > newSensorProcessedData.pressure_max) {
        newSensorProcessedData.pressure_max = sensorData.pressure_p;
    }
    if (sensorData.pressure_p < newSensorProcessedData.pressure_min) {
        newSensorProcessedData.pressure_min = sensorData.pressure_p;
    }

    return newSensorProcessedData;
}

#endif //LOOPTIMEACTIONS_H
