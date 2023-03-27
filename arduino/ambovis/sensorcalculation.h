//
// Created by Mirian Quinteros on 02/09/2022.
//

#ifndef AMBOVIS_DPFLUX_H
#define AMBOVIS_DPFLUX_H

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_ADS1X15.h"
#include "defaults.h"
#include "pinout.h"

typedef struct sensor_data {
    float pressure_p = 0.;
    float pressure_max = 0.;
    float pressure_min = 1000.;
    float v_level = 0.;
    float voltage = 0.;
    float flux_filter[5] = {0.};
    float flux = 0.;
    float flow_f = 0.;
    float ml_ins_vol = 0.;
    float ml_exs_vol = 0.;
    unsigned long last_read_sensor = 0L;
} SensorData;

float findFlux(float p_dpt);
void readSensor(Adafruit_ADS1115& ads, SensorData& result, float vzero, bool filter);

#endif //AMBOVIS_DPFLUX_H
