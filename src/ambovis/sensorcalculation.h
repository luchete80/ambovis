//
// Created by Mirian Quinteros on 02/09/2022.
//

#ifndef AMBOVIS_DPFLUX_H
#define AMBOVIS_DPFLUX_H

#include <Arduino.h>
#include "defaults.h"
#include "private_data.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
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
    float cdyn_pass[3] = {0.};
    float cdyn_avg = 0.;
} SensorData;

typedef struct calibration_data {
    byte vcorr_count = 0;
    float verror_sum = 0.;
    float verror_sum_outcycle =0.;
    float vzero = 0.;  //verror sum is intra cycle, verror_sum_outcycle is inter-cycle
    bool calibration_run = true;
    byte calib_cycle = 0;
} Calibration_Data_t;

float get_flow(SensorData & sensorData);
void update_vol(SensorData& sensorData, unsigned long time);
float find_flux(float p_dpt, float _dp[], byte _po_flux[], int size);
float get_dpt(float voltage, float v_level, float vzero);
void init_sensor(Adafruit_ADS1115& ads);
void convert_sensor_data(int16_t adc0, int pressure, int mpx_lev, SensorData& sensorData);
void update_verror_sum(Calibration_Data_t & calibration_data, SensorData sensorData);
void update_cycle_verror_sum(Calibration_Data_t& calibration_data);
void eval_max_min_pressure(SensorData& sensorData);


#endif //AMBOVIS_DPFLUX_H
