//
// Created by Mirian Quinteros on 02/09/2022.
//

#ifndef AMBOVIS_DPFLUX_H
#define AMBOVIS_DPFLUX_H

#include <Arduino.h>
#include "defaults.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "pinout.h"

#define DP_LENGTH 55
static float dp[] = { -3.074176245, -2.547210457, -2.087678384, -1.60054669, -1.216013465, -0.795599072, -0.630753349, -0.504544509, -0.365700986, -0.260808033, -0.176879848, -0.109004974, -0.05874157, -0.051474571, -0.043771552, -0.037061691, -0.029794693, -0.023012161, -0.017561913, -0.01441288, -0.012111664, -0.009325981, -0.007024765, -0.004602432, -0.002664566, 0.00090924, 0.00030358, 0, -0.000242233, -0.000837976, 0.001999305, 0.003937171, 0.006117271, 0.008176254, 0.011688636, 0.014830113, 0.020045692, 0.023566372, 0.028644966, 0.03312327, 0.039664506, 0.047781395, 0.052868293, 0.096530072, 0.151339196, 0.216332764, 0.295221736, 0.377891785, 0.491216024, 0.606462279, 0.877207832, 1.207061607, 1.563385753, 2.030351958, 2.444452733};
static byte po_flux[] = {0, 10, 20, 30, 40, 50, 55, 60, 65, 70, 75, 80, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 100, 100, 100, 100, 100, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 120, 125, 130, 135, 140, 145, 150, 160, 170, 180, 190, 200};

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

float get_flow(SensorData & sensorData, bool filter);
void update_vol(SensorData& sensorData, unsigned long time);
float find_flux(float p_dpt, float _dp[], byte _po_flux[], int size);
float get_dpt(float voltage, float v_level, float vzero);
void init_sensor(Adafruit_ADS1115& ads);
void convert_sensor_data(int16_t adc0, int pressure, int mpx_lev, SensorData& sensorData);
void update_verror_sum(Calibration_Data_t & calibration_data, SensorData sensorData);
void update_cycle_verror_sum(Calibration_Data_t& calibration_data);
void eval_max_min_pressure(SensorData& sensorData);


#endif //AMBOVIS_DPFLUX_H
