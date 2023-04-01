//
// Created by Mirian Quinteros on 02/09/2022.
//

#include "sensorcalculation.h"

int findClosest(float target, float _dp[], int size) {
    int i = 0, j = size - 1, mid = 0;
    while ( j - i > 1 ) {
        mid = (i + j) / 2;
        if (target < _dp[mid]) {
            j = mid;
        } else {
            i = mid;
        }
    }
    return i;
}

float find_flux(float p_dpt, float _dp[], byte _po_flux[], int size) {
    byte pos = findClosest(p_dpt, _dp, size);
    //flux should be shifted up (byte storage issue)
    float flux = _po_flux[pos] - 100 + ( float (_po_flux[pos + 1] - 100) - float (_po_flux[pos] - 100) ) * ( p_dpt - float(_dp[pos]) ) / (float)( _dp[pos + 1] - _dp[pos]);
    flux *= 16.6667;
    return flux;
}

float get_flow(SensorData & sensorData, bool filter) {
    float flux_sum = 0.;
    float flow_f = sensorData.flux;
    if (filter) {
        for (int i = 0; i < 4; i++) {
            sensorData.flux_filter[i] = sensorData.flux_filter[i + 1];
            flux_sum += sensorData.flux_filter[i];
        }
        sensorData.flux_filter[4] = sensorData.flux;
        flux_sum += sensorData.flux_filter[4];
        flow_f = flux_sum / 5.;
    }
    return flow_f;
}

void update_vol(SensorData& sensorData, unsigned long time) {
    float vol = sensorData.flow_f * float((time - sensorData.last_read_sensor)) * 0.001;
    if (sensorData.flux > 0) {
        sensorData.ml_ins_vol += vol;//flux in l and time in msec, results in ml
    } else {
        sensorData.ml_exs_vol -= vol; //flux in l and time in msec, results in ml
    }
}

void convert_sensor_data(int16_t adc0, int pressure, int mpx_lev, SensorData& sensorData) {
    sensorData.pressure_p = (pressure/ 1023. - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20;//MPX5010
    sensorData.v_level = float(mpx_lev) / 1024. * 1.1 * VOLTAGE_CONV;
    sensorData.voltage = (float(adc0) * 0.1875) * 0.001; //Volts
}

float get_dpt(float voltage, float vzero, float v_level) {
    return ((voltage - vzero) / v_level - 0.04) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20; //WITH TRIM
}

void check_pip_and_peep(SensorData& sensorData) {
    //CHECK PIP AND PEEP (OUTSIDE ANY CYCLE!!)
    if (sensorData.pressure_p > sensorData.pressure_max) {
        sensorData.pressure_max = sensorData.pressure_p;
    }
    if (sensorData.pressure_p < sensorData.pressure_min) {
        sensorData.pressure_min = sensorData.pressure_p;
    }
}