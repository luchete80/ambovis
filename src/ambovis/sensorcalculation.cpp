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

void init_sensor(Adafruit_ADS1115& ads) {
    ads.begin();
}

float find_flux(float p_dpt, float _dp[], byte _po_flux[], int size) {
    byte pos = findClosest(p_dpt, _dp, size);
    //flux should be shifted up (byte storage issue)
    float flux = _po_flux[pos] - 100 + ( float (_po_flux[pos + 1] - 100) - float (_po_flux[pos] - 100) ) * ( p_dpt - float(_dp[pos]) ) / (float)( _dp[pos + 1] - _dp[pos]); //-0.0156008/0.06787487
    flux *= 16.6667;
    return flux;
}

float get_flow(SensorData & sensorData) {
    float flux_sum = 0.;
    for (int i = 0; i < 4; i++) {
        sensorData.flux_filter[i] = sensorData.flux_filter[i + 1];
        flux_sum += sensorData.flux_filter[i];
    }
    sensorData.flux_filter[4] = sensorData.flux;
    flux_sum += sensorData.flux_filter[4];
    float flow_f = flux_sum / 5.;
    return flow_f;
}

void update_vol(SensorData& sensorData, unsigned long time) {
    float vol = sensorData.flow_f * float((time - sensorData.last_read_sensor)) * 0.001; //read sensor time
    if (sensorData.flux > 0) {
        sensorData.ml_ins_vol += vol;//flux in l and time in msec, results in ml
    } else {
        sensorData.ml_exs_vol -= vol; //flux in l and time in msec, results in ml
    }
}

void convert_sensor_data(int16_t adc0, int pressure, int mpx_lev, SensorData& sensorData) {
    sensorData.pressure_p = (pressure/ 1023. - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20;//MPX5010
    sensorData.v_level = float(mpx_lev) / 1024. * 1.1 * VOLTAGE_CONV;
    sensorData.voltage = adc0 * 0.1875 * 0.001; //Volts
}

float get_dpt(float voltage, float v_level, float vzero) {
//    Serial.println("v="+String(voltage) + ",v_level="+ String(v_level));
    return ((voltage - vzero)/v_level - 0.04) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20; //WITH TRIM
}

void eval_max_min_pressure(SensorData& sensorData) {
    //CHECK PIP AND PEEP (OUTSIDE ANY CYCLE!!)
    if (sensorData.pressure_p > sensorData.pressure_max) {
        sensorData.pressure_max = sensorData.pressure_p;
    }
    if (sensorData.pressure_p < sensorData.pressure_min) {
        sensorData.pressure_min = sensorData.pressure_p;
    }
}

void update_cycle_verror_sum(Calibration_Data_t& calibration_data) {
    float verror = calibration_data.verror_sum / float(calibration_data.vcorr_count);
    calibration_data.vcorr_count = 0.;
    calibration_data.verror_sum = 0.;
    calibration_data.calib_cycle++;
    calibration_data.verror_sum_outcycle += verror;
}

void update_verror_sum(Calibration_Data_t & calibration_data, SensorData sensorData) {
    calibration_data.vcorr_count++;
    calibration_data.verror_sum += (sensorData.voltage - 0.04 * sensorData.v_level);
}
