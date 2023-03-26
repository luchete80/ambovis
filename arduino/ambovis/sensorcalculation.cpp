//
// Created by Mirian Quinteros on 02/09/2022.
//

#include "sensorcalculation.h"

#define DP_LENGTH 55

static float dp[] = { -3.074176245, -2.547210457, -2.087678384, -1.60054669, -1.216013465, -0.795599072, -0.630753349, -0.504544509, -0.365700986, -0.260808033, -0.176879848, -0.109004974, -0.05874157, -0.051474571, -0.043771552, -0.037061691, -0.029794693, -0.023012161, -0.017561913, -0.01441288, -0.012111664, -0.009325981, -0.007024765, -0.004602432, -0.002664566, 0.00090924, 0.00030358, 0, -0.000242233, -0.000837976, 0.001999305, 0.003937171, 0.006117271, 0.008176254, 0.011688636, 0.014830113, 0.020045692, 0.023566372, 0.028644966, 0.03312327, 0.039664506, 0.047781395, 0.052868293, 0.096530072, 0.151339196, 0.216332764, 0.295221736, 0.377891785, 0.491216024, 0.606462279, 0.877207832, 1.207061607, 1.563385753, 2.030351958, 2.444452733};
static byte po_flux[] = {0, 10, 20, 30, 40, 50, 55, 60, 65, 70, 75, 80, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 100, 100, 100, 100, 100, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 120, 125, 130, 135, 140, 145, 150, 160, 170, 180, 190, 200};

int findClosest(float target) {
    int i = 0, j = DP_LENGTH - 1, mid = 0;
    while ( j - i > 1 ) {
        mid = (i + j) / 2;
        if (target < dp[mid]) {
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

float findFlux(float p_dpt) {
    byte pos = findClosest(p_dpt);
    //flux should be shifted up (byte storage issue)
    float flux = po_flux[pos] - 100 + ( float (po_flux[pos + 1] - 100) - float (po_flux[pos] - 100) ) * ( p_dpt - float(dp[pos]) ) / (float)( dp[pos + 1] - dp[pos]);
    flux *= 16.6667;
    return flux;
}

void read_sensor(Adafruit_ADS1115& ads, SensorData& sensorData, float vzero, bool filter) {
    int16_t adc0 = ads.readADC_SingleEnded(0);

    sensorData.pressure_p = (analogRead(PIN_PRESSURE)/ (1023.) - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20;//MPX5010
    sensorData.v_level = float(analogRead(PIN_MPX_LEV)) / 1024. * 1.1 * VOLTAGE_CONV;
    sensorData.voltage = (adc0 * 0.1875) * 0.001; //Volts

    float p_dpt = ((sensorData.voltage - vzero) / sensorData.v_level - 0.04) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20; //WITH TRIM
    sensorData.flux = findFlux(p_dpt);

    float flux_sum = 0.;
    if (filter) {
        for (int i = 0; i < 4; i++) {
            sensorData.flux_filter[i] = sensorData.flux_filter[i + 1];
            flux_sum += sensorData.flux_filter[i];
        }
        sensorData.flux_filter[4] = sensorData.flux;
        flux_sum += sensorData.flux_filter[4];

        sensorData.flow_f = flux_sum / 5.;
    } else {
        sensorData.flow_f = sensorData.flux;
    }

    float vol = sensorData.flow_f * float((millis() - sensorData.last_read_sensor)) * 0.001;
    if (sensorData.flux > 0) {
        sensorData.ml_ins_vol += vol;//flux in l and time in msec, results in ml
    } else {
        sensorData.ml_exs_vol -= vol; //flux in l and time in msec, results in ml
    }
    sensorData.last_read_sensor = millis();
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
