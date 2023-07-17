#include <AUnit.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "../../../alarms.h"
#include "../../../sensorcalculation.h"


SensorData sensorData;
static float test_dp[] = { 0.1, 0.2, 0.3, 0.4, 0.5 };
static unsigned char test_po_flux[] = { 1, 2, 3, 5, 7 };

test(find_flux) {
    int pos = 3;
    float expected = test_po_flux[pos] - 100 + ( float (test_po_flux[pos + 1] - 100) - float (test_po_flux[pos] - 100) ) * ( 0.38 - float(test_dp[pos]) ) / (float)( test_dp[pos + 1] - test_dp[pos]);
    expected = expected * 16.6667;

    float x = find_flux(0.38, test_dp, test_po_flux, 5);
    assertEqual(x, expected);
}

test(get_dpt) {
    float expected = (((5.0 - 3.0) / 0.5) - 0.04) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20;
    float x = get_dpt(5., 0.5, 3.);
    assertEqual(x, expected);
}

test(convert_sensor_data) {
    float expected_pressure = (2 / 1023. - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20;
    float expected_v_level = float(3) / 1024. * 1.1 * VOLTAGE_CONV;
    float expected_voltage = float(1) * 0.1875 * 0.001;
    convert_sensor_data(1, 2, 3, sensorData);
    assertEqual(sensorData.pressure_p, expected_pressure);
    assertEqual(sensorData.v_level, expected_v_level);
    assertEqual(sensorData.voltage, expected_voltage);
}

test(get_flow) {
    SensorData sensor1;
    sensor1.flux = 10.;
    sensor1.flux_filter[0] = 1.;
    sensor1.flux_filter[1] = 2.;
    sensor1.flux_filter[2] = 3.;
    sensor1.flux_filter[3] = 4.;
    sensor1.flux_filter[4] = 5.;
    float expected = (24.) / 5.;
    float flow = get_flow(sensor1);
    assertEqual(flow, expected);
}

test(update_vol_when_flux_is_negative) {
    SensorData sensor1;
    sensor1.last_read_sensor = millis();
    sensor1.ml_exs_vol = 10.;
    sensor1.flux = -10.;
    sensor1.flow_f = 3.;
    float expected = sensor1.ml_exs_vol - sensor1.flow_f * 2000. * 0.001;
    update_vol(sensor1, sensor1.last_read_sensor + 2000);
    assertEqual(sensor1.ml_exs_vol, expected);
}

test(update_vol_when_flux_is_positive) {
    SensorData sensor1;
    sensor1.last_read_sensor = millis();
    sensor1.ml_ins_vol = 10.;
    sensor1.flux = 10.;
    sensor1.flow_f = 3.;
    float expected = sensor1.ml_ins_vol + sensor1.flow_f * 2000. * 0.001;
    update_vol(sensor1, sensor1.last_read_sensor + 2000);
    assertEqual(sensor1.ml_ins_vol, expected);
}

test(eval_max_min_pressure) {
    SensorData sensor1;
    sensor1.pressure_min = 10.;
    sensor1.pressure_max = 100.;
    sensor1.pressure_p = 130.;
    eval_max_min_pressure(sensor1);
    assertEqual(sensor1.pressure_max, 130.);
    assertEqual(sensor1.pressure_min, 10.);
}

void setup() {
    delay(1000); // wait for stability on some boards to prevent garbage Serial
    Serial.begin(115200); // ESP8266 default of 74880 not supported on Linux
    while(!Serial);
}

void loop() {
    aunit::TestRunner::run();
}

AlarmData alarm_data;
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
bool put_to_sleep;
bool sleep_mode;
unsigned long time2;
bool wake_up;
EpoxyEepromAvr EEPROM;