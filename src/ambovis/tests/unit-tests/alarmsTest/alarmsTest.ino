#include <AUnit.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "../../../defaults.h"
#include "../../../alarms.h"
#include "../../../MechanicalVentilation.h"

Ventilation_Status_t status;
AlarmData alarm_data;
Buzzer_State_t buzzer;

test(check_alarms_when_no_alarms) {
    alarm_data.alarm_max_pressure = 20;
    alarm_data.alarm_peep_pressure = 5;
    status.last_max_pressure = 15;
    status.last_min_pressure = 8;

    status.ml_last_ins_vol = 15;
    status.ml_last_exp_vol = 8;
    alarm_data.alarm_vt = 10; // (23 / 2) < 10

    alarm_data = check_alarms(status, alarm_data);

    assertEqual(alarm_data.alarm_state, NO_ALARM);
    assertEqual(alarm_data.is_alarm_vt_on, false);
}

test(get_alarm_state_when_min_pressure_is_below) {
    alarm_data.alarm_max_pressure = 20;
    alarm_data.alarm_peep_pressure = 5;
    status.last_max_pressure = 15;
    status.last_min_pressure = 2;

    alarm_data = check_alarms(status, alarm_data);
    assertEqual(alarm_data.alarm_state, PEEP_ALARM);
}

test(get_alarm_state_when_max_pressure_is_above) {
    alarm_data.alarm_max_pressure = 20;
    alarm_data.alarm_peep_pressure = 5;
    status.last_max_pressure = 30;
    status.last_min_pressure = 8;

    alarm_data = check_alarms(status, alarm_data);
    assertEqual(alarm_data.alarm_state, PIP_ALARM);
}

test(get_alarm_state_when_both_limits_exceed) {
    alarm_data.alarm_max_pressure = 20;
    alarm_data.alarm_peep_pressure = 5;
    status.last_max_pressure = 30;
    status.last_min_pressure = 1;

    alarm_data = check_alarms(status, alarm_data);
    assertEqual(alarm_data.alarm_state, PEEP_PIP_ALARM);
}

test(check_alarm_vt_is_on) {
    status.ml_last_ins_vol = 4;
    status.ml_last_exp_vol = 2;
    alarm_data.alarm_vt = 10;

    alarm_data = check_alarms(status, alarm_data);

    assertEqual(alarm_data.is_alarm_vt_on, true);
}

test(check_buzzer_should_not_be_muted) {
    //prepare
    digitalReadValue(PIN_MUTE, LOW);
    buzzer.mute_count = millis();
    buzzer.last_mute = HIGH;
    buzzer.buzz_muted = true;

    // actual call
    buzzer = check_buzzer_mute(buzzer, buzzer.mute_count + 80000);
    assertEqual(buzzer.buzz_muted, false);
}

test(check_buzzer_when_is_not_update_time_should_stay_muted) {
    //prepare
    digitalReadValue(PIN_MUTE, LOW);
    buzzer.mute_count = millis();
    buzzer.last_mute = HIGH;
    buzzer.buzz_muted = true;

    //actual call
    buzzer = check_buzzer_mute(buzzer, buzzer.mute_count + 2000);
    assertEqual(buzzer.buzz_muted, true);
}

test(check_buzzer_should_go_muted_and_last_mute_updated) {
    //prepare
    digitalReadValue(PIN_MUTE, LOW);
    buzzer.mute_count = millis();
    unsigned long now = buzzer.mute_count + 2000;
    buzzer.last_mute = HIGH;
    buzzer.buzz_muted = false;

    //actual call
    buzzer = check_buzzer_mute(buzzer, now);
    assertEqual(buzzer.buzz_muted, true);
    assertEqual(buzzer.last_mute, false); // LOW == false
    assertEqual(buzzer.mute_count, now);
}

void setup() {
    delay(1000); // wait for stability on some boards to prevent garbage Serial
    Serial.begin(115200); // ESP8266 default of 74880 not supported on Linux
    while(!Serial);
}

void loop() {
    aunit::TestRunner::run();
}

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
bool put_to_sleep;
bool show_changed_options;
bool sleep_mode;
unsigned long time2;
bool update_options;
bool wake_up;
EpoxyEepromAvr EEPROM;