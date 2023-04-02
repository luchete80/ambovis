#include <AUnit.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "../../../defaults.h"
#include "../../../alarms.h"

byte alarm_max_pressure;
byte alarm_peep_pressure;
short alarm_state;
byte alarm_vt;
bool autopid;
int bck_state;
int curr_sel;
float dpip;
byte dpip_b;
byte encoderPos;
int endPressed;
float f_acc;
byte f_acc_b;
bool filter;
int holdTime;
int idleTime;
bool is_alarm_vt_on;
bool isitem_sel;
unsigned long lastButtonPress;
int last_bck_state;
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
int max_accel, max_cd, max_pidd, max_pidi, max_pidk;
byte max_sel;
int max_speed;
byte menu_number;
int min_accel, min_pidd, min_pidi, min_pidk;
byte min_sel;
byte oldEncPos;
int old_curr_sel;
byte p_acc, p_trim;
float peep_fac, pf_max, pf_min;
byte pfmax, pfmin;
bool put_to_sleep;
bool show_changed_options;
bool sleep_mode;
int startPressed;
char tempstr[5];
unsigned long time2;
bool update_options;
bool wake_up;
EpoxyEepromAvr EEPROM;

test(get_alarm_state_when_no_alarm) {
    short alarm = get_alarm_state(100., 80., 150, 50);
    assertEqual(alarm, NO_ALARM);
}

test(get_alarm_state_when_min_pressure_is_below) {
    short alarm = get_alarm_state(100., 30., 150, 50);
    assertEqual(alarm, PEEP_ALARM);
}

test(get_alarm_state_when_max_pressure_is_above) {
    short alarm = get_alarm_state(200., 80., 150, 50);
    assertEqual(alarm, PIP_ALARM);
}

test(get_alarm_state_when_limits_exceed) {
    short alarm = get_alarm_state(200., 10., 150, 50);
    assertEqual(alarm, PEEP_PIP_ALARM);
}

test(alarm_vt_should_be_on) {
    bool is_alarm = calc_alarm_vt_is_on(10, 30, 15);
    assertEqual(is_alarm, false);
}

test(alarm_vt_should_be_off) {
    bool is_alarm = calc_alarm_vt_is_on(10, 15, 15);
    assertEqual(is_alarm, true);
}

test(check_buzzer_should_not_be_muted) {
    //prepare
    digitalReadValue(PIN_MUTE, LOW);
    unsigned long last_count = millis();
    bool last_mute = HIGH;
    // actual call
    bool is_buzz_muted = check_buzzer_mute(last_mute, true, last_count, last_count + 80000);
    assertEqual(is_buzz_muted, false);
}

test(check_buzzer_when_is_not_update_time_should_stay_muted) {
    //prepare
    digitalReadValue(PIN_MUTE, LOW);
    unsigned long last_count = millis();
    bool last_mute = HIGH;

    //actual call
    bool is_buzz_muted = check_buzzer_mute(last_mute, true, last_count, last_count + 2000);
    assertEqual(is_buzz_muted, true);
}

test(check_buzzer_should_go_muted_and_last_mute_updated) {
    //prepare
    digitalReadValue(PIN_MUTE, LOW);
    unsigned long last_count = millis();
    unsigned long now = last_count + 2000;
    bool last_mute = HIGH;

    //actual call
    bool is_buzz_muted = check_buzzer_mute(last_mute, false, last_count, now);
    assertEqual(is_buzz_muted, true);
    assertEqual(last_mute, false); // LOW == false
    assertEqual(last_count, now);
}

void setup() {
    delay(1000); // wait for stability on some boards to prevent garbage Serial
    Serial.begin(115200); // ESP8266 default of 74880 not supported on Linux
    while(!Serial);
}

void loop() {
    aunit::TestRunner::run();
}