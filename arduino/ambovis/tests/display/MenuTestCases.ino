#line 2 "MenuTestCases.ino"
#include "AUnit.h"
#include <Arduino.h>
#include "../../menu.h"
#include "../../MechVentilation.h"
#define LED LED_BUILTIN
#define LED_ON HIGH
#define LED_OFF LOW

byte max_sel,min_sel; //According to current selection
unsigned long lastButtonPress;
int bck_state ;     // current state of the button
int last_bck_state; // previous state of the button
int startPressed ;    // the moment the button was pressed
int endPressed ;      // the moment the button was released
int holdTime ;        // how long the button was hold
int idleTime ;        // how long the button was idle

int curr_sel, old_curr_sel;
byte encoderPos; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
byte oldEncPos; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
byte old_menu_pos,old_menu_num;
bool show_changed_options; //Only for display
bool update_options;
char tempstr[5],tempstr2[5];
byte menu_number;
byte p_trim;

byte Cdyn;
float _mlInsVol,_mlExsVol;
int _mllastInsVol,_mllastExsVol;
unsigned int _timeoutIns;
unsigned int _timeoutEsp;
byte cycle_pos; //0 to 127
bool display_needs_update;
float last_pressure_max,last_pressure_min,last_pressure_peep;
MechVentilation * ventilation;

byte alarm_max_pressure,alarm_peep_pressure;
int alarm_vt;
bool autopid;
bool change_pid_params;
bool filter;
bool isitem_sel;
unsigned long last_cycle;
int max_accel,min_accel;
int max_speed, min_speed;
int min_pidk,max_pidk;
int min_pidi,max_pidi;
int min_pidd,max_pidd;
byte pfmin,pfmax;
float pf_min,pf_max;
float peep_fac;
bool sleep_mode;
bool put_to_sleep,wake_up;
unsigned long print_bat_time;
VentilationOptions_t options;
int min_cd,max_cd;
byte vent_mode = VENTMODE_MAN;
unsigned long time;

test(check_back_state_when_button_is_changed) {
    last_bck_state = 3;
    time = 300;
    lastButtonPress = 0;
    //as we did not mock the read value, it will always be 0 (LOW)
    Serial.println(last_bck_state);
    Serial.println(time);
    Serial.println(lastButtonPress);

    check_bck_state();
    Serial.println(lastButtonPress);

    assertEqual(lastButtonPress, time); // should be the same as time
    assertFalse(isitem_sel);
    assertEqual(last_bck_state, LOW);
}

test(check_back_state_when_button_is_still_pressed) {
    time = 5000;
    last_bck_state = 0;
    startPressed = 0;
    sleep_mode = false;
    //as we did not mock the read value, it will always be 0 (LOW)
    Serial.println(last_bck_state);
    Serial.println(time);
    Serial.println(lastButtonPress);

    check_bck_state();

    Serial.println(holdTime);
    assertEqual(last_bck_state, LOW);
    assertTrue(sleep_mode);
    assertTrue(put_to_sleep);
}

test(check_back_state_when_button_is_still_pressed_when_sleep_mode) {
    last_bck_state = 2;
    check_bck_state(); // change state

    time = 5000;
    last_bck_state = 0;
    startPressed = 0;
    sleep_mode = true;
    //as we did not mock the read value, it will always be 0 (LOW)
    check_bck_state();

    Serial.println(holdTime);
    assertEqual(last_bck_state, LOW);
    assertFalse(sleep_mode);
    assertTrue(wake_up);
}

void setup() {
    delay(1000); // wait for stability on some boards to prevent garbage Serial
    Serial.begin(115200); // ESP8266 default of 74880 not supported on Linux
    while(!Serial); // for the Arduino Leonardo/Micro only
}

void loop() {
    aunit::TestRunner::run();
}