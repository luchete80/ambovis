#line 2 "MenuTestCases.ino"
#include "AUnit.h"
#include <Arduino.h>
#include "../DefaultTestVars.h"
#include "../../menu.h"
#include "../../MechVentilation.h"
#define LED LED_BUILTIN
#define LED_ON HIGH
#define LED_OFF LOW

MechVentilation * ventilation;
VentilationOptions_t options;
byte vent_mode = VENTMODE_MAN;

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

    assertEqual(last_bck_state, LOW);
    assertFalse(sleep_mode);
    assertTrue(wake_up);
}

//test(check_back_state_when_button_is_still_pressed_when_sleep_mode) {
//    last_bck_state = 2;
//    check_bck_state(); // change state
//
//    time = 5000;
//    last_bck_state = 0;
//    startPressed = 0;
//    sleep_mode = true;
//    //as we did not mock the read value, it will always be 0 (LOW)
//    check_bck_state();
//
//    Serial.println(holdTime);
//    assertEqual(last_bck_state, LOW);
//    assertFalse(sleep_mode);
//    assertTrue(wake_up);
//}

void setup() {
    delay(1000); // wait for stability on some boards to prevent garbage Serial
    Serial.begin(115200); // ESP8266 default of 74880 not supported on Linux
    while(!Serial); // for the Arduino Leonardo/Micro only
}

void loop() {
    aunit::TestRunner::run();
}