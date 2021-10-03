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
int last_bck_state ; // previous state of the button
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


test(menuBeforeStaring) {
    digitalWrite(LED, LED_ON);

}

void setup() {
    delay(1000); // wait for stability on some boards to prevent garbage Serial
    Serial.begin(115200); // ESP8266 default of 74880 not supported on Linux
    while(!Serial); // for the Arduino Leonardo/Micro only
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LED_OFF);
}

void loop() {
    aunit::TestRunner::run();
}