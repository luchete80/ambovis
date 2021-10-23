#line 2 "MechVentilationTestCases.ino"
#include "AUnit.h"
#include <Arduino.h>
#include "../../defaults.h"
#include "../../MechVentilation.h"

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
MechVentilation* ventilation;

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
int min_cd,max_cd;
byte vent_mode = VENTMODE_MAN;
unsigned long time;

VentilationOptions_t options;
MechVentilation* mechV;

test(optionsSet) {
    Serial.println("Assert options holds the set values");
    assertEqual(options.respiratoryRate, 17);
    assertEqual(options.percInspEsp, 18);
    assertEqual(options.hasTrigger, true);
    assertEqual(options.modeCtl, 19);
    assertEqual(options.peakEspiratoryPressure, 20);
    assertEqual(options.peakInspiratoryPressure, 21);
    assertEqual(options.percVolume, 22);
    assertEqual(options.tidalVolume, 23);
    assertEqual(options.triggerThreshold, 1.5);
}

test(mechVentilationStart) {
    Serial.println("Assert when mechVentilation starts, running status is true");
    mechV = new MechVentilation(options);
    mechV->start();
    assertEqual(mechV->isRunning(), true);
}

test(mechVentilationStop) {
    Serial.println("Assert when mechVentilation stops, running status is false");
    mechV = new MechVentilation(options);
    mechV->stop();
    assertEqual(mechV->isRunning(), false);
}

test(mechVentilationSetInspiratoryCycle) {
    Serial.println("Assert that timeoutCycle, _timeoutIns and _timeoutEsp get updated");
    //TODO: need to add getter for timeoutCycle to test
    mechV = new MechVentilation(options);
    mechV->_setInspiratoryCycle();

    //_rpm  = options.respiratoryRate = 17
    // timeoutCycle = ((float)60) * 1000.0f / ((float)_rpm) = 3529.4
    // _percIE = options.percInspEsp = 18
    //_timeoutIns = timeoutCycle / (float(_percIE+1)) = 185
    // _timeoutEsp = (timeoutCycle) - _timeoutIns; = 3344
    short expectedInsuflationTime = 185;
    short expectedExsuflationTime = 3344;
    assertEqual(mechV->getInsuflationTime(), expectedInsuflationTime);
    assertEqual(mechV->getExsuflationTime(), expectedExsuflationTime);
}

test(mechVentilationUpdate) {
    Serial.println("Assert _msecTimerCnt cycle_pos _currentState");
    mechV = new MechVentilation(options);

    State expected = State_Homing;
    assertEqual(mechV->getState(), expected);

    mechV->update();
    expected = Init_Insufflation;
    assertEqual(mechV->getState(), expected);

    mechV->update();
    expected = State_Insufflation;
    assertEqual(mechV->getState(), expected);
    assertEqual(display_needs_update, true);
}

void setup() {
    delay(1000); // wait for stability on some boards to prevent garbage Serial
    Serial.begin(115200); // ESP8266 default of 74880 not supported on Linux
    while(!Serial); // for the Arduino Leonardo/Micro only

    options.respiratoryRate = 17;
    options.percInspEsp = 18;
    options.hasTrigger = true;
    options.modeCtl = 19;
    options.peakEspiratoryPressure = 20;
    options.peakInspiratoryPressure = 21;
    options.percVolume = 22;
    options.tidalVolume = 23;
    options.triggerThreshold = 1.5;
}

void loop() {
    aunit::TestRunner::run();
}