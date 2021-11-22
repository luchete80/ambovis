#line 2 "MechVentilationTestCases.ino"
#include "AUnit.h"
#include <Arduino.h>
#include "../DefaultTestVars.h"
#include "../../defaults.h"
#include "../../MechVentilation.h"

MechVentilation* ventilation;
VentilationOptions_t options;
MechVentilation* mechV;
byte vent_mode = VENTMODE_MAN;

test(options_setup) {
    Serial.println("Assert options holds the set values");
    assertEqual(options.respiratoryRate, 17);
    assertEqual(options.percInspEsp, 18);
    assertEqual(options.hasTrigger, true);
    assertEqual(options.modeCtl, 19);
    assertEqual(options.peakEspiratoryPressure, 20);
    assertEqual(options.peakInspiratoryPressure, 21);
    assertEqual(options.percVolume, 2);
    assertEqual(options.tidalVolume, 23);
    assertEqual(options.triggerThreshold, 1.5);
}

test(mech_ventilation_start) {
    Serial.println("Assert when mechVentilation starts, running status is true");
    mechV = new MechVentilation(options);
    mechV->start();
    assertEqual(mechV->isRunning(), true);
}

test(mech_ventilation_stop) {
    Serial.println("Assert when mechVentilation stops, running status is false");
    mechV = new MechVentilation(options);
    mechV->stop();
    assertEqual(mechV->isRunning(), false);
}

test(mech_ventilation_set_rpm_updates_inspiratory_cycle) {
    uint8_t rpm = 15;
    Serial.println("Assert that timeoutCycle, _timeoutIns and _timeoutEsp get updated");
    mechV = new MechVentilation(options);

    //Actual call
    mechV->setRPM(rpm);

    assertEqual(rpm, mechV->getRPM());
    //setInspiratoryCycle is called

    float timeoutCycle = ((float)60) * 1000.0f / ((float) rpm); // 4000
    // _percIE = options.percInspEsp = 18
    //_timeoutIns = timeoutCycle / (float(_percIE+1)) = 210.52
    //_timeoutEsp = (timeoutCycle) - _timeoutIns; = 3789.48
    short expectedInsuflationTime = 210;
    short expectedExsuflationTime = 3790;

    assertEqual(mechV->getTimeoutCycle(), timeoutCycle);
    assertEqual(mechV->getInsuflationTime(), expectedInsuflationTime);
    assertEqual(mechV->getExsuflationTime(), expectedExsuflationTime);
}

test(mech_ventilation_update_from_Init_Insufflation_to_State_Insufflation) {
    mechV = new MechVentilation(options);

    //Previous state
    assertEqual(mechV->getState(), State_Homing);

    //actual call
    mechV->update();
    assertEqual(mechV->getState(), Init_Insufflation);

    //second call should be different as state has changed
    mechV->update();
    assertEqual(mechV->getState(), State_Insufflation);
    assertEqual(display_needs_update, true);
    assertEqual(Cdyn, 0);

    //stepperSpeed = STEPPER_HIGHEST_POSITION*(float(_percVol)*0.01)/( (float)(_timeoutIns*0.001) * DEFAULT_FRAC_CYCLE_VCL_INSUFF);
    float roundStepperSpeed = roundf(mechV->getStepperSpeed() * 100) / 100;
    assertEqual(roundStepperSpeed, float(4800.00));
}

test(mech_ventilation_update_from_State_Insufflation_to_State_Exsufflation) {
    mechV = new MechVentilation(options);

    mechV->update();
    mechV->update();

    //Actual call. Time has not expired
    mechV->update();
    assertEqual(mechV->getState(), State_Insufflation);

    //Let the time expire
    delay(1000);
    mechV->update();

    //if MSec > timeIns -> Init_Exsufflation
    assertEqual(mechV->getState(), Init_Exsufflation);

    //next round, it changes the state
    mechV->update();
    assertEqual(mechV->getState(), State_Exsufflation);
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
    options.percVolume = 2;
    options.tidalVolume = 23;
    options.triggerThreshold = 1.5;
}

void loop() {
    aunit::TestRunner::run();
}