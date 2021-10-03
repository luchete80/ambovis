#line 2 "MechVentilationTestCases.ino"
#define FOR_TEST 1
#include "AUnit.h"
#include <Arduino.h>
#include "../../../menu.h"

test(menuBeforeStaring) {

    assertEqual(eh, true);
}

void setup() {
    delay(1000); // wait for stability on some boards to prevent garbage Serial
    Serial.begin(115200); // ESP8266 default of 74880 not supported on Linux
    while(!Serial); // for the Arduino Leonardo/Micro only
}

void loop() {
    aunit::TestRunner::run();
}