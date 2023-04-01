#include <AUnit.h>
#include "../../pinout.h"
#include "../../menu.h"
#include "../../MechanicalVentilation.h"

byte cycle_pos;
float pressure_p;
float _mlInsVol,_mlExsVol;
int _mllastInsVol,_mllastExsVol;
bool ended_whilemov;
float vlevel;
short alarm_state;
bool drawing_cycle;
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);


test(correct) {
    int x = 1;
    assertEqual(x, 1);
}

test(incorrect) {
    int x = 2;
    assertNotEqual(x, 1);
}

void setup() {
    delay(1000); // wait for stability on some boards to prevent garbage Serial
    Serial.begin(115200); // ESP8266 default of 74880 not supported on Linux
    while(!Serial); // for the Arduino Leonardo/Micro only

    Serial.println(F("This test should produce the following: arr"));
    Serial.println(
            F("1 passed, 1 failed, 0 skipped, 0 timed out, out of 2 test(s).")
            );
    Serial.println(F("----"));
}

void loop() {
    aunit::TestRunner::run();
}