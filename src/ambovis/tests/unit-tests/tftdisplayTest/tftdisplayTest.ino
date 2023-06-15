#include <AUnit.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "../../../display.h"
#include "../../../MechanicalVentilation.h"

Ventilation_Status_t status;
AlarmData alarm_data;
SensorData sensorData;
bool drawing_cycle = 0;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

test(change_draw_cycle) {
    status.cycle_pos=118;
    sensorData.pressure_p = 1.3;
    sensorData.flow_f = 23.5;
    sensorData.v_level = 2.2;
    tft_draw(tft, sensorData, status, drawing_cycle, alarm_data);
    assertEqual(true, drawing_cycle);
}

void setup() {
    delay(1000); // wait for stability on some boards to prevent garbage Serial
    Serial.begin(115200); // ESP8266 default of 74880 not supported on Linux
    while(!Serial);
}

void loop() {
    aunit::TestRunner::run();
}


Buzzer_State_t buzzer;
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
bool put_to_sleep;
bool sleep_mode;
unsigned long time2;
bool wake_up;
EpoxyEepromAvr EEPROM;