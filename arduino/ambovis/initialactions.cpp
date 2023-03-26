//
// Created by Mirian Quinteros on 02/09/2022.
//

#include "initialactions.h"

void waitForFluxDisconnected() {
    bool fin = false;
    delay(100); //Otherwise low enter button readed
    long lastButtonPress = millis();
    while (!fin) {
        if (digitalRead(PIN_MENU_EN) == LOW) {
            if (millis() - lastButtonPress > 50) {
                fin = true;
                lastButtonPress = millis();
            }
        }
    }
}

void initPins() {
    pinMode(PIN_STEPPER, OUTPUT);
    digitalWrite(PIN_STEPPER, LOW);

    pinMode(TFT_SLEEP, OUTPUT);
    digitalWrite(TFT_SLEEP, HIGH);

    pinMode(LCD_SLEEP, OUTPUT);
    digitalWrite(LCD_SLEEP, HIGH);

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, BUZZER_LOW);

    pinMode(GREEN_LED,  OUTPUT);
    pinMode(BCK_LED,    OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);

    pinMode(PIN_MUTE, INPUT_PULLUP);
    pinMode(PIN_POWEROFF, INPUT);

    pinMode(PIN_EN, OUTPUT);
    digitalWrite(PIN_EN, HIGH);

    pinMode(PIN_MENU_UP, INPUT_PULLUP);
    pinMode(PIN_MENU_DN, INPUT_PULLUP);
    pinMode(PIN_MENU_EN, INPUT_PULLUP);
    pinMode(PIN_MENU_BCK, INPUT_PULLUP);

    pinMode(PIN_BAT_LEV, INPUT);
    pinMode(PIN_MPX_LEV, INPUT);

    pinMode(PIN_ENC_SW, INPUT_PULLUP);
}