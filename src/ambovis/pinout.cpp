//
// Created by Mirian Quinteros on 24/12/2022.
//
#include "pinout.h"

void init_pins() {
    pinMode(PIN_STEPPER, OUTPUT);
    digitalWrite(PIN_STEPPER, LOW);

    pinMode(TFT_SLEEP, OUTPUT);

    pinMode(LCD_SLEEP, OUTPUT);

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, BUZZER_LOW);

    pinMode(GREEN_LED, OUTPUT);
    pinMode(BCK_LED, OUTPUT);
    digitalWrite(BCK_LED, LOW);
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