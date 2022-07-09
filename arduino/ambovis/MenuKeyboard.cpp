//
// Created by Mirian Quinteros on 28/06/2022.
//
#include "MenuKeyboard.h"

void checkUPButtonPressed(KeyboardState & keyboardState, unsigned long time) {
    if (digitalRead(PIN_MENU_UP) == LOW) {
        int timeDiff = time - keyboardState.lastKeyPressedTime;
        if (timeDiff > 500) {
            Serial.print("Up pressed ");Serial.println(timeDiff);
            keyboardState.count = keyboardState.count + 1;
            keyboardState.lastKeyPressedTime = time;
        }
    }
}

void checkDOWNButtonPressed(KeyboardState& keyboardState, unsigned long time) {
    if (digitalRead(PIN_MENU_DN) == LOW) {
        int timeDiff = time - keyboardState.lastKeyPressedTime;
        if (timeDiff > 500) {
            Serial.print("Down pressed ");;Serial.println(timeDiff);
            keyboardState.count = keyboardState.count - 1;
            keyboardState.lastKeyPressedTime = time;
        }
    }
}

void checkOKButtonPressed(KeyboardState& keyboardState, unsigned long time) {
    if (digitalRead(PIN_MENU_EN) == LOW) {
        int timeDiff = time - keyboardState.lastKeyPressedTime;
        if (timeDiff > 700) {
            Serial.print("OK pressed ");;Serial.println(timeDiff);
            keyboardState.ok = true;
            keyboardState.lastKeyPressedTime = time;
        }
    }
}

void checkBackButtonPressed(KeyboardState& keyboardState, unsigned long time) {
    if (digitalRead(PIN_MENU_BCK) == LOW) {
        if (time - keyboardState.lastKeyPressedTime > 700) {
            keyboardState.back = true;
            keyboardState.lastKeyPressedTime = time;
        }
    }
}

void checkKeyboard(KeyboardState & keyboardState, unsigned long time) {
    checkUPButtonPressed(keyboardState, time);
    checkDOWNButtonPressed(keyboardState, time);
    checkOKButtonPressed(keyboardState, time);
    checkBackButtonPressed(keyboardState, time);
}