#ifndef _MENU_KEYBOARD_H_
#define _MENU_KEYBOARD_H_

#include "Arduino.h"
#include "defaults.h"
#include "pinout.h"
#include "MenuDataTypes.h"

void checkUPButtonPressed(KeyboardState & keyboardState, unsigned long time) {
    if (digitalRead(PIN_MENU_UP) == AMBOVIS_LOW) {
        if (time - keyboardState.lastKeyPressedTime > 150) {
            keyboardState.count = keyboardState.count + 1;
            keyboardState.lastKeyPressedTime = time;
        }
    }
}

void checkDOWNButtonPressed(KeyboardState& keyboardState, unsigned long time) {
    if (digitalRead(PIN_MENU_DN) == AMBOVIS_LOW) {
        if (time - keyboardState.lastKeyPressedTime > 150) {
            keyboardState.count = keyboardState.count - 1;
            keyboardState.lastKeyPressedTime = time;
        }
    }
}

void checkOKButtonPressed(KeyboardState& keyboardState, unsigned long time) {
    if (digitalRead(PIN_MENU_EN) == AMBOVIS_LOW) {
        if (time - keyboardState.lastKeyPressedTime > 50) {
            keyboardState.ok = true;
            keyboardState.lastKeyPressedTime = time;
        }
    }
}

void checkBackButtonPressed(KeyboardState& keyboardState, unsigned long time) {
    if (digitalRead(PIN_MENU_BCK) == AMBOVIS_LOW) {
        if (time - keyboardState.lastKeyPressedTime > 50) {
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

#endif
