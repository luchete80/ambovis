//
// Created by Mirian Quinteros on 28/06/2022.
//
#include "MenuKeyboard.h"

void checkUPButtonPressed(KeyboardState & keyboardState, unsigned long time) {
    int currentState = digitalRead(PIN_MENU_UP);
    if (currentState != keyboardState.lastUpState) {
        if (currentState == 0) {
            keyboardState.count = keyboardState.count + 1;
            keyboardState.lastKeyPressedTime = time;
        }
        keyboardState.lastUpState = currentState;
    }
}

void checkDOWNButtonPressed(KeyboardState& keyboardState, unsigned long time) {
    int currentState = digitalRead(PIN_MENU_DN);
    if (currentState != keyboardState.lastDownState) {
        if (currentState == 0) {
            keyboardState.count = keyboardState.count - 1;
            keyboardState.lastKeyPressedTime = time;
        }
        keyboardState.lastDownState = currentState;
    }
}

void checkOKButtonPressed(KeyboardState& keyboardState, unsigned long time) {
    int currentState = digitalRead(PIN_MENU_EN);
    if (currentState != keyboardState.lastOKState) {
        if (currentState == 0) {
            keyboardState.ok = true;
            keyboardState.lastKeyPressedTime = time;
        }
        keyboardState.lastOKState = currentState;
    }
}

void checkBackButtonPressed(KeyboardState& keyboardState, unsigned long time) {
    int currentState = digitalRead(PIN_MENU_BCK);
    if (currentState != keyboardState.lastBackState) {
        keyboardState.backHoldTime = 0;
        if (currentState == 0) {
            keyboardState.back = true;
            keyboardState.lastKeyPressedTime = time;
            keyboardState.backHoldTime = 200;
        }
        keyboardState.lastBackState = currentState;
    }
//    else {
//        if (currentState == 0) {
//            keyboardState.backHoldTime += 100;
//        }
//    }
}

void checkKeyboard(KeyboardState & keyboardState, unsigned long time) {
    checkUPButtonPressed(keyboardState, time);
    checkDOWNButtonPressed(keyboardState, time);
    checkOKButtonPressed(keyboardState, time);
    checkBackButtonPressed(keyboardState, time);
}