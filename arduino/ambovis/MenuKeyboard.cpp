//
// Created by Mirian Quinteros on 28/06/2022.
//
#include "MenuKeyboard.h"

void checkUPButtonPressed(KeyboardState & keyboardState, unsigned long time) {
    int currentState = digitalRead(PIN_MENU_UP);
    unsigned long timeDiff = time - keyboardState.lastKeyPressedTime;
    if (currentState == 0 && timeDiff > 150) {
        keyboardState.count = keyboardState.count - 1;
        keyboardState.lastKeyPressedTime = time;
        Serial.println("Key UP");
    }
}

void checkDOWNButtonPressed(KeyboardState& keyboardState, unsigned long time) {
    int currentState = digitalRead(PIN_MENU_DN);
    unsigned long timeDiff = time - keyboardState.lastKeyPressedTime;
    if (currentState == 0 && timeDiff > 150) {
        keyboardState.count = keyboardState.count + 1;
        keyboardState.lastKeyPressedTime = time;
        Serial.println("Key DOWN");
    }
}

void checkOKButtonPressed(KeyboardState& keyboardState, unsigned long time) {
    int currentState = digitalRead(PIN_MENU_EN);
    unsigned long timeDiff = time - keyboardState.lastKeyPressedTime;
    if (currentState == 0 && timeDiff > 100) {
        keyboardState.ok = true;
        keyboardState.lastKeyPressedTime = time;
        Serial.println("Key OK");
    }
}

void updateState(KeyboardState& keyboardState, int currentState, long time) {
    if (currentState == LOW) {
        keyboardState.backHoldTime = 0;
    } else {
        keyboardState.backHoldTime = time - keyboardState.backPressStart;
    }
}

void checkBackButtonPressed(KeyboardState& keyboardState, unsigned long time) {
    int currentState = digitalRead(PIN_MENU_BCK);
    unsigned long timeDiff = time - keyboardState.lastKeyPressedTime;
    if (currentState != keyboardState.lastBackState) {
        updateState(keyboardState, currentState, time);
        if (currentState == 0 && timeDiff > 150) {
            keyboardState.back = true;
            keyboardState.lastKeyPressedTime = time;
            keyboardState.backHoldTime = 200;
            Serial.println("Key BACK");
        }
    } else {
         if (currentState == 0) {
            keyboardState.backHoldTime += 100;
         }
    }
    keyboardState.lastBackState = currentState;
}

void checkKeyboard(KeyboardState & keyboardState, unsigned long time) {
    checkUPButtonPressed(keyboardState, millis());
    checkDOWNButtonPressed(keyboardState, millis());
    checkOKButtonPressed(keyboardState, millis());
    checkBackButtonPressed(keyboardState, millis());
}