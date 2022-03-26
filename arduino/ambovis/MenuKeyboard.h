#ifndef _MENU_KEYBOARD_H_
#define _MENU_KEYBOARD_H_

#include "Arduino.h"
#include "defaults.h"
#include "pinout.h"

typedef struct {
    unsigned long lastKeyPressedTime = 0;
    byte value;
    bool itemSelected = false;
} MenuKeyboard;

void checkUPButtonPressed(MenuKeyboard& menuKeyboard, unsigned long time) {
    if (digitalRead(PIN_MENU_UP) == AMBOVIS_LOW) {
        if (time - menuKeyboard.lastKeyPressedTime > 150) {
            menuKeyboard.value = menuKeyboard.value + 1;
            menuKeyboard.lastKeyPressedTime = time;
        }
    }
}

void checkDOWNButtonPressed(MenuKeyboard& menuKeyboard, unsigned long time) {
    if (digitalRead(PIN_MENU_DN) == AMBOVIS_LOW) {
        if (time - menuKeyboard.lastKeyPressedTime > 150) {
            menuKeyboard.value = menuKeyboard.value - 1;
            menuKeyboard.lastKeyPressedTime = time;
        }
    }
}

void checkOKButtonPressed(MenuKeyboard& menuKeyboard, unsigned long time) {
    if (digitalRead(PIN_MENU_EN) == AMBOVIS_LOW) {
        if (time - menuKeyboard.lastKeyPressedTime > 50) {
            menuKeyboard.itemSelected = true;
            menuKeyboard.lastKeyPressedTime = time;
        }
    }
}

void checkBackButtonPressed(MenuKeyboard& menuKeyboard, unsigned long time) {
    if (digitalRead(PIN_MENU_BCK) == AMBOVIS_LOW) {
        if (time - menuKeyboard.lastKeyPressedTime > 50) {
            menuKeyboard.itemSelected = false;
            menuKeyboard.lastKeyPressedTime = time;
        }
    }
}

//void updateState() {
//    // the button has been just pressed
//    if (bck_state == LOW) {
//        startPressed = time;
//        idleTime = startPressed - endPressed;
//        change_sleep=false;
//      // the button has been just released
//    } else {
//        endPressed = time;
//        holdTime = endPressed - startPressed;
//    }
//}

//void updateCounter() {
//    // the button is still pressed
//    if (bck_state == LOW) {
//      holdTime = time - startPressed;
//      // the button is still released
//    } else {
//      idleTime = time - endPressed;
//    }
//}

//void checkBackButtonPressed(MenuSelector& menuSelector, unsigned long time) {
//    int bck_state = digitalRead(PIN_MENU_BCK);
//    if (bck_state != last_bck_state) {
//        updateState(); // button state changed. It runs only once.
//        if ( bck_state == LOW ) { //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6)
//            if (time - lastButtonPress > 150) {
//                pressed = 2;
//                isitem_sel=false;
//                lastButtonPress = time;
//            }// if time > last button press
//        }
//    } else {
//        updateCounter(); // button state not changed. It runs in a loop.
//        if (holdTime > 2000 && !change_sleep) {
//            if (!systemState.sleep_mode){
//                systemState.sleep_mode=true;
//                systemState.put_to_sleep=true;
//            } else {
//                systemState.sleep_mode=false;
//                systemState.wake_up=true;
//            }
//            change_sleep=true;
//        }
//    }
//    last_bck_state = bck_state;
//}

#endif
