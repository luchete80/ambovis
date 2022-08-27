//
// Created by Mirian Quinteros on 28/06/2022.
//

#include "MenuSelector.h"

int getCursorCode(int menu, int i, int ventMode) {
    switch (menu) {
        case MAIN:
            return MAIN_MENU[i];
        case PARAMETER:
            if (ventMode == VENTMODE_MAN) {
                return PARAM_MENU[i];
            } else {
                return PARAM_MENU_PCV[i];
            }
        case ALARM:
            return ALARM_MENU[i];
        case SETTINGS:
            return SETTINGS_MENU[i];
        default:
            return -1;
    }
}

int* getValueToEdit(int code, VariableParameters& parameters) {
    switch (code) {
        case MODE_OPT:
            return &parameters.vent_mode;
        case PERC_V_OPT:
            return &parameters.percVolume;
        case BPM_OPT:
            return &parameters.respiratoryRate;
        case IE_OPT:
            return &parameters.percInspEsp;
        case PIP_OPT:
            return &parameters.peakInspiratoryPressure;
        case PIP_ALARM_OPT:
            return &parameters.alarm_max_pressure;
        case PEEP_ALARM_OPT:
            return &parameters.alarm_peep_pressure;
        case VT_ALARM_OPT:
            return &parameters.alarm_vt;
        case FIL_OPT:
            return &parameters.filter;
        case AUTO_OPT:
            return &parameters.autopid;
    }
}

void resetKeyboardState(KeyboardState& keyboardState) {
    keyboardState.count = 0;
    keyboardState.ok = false;
    keyboardState.back = false;
}

int validate(int val, int min, int max) {
    if (val < min) {
        return min;
    } else if (val > max) {
        return max;
    } else {
        return val;
    }
}

int validate(int val, int cursorCode) {
    switch (cursorCode) {
        case MAIN:
        case PARAMETER:
            return validate(val, 0, 3);
        case ALARM:
            return validate(val, 0, 2);
        case SETTINGS:
            return validate(val, 0, 1);
        case MODE_OPT:
            return validate(val, 1, 2);
        case PERC_V_OPT:
            return validate(val, 40, 100);
        case PIP_OPT:
            return validate(val, 15, 30);
        case BPM_OPT:
            return validate(val, DEFAULT_MIN_RPM, DEFAULT_MAX_RPM);
        case IE_OPT:
            return validate(val, 1, 3);
        case PIP_ALARM_OPT:
            return validate(val, 20, 50);
        case PEEP_ALARM_OPT:
            return validate(val, 5, 30);
        case VT_ALARM_OPT:
            return validate(val, 10, 50);
        case FIL_OPT:
            return validate(val, 0, 1);
        case AUTO_OPT:
            return validate(val, 0, 1);
        default:
            return val;
    }
}

void editParameter(KeyboardState& keyboardState, MenuState& menuState, VariableParameters& variables) {
    if (keyboardState.ok) {
        *getValueToEdit(menuState.cursorCode, variables) = menuState.editedParameter;
        menuState.isEditingParam = false;
        resetKeyboardState(keyboardState);
        keyboardState.count = keyboardState.previousCount;
        menuState.updatedOptions = true;
    } else if (keyboardState.back) {
        menuState.isEditingParam = false;
        resetKeyboardState(keyboardState);
        keyboardState.count = keyboardState.previousCount;
    } else {
        int newValue = menuState.editedParameter + keyboardState.count;
        newValue = validate(newValue, menuState.cursorCode);
        menuState.editedParameter = newValue;
        keyboardState.count = 0;
    }
}

void moveCursor(KeyboardState& keyboardState, MenuState& menuState, VariableParameters& variables, bool isInitialMenu = false) {
    if (keyboardState.ok) {
        if (menuState.cursorCode == END_SETUP) {
            menuState.setupReady = true;
        } else {
            menuState.isEditingParam = true;
            menuState.editedParameter = *getValueToEdit(menuState.cursorCode, variables);
        }
        resetKeyboardState(keyboardState);
    } else if (keyboardState.back && !isInitialMenu) {
        menuState.menu = MAIN;
        menuState.cursorCode = getCursorCode(MAIN, 0, variables.vent_mode);
        menuState.changedMenu = true;
        resetKeyboardState(keyboardState);
    } else {
        keyboardState.count = validate(keyboardState.count, menuState.menu);
        keyboardState.previousCount = keyboardState.count;
        menuState.previousCursorCode = menuState.cursorCode;
        menuState.cursorCode = isInitialMenu ? INIT_PARAM_MENU[keyboardState.count] :
                getCursorCode(menuState.menu, keyboardState.count, variables.vent_mode);
    }
}

void checkKeyboardActionForSetup(KeyboardState& keyboardState, MenuState& menuState, VariableParameters& variables) {
    if (menuState.isEditingParam) {
        editParameter(keyboardState, menuState, variables);
    } else {
        moveCursor(keyboardState, menuState, variables, true);
    }
}

void checkKeyboardActions(KeyboardState& keyboardState, MenuState& menuState, VariableParameters& variables) {
    if (menuState.menu == MAIN) {
        if (keyboardState.ok) {
            menuState.menu = menuState.cursorCode;
            menuState.changedMenu = true;
            menuState.previousCursorCode = menuState.cursorCode;
            menuState.cursorCode = getCursorCode(menuState.menu, 0, variables.vent_mode);
            resetKeyboardState(keyboardState);
        } else {
            keyboardState.count = validate(keyboardState.count, MAIN);
            menuState.previousCursorCode = menuState.cursorCode;
            menuState.cursorCode = getCursorCode(MAIN, keyboardState.count, variables.vent_mode);
        }
    } else {
        if (menuState.isEditingParam) {
            editParameter(keyboardState, menuState, variables);
        } else {
            moveCursor(keyboardState, menuState, variables, false);
        }
    }
}
