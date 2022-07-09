//
// Created by Mirian Quinteros on 28/06/2022.
//

#include "MenuSelector.h"

int getCursorCode(int menu, int i) {
    switch (menu) {
        case MAIN:
            return MAIN_MENU[i];
        case PARAMETER:
            return PARAM_MENU[i];
        case ALARM:
            return ALARM_MENU[i];
        case SETTINGS:
            return SETTINGS_MENU[i];
        case PID_SETTINGS:
            return PID_SETTINGS_MENU[i];
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
        case TRIM_OPT:
            return &parameters.p_trim;
        case FIL_OPT:
            return &parameters.fil;
        case AUTO_OPT:
            return &parameters.autopid;
        case CD_OPT:
            return &parameters.cd_opt;
        case DP_OPT:
            return &parameters.dp_opt;
        case F_OPT:
            return &parameters.f_min;
        case FF_OPT:
            return &parameters.f_max;
        case PA_OPT:
            return &parameters.pa_opt;
        case FA_OPT:
            return &parameters.fa_opt;
    }
}

void resetKeyboardState(KeyboardState& keyboardState) {
    keyboardState.count = 0;
    keyboardState.ok = false;
    keyboardState.back = false;
}

int validate(int val, int cursorCode) {
    int min = 0;
    int max = 0;
    switch (cursorCode) {
        case MAIN:
            max = 3; break;
        case PARAMETER:
        case SETTINGS:
            max = 3; break;
        case ALARM:
            max = 2; break;
        case PID_SETTINGS:
            max = 4; break;
        case MODE_OPT:
            max = 1; break;
        case PERC_V_OPT:
            min = 80; max = 90; break;
        case PIP_OPT:
            min = 5; max = 20; break;
        case BPM_OPT:
            min = 70; max = 100; break;
        case IE_OPT:
            min = 1; max = 3; break;
        case PIP_ALARM_OPT:
            min = 100; max = 200; break;
        case PEEP_ALARM_OPT:
            min = 500; max = 700; break;
        case VT_ALARM_OPT:
            min = 5; max = 10; break;
        case TRIM_OPT:
            min = 70; max = 100; break;
        case FIL_OPT:
            min = 0; max = 1; break;
        case AUTO_OPT:
            min = 0; max = 1; break;
        case CD_OPT:
            min = 70; max = 100; break;
        case DP_OPT:
            min = 70; max = 100; break;
        case F_OPT:
            min = 70; max = 100; break;
        case FF_OPT:
            min = 70; max = 100; break;
        case PA_OPT:
            min = 70; max = 100; break;
        case FA_OPT:
            min = 70; max = 100; break;
    }
    if (val < min) {
        return min;
    } else if (val > max) {
        return max;
    } else {
        return val;
    }
}

void checkKeyboardActions(KeyboardState& keyboardState, MenuState& menuState, VariableParameters& variables) {
    if (menuState.menu == MAIN) {
        if (keyboardState.ok) {
            menuState.menu = menuState.cursorCode;
            menuState.changedMenu = true;
            menuState.previousCursorCode = menuState.cursorCode;
            menuState.cursorCode = getCursorCode(menuState.menu, 0);
            resetKeyboardState(keyboardState);
        } else {
            keyboardState.count = validate(keyboardState.count, MAIN);
            menuState.previousCursorCode = menuState.cursorCode;
            menuState.cursorCode = getCursorCode(MAIN, keyboardState.count);
        }
    } else {
        if (menuState.isEditingParam) {
            if (keyboardState.ok) {
                *getValueToEdit(menuState.cursorCode, variables) = menuState.editedParameter;
                menuState.isEditingParam = false;
                resetKeyboardState(keyboardState);
                keyboardState.count = keyboardState.previousCount;
            } else if (keyboardState.back) {
                menuState.isEditingParam = false;
                resetKeyboardState(keyboardState);
                keyboardState.count = keyboardState.previousCount;
            } else {
                int newValue = menuState.editedParameter + keyboardState.count;
                newValue = validate(newValue, menuState.cursorCode);
                menuState.editedParameter = newValue;
            }
        } else {
            if (keyboardState.ok) {
                menuState.isEditingParam = true;
                menuState.editedParameter = *getValueToEdit(menuState.cursorCode, variables);
                resetKeyboardState(keyboardState);
            } else if (keyboardState.back) {
                menuState.menu = MAIN;
                menuState.cursorCode = getCursorCode(MAIN, 0);
                menuState.changedMenu = true;
                resetKeyboardState(keyboardState);
            } else {
                keyboardState.count = validate(keyboardState.count, menuState.menu);
                keyboardState.previousCount = keyboardState.count;
                menuState.previousCursorCode = menuState.cursorCode;
                menuState.cursorCode = getCursorCode(menuState.menu, keyboardState.count);
            }
        }
    }
}
