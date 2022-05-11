//
// Created by Mirian Quinteros on 19/03/2022.
//

#ifndef MENUSELECTOR_H
#define MENUSELECTOR_H

#include "MenuDataTypes.h"

Cursor getCursorByCode(int cursorCode) {
    switch (cursorCode) {
        case PARAMETER:
            return C_PARAMS;
        case ALARM:
            return C_ALARMS;
        case SETTINGS:
            return C_SETTINGS;
        case PID_SETTINGS:
            return C_PID_SETTINGS;
        case MODE_OPT:
            return C_P1;
        case PERC_V_OPT:
            return C_P2;
        case BPM_OPT:
            return C_P3;
        case IE_OPT:
            return C_P4;
        case PIP_OPT:
            return C_P5;
        case PIP_ALARM_OPT:
            return C_AL1;
        case PEEP_ALARM_OPT:
            return C_AL2;
        case VT_ALARM_OPT:
            return C_AL3;
        case VM_ALARM_OPT:
            return C_AL4;
        case AMBU_ALARM_OPT:
            return C_AL5;
        case TRIM_OPT:
            return C_S1;
        case FIL_OPT:
            return C_S2;
        case AUTO_OPT:
            return C_S3;
        case CD_OPT:
            return C_S4;
        case DP_OPT:
            return C_PS1;
        case F_OPT:
            return C_PS2;
        case FF_OPT:
            return C_PS3;
        case PA_OPT:
            return C_PS4;
        case FA_OPT:
            return C_PS5;
    }
}

Cursor getOption(int menu, int i) {
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
            return &parameters.alarm_vt;
        case PEEP_ALARM_OPT:
            return &parameters.alarm_peep_pressure;
        case VT_ALARM_OPT:
            return &parameters.alarm_vt;
        case VM_ALARM_OPT:
            return &parameters.alarm_vt;
        case AMBU_ALARM_OPT:
            return &parameters.dpip_b;
        case TRIM_OPT:
            return &parameters.p_trim;
        case FIL_OPT:
            return &parameters.ff_opt; // filter is boolean
        case AUTO_OPT:
            return &parameters.autopid;
        case CD_OPT:
            return &parameters.cd_opt;
        case DP_OPT:
            return &parameters.dp_opt;
        case F_OPT:
            return &parameters.f_acc;
        case FF_OPT:
            return &parameters.ff_opt;
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

int validate(int val, int min, int max) {
    if (val < min) {
        return min;
    } else if (val > max) {
        return max;
    } else {
        return val;
    }
}

String getPrintableParameter(int paramCode, MenuState& menuState, VariableParameters& parameters) {
    if (menuState.cursor.code == paramCode) {
        if (menuState.isEditingParam) {
            return String(menuState.editedParameter);
        } else {
            return String(*getValueToEdit(menuState.cursor.code, parameters));
        }
    } else {
        return String(*getValueToEdit(paramCode, parameters));
    }
}

void checkKeyboardActions(KeyboardState& keyboardState, MenuState& menuState, VariableParameters& variables) {
    if (menuState.menu == MAIN) {
        if (keyboardState.ok) {
            menuState.menu = getOption(MAIN, keyboardState.count).code;
            menuState.changedMenu = true;
            menuState.cursor = getOption(menuState.menu, 0);
            resetKeyboardState(keyboardState);
        } else {
            keyboardState.count = validate(keyboardState.count, 0, SIZE_MENU -1);
            menuState.cursor = getOption(MAIN, keyboardState.count);
        }
    } else {
        if (menuState.isEditingParam) {
            if (keyboardState.ok) {
                *getValueToEdit(menuState.cursor.code, variables) = menuState.editedParameter;
                menuState.isEditingParam = false;
                resetKeyboardState(keyboardState);
            } else if (keyboardState.back) {
                menuState.isEditingParam = false;
                resetKeyboardState(keyboardState);
            } else {
                int newValue = menuState.editedParameter + keyboardState.count;
                newValue = validate(newValue, menuState.cursor.min, menuState.cursor.max);
                menuState.editedParameter = newValue;
            }
        } else {
            if (keyboardState.ok) {
                menuState.cursor = getOption(menuState.menu, keyboardState.count);
                menuState.isEditingParam = true;
                menuState.editedParameter = *getValueToEdit(menuState.cursor.code, variables);
                resetKeyboardState(keyboardState);
            } else if (keyboardState.back) {
                menuState.menu = MAIN;
                menuState.cursor = getOption(menuState.menu, 0);
                menuState.changedMenu = true;
                resetKeyboardState(keyboardState);
            } else {
                Cursor menuCursor = getCursorByCode(menuState.menu);
                keyboardState.count = validate(keyboardState.count, menuCursor.min, menuCursor.max);
                menuState.cursor = getOption(menuState.menu, keyboardState.count);
            }
        }
    }
}

#endif //MENUSELECTOR_H
