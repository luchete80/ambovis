//
// Created by Mirian Quinteros on 19/03/2022.
//

#ifndef MENUSELECTOR_H
#define MENUSELECTOR_H

#include "MenuDataTypes.h"

Cursor getMenuCursor(int menu) {
    switch (menu) {
        case PARAMETER:
            return C_PARAMS;
        case ALARM:
            return C_ALARMS;
        case SETTINGS:
            return C_SETTINGS;
        case PID_SETTINGS:
            return C_PID_SETTINGS;
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

int validate(int val, int min, int max) {
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
            menuState.menu = getOption(menuState.menu, keyboardState.count).code;
            menuState.changedMenu = true;
            menuState.cursor = getOption(menuState.menu, 0);
            resetKeyboardState(keyboardState);
        } else {
            keyboardState.count = validate(keyboardState.count, 0, SIZE_MENU - 1);
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
                Cursor menuCursor = getMenuCursor(menuState.menu);
                keyboardState.count = validate(keyboardState.count, menuCursor.min, menuCursor.max);
                menuState.cursor = getOption(menuState.menu, keyboardState.count);
            }
        }
    }
}

#endif //MENUSELECTOR_H
