//
// Created by Mirian Quinteros on 19/03/2022.
//

#ifndef MENUSELECTOR_H
#define MENUSELECTOR_H

#include "MenuConstants.h"

void moveCursorInMenu(int menuNumber, int currenSelection, int moves, int ventMode) {
    switch (menuNumber) {
        case MAIN_MENU:
            int currentCursor = (currentSelection + moves) % 4;
            switch (currentCursor) {
                case 0:
                    currentSelection = GO_TO_PARAMETER_MENU;
                    break;
                case 1:
                    currentSelection = GO_TO_ALARM_MENU;
                    break;
                case 2:
                    currentSelection = GO_TO_SETTINGS_MENU;
                    break;
                case 3:
                    currentSelection = GO_TO_PID_SETTINGS_MENU;
                    break;
            }
            break;
        case PARAMETER_MENU:
            int currentCursor = (currentSelection + moves) % 4;
            switch (currentCursor) {
                case 0:
                    currentSelection = MODE_OPT;
                    break;
                case 1:
                    currentSelection = ventMode == VENTMODE_MAN ? PERC_V_OPT : PIP_OPT;
                    break;
                case 2:
                    currentSelection = BPM_OPT;
                    break;
                case 3:
                    currentSelection = IE_OPT;
                    break;
            }
            break;
        case ALARM_MENU:
            int currentCursor = (currentSelection + moves) % 5;
            switch (currentCursor) {
                case 0:
                    currentSelection = PIP_ALARM_OPT;
                    break;
                case 1:
                    currentSelection = PEEP_ALARM_OPT;
                    break;
                case 2:
                    currentSelection = VT_ALARM_OPT;
                    break;
                case 3:
                    currentSelection = VM_ALARM_OPT;
                    break;
                case 4:
                    currentSelection = AMBU_ALARM_OPT;
                    break;
            }
            break;
        case SETTINGS_MENU:
            break;
        case PID_SETTINGS_MENU:
            break;
    }
}

void doMainMenuAction(int menuNumber, int currentSelection) {
    switch (currentSelection) {
        case GO_TO_PARAMETER_MENU:
            menuNumber = PARAMETER_MENU;
            updateDisplay = true;
            break;
        case GO_TO_ALARM_MENU:
            menu.menuNumber = ALARM_MENU;
            menu.updateDisplay = true;
            break;
        case GO_TO_SETTINGS_MENU:
            menu.menuNumber = SETTINGS_MENU;
            menu.updateDisplay = true;
            break;
        case GO_TO_PID_SETTINGS_MENU:
            menu.menuNumber = PID_SETTINGS_MENU;
            menu.updateDisplay = true;
            break;
    }
}

int applyValidRange(int value, int min, int max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

void doParameterMenuAction(Menu& menu, VariableParameters& parameters) {
    switch (menu.currentSelection) {
        case MODE_OPT:
            int currentMode = parameters.vent_mode;
            int newSelectedMode = menu.menuKeyboard.value;
            newSelectedMode = applyValidRange(newSelectedMode, 1, 2);
            parameters.vent_mode = newSelectedMode;
            break;
            case PERC_V_OPT:
                int currentPercVol = parameters.percVolume;
                int newPercVol = menu.menuKeyboard.value;
                newPercVol = applyValidRange(newPercVol, 40, 100); //TODO extract default
                parameters.percVolume = newPercVol;
                break;
                case BPM_OPT:
                    int currentBpm = parameters.respiratoryRate;
                    int newBpm = menu.menuKeyboard.value;
                    newBpm = applyValidRange(newBpm, DEFAULT_MIN_RPM, DEFAULT_MAX_RPM);
                    parameters.respiratoryRate = newBpm;
                    break;
                    case IE_OPT:
                        int currentIE = parameters.percInspEsp;
                        int newIE = menu.menuKeyboard.value;
                        newIE = applyValidRange(newIE, 1, 3); //TODO extract default
                        parameters.percInspEsp = newIE;
                        break;
                        case PIP_OPT:
                            int currentPip = parameters.peakInspiratoryPressure;
                            int newPip = menu.menuKeyboard.value;
                            newPip = applyValidRange(newPip, 15, 30); //TODO extract default
                            parameters.peakInspiratoryPressure = newPip;
                            break;
    }
}

void updateSelection(Menu& menu, VariableParameters& parameters) {
    switch (menu.menuNumber) {
        case MAIN_MENU:
            doMainMenuAction(menu);
            break;
            case PARAMETER_MENU:
                doParameterMenuAction(menu, parameters);
                break;
                case ALARM_MENU:
                    break;
                case SETTINGS_MENU:
                    break;
                case PID_SETTINGS_MENU:
                    break;
    }
}

void checkEncoder(Menu& menu, SystemState& systemState, VariableParameters& parameters) {
    checkUPButtonPressed(menu.menuKeyboard, time);
    checkDOWNButtonPressed(menu.menuKeyboard, time);
    checkOKButtonPressed(menu.menuKeyboard, time);
    checkBackButtonPressed(menu.menuKeyboard, time);

    if (menu.menuKeyboard.itemSelected) {
        updateSelection(menu, parameters);
    } else {
        moveCursorInMenu(menu, parameters);
    }
}

#endif //MENUSELECTOR_H
