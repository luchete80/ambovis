//
// Created by Mirian Quinteros on 28/06/2022.
//

#include "MenuV2.h"

byte back2[8] = {
        0b00100,
        0b01000,
        0b11111,
        0b01001,
        0b00101,
        0b00001,
        0b00001,
        0b11111
};

typedef struct pos {
    int c_x = 0;
    int c_y = 0;
    int x = 0;
    int y = 0;
    String label = "";
} Pos;

void initDisplay(MenuV2& menu) {
    menu.lcd->begin(20, 4);
    menu.lcd->clear();
    menu.lcd->setCursor(0, 0);
    menu.lcd->createChar(0, back2);
}

void writeLine(MenuV2& menu, int line, String message = "", int offsetLeft = 0) {
    menu.lcd->setCursor(offsetLeft, line);
    menu.lcd->print(message);
}

void clearDigits(MenuV2& menu, int x, int y, int off) {
    for (int i=0; i< off; i++) {
        menu.lcd->setCursor(x+i, y);
        menu.lcd->print(" ");
    }
}
void printBackDigit(MenuV2& menu, int x, int y) {
    menu.lcd->setCursor(x, y);
    menu.lcd->write(byte(0));
}

void printDigits(MenuV2& menu, int x, int y, String s) {
    menu.lcd->setCursor(x, y);
    menu.lcd->print(s);
}

void displayMainMenu(MenuV2& menu) {
    int x, y = 0;
    switch (menu.menuState.cursorCode) {
        case PARAMETER:
            y = 1; break;
        case ALARM:
            y = 2; break;
        case SETTINGS:
            y = 3; break;
        case PID_SETTINGS:
            y = 4; break;
    }
    printDigits(menu, x, y , ">");
    writeLine(menu, 0, "Seleccione un menu", 1);
    writeLine(menu, 1, "Parametros Principales", 1);
    writeLine(menu, 2, "Alarmas", 1);
    writeLine(menu, 3, "Ajustes", 1);
    writeLine(menu, 4, "Ajustes PID 1", 1);
}

String getValueToDisplay(int code, VariableParameters& parameters, MenuState& menuState) {
    bool useEditedParam = menuState.cursorCode == code && menuState.isEditingParam;
    int value;
    char tempstr[5];
    switch (code) {
        case PARAMETER:
        case ALARM:
        case SETTINGS:
        case PID_SETTINGS:
            return String("");
        case MODE_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.vent_mode;
            return value == 0 ? String("MAN") : String("PCL");
        case PERC_V_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.percVolume;
            return String(value);
        case BPM_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.respiratoryRate;
            return String(value);
        case IE_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.percInspEsp;
            return String(value);
        case PIP_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.peakInspiratoryPressure;
            return String(value);
        case PIP_ALARM_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.alarm_max_pressure;
            return String(value);
        case PEEP_ALARM_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.alarm_peep_pressure;
            return String(value);
        case VT_ALARM_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.alarm_peep_pressure;
            return String(value);
        case TRIM_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.p_trim;
            dtostrf((float(value-100)), 2, 0, tempstr);
            return String(tempstr) + String("e-3");
        case FIL_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.fil;
            return value == 0 ? String("OFF") : String("ON");
        case AUTO_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.autopid;
            return value == 0 ? String("OFF") : String("ON");
        case CD_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.cd_opt;
            dtostrf(value*1.01972, 2, 1, tempstr);
            return String(tempstr);
        case DP_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.dp_opt;
            return String(value);
    }
}

Pos getCursorPosition(int cursorCode) {
    Pos value;
    switch (cursorCode) {
        case MODE_OPT:
            value.x = 3; value.y = 1; value.c_x = 0; value.c_y = 1; value.label = "MOD:";break;
        case PERC_V_OPT:
            value.x = 9; value.y = 1; value.c_x = 7; value.c_y = 1; value.label = "V%:"; break;
        case PIP_OPT:
            value.x = 9; value.y = 1; value.c_x = 7; value.c_y = 2; value.label = "PIP:"; break;
        case BPM_OPT:
            value.x = 3; value.y = 2; value.c_x = 7; value.c_y = 2; value.label = "BPM:"; break;
        case IE_OPT:
            value.x = 9; value.y = 2; value.c_x = 7; value.c_y = 2; value.label = "IE:"; break;

        case PIP_ALARM_OPT:
            value.x = 3; value.y = 1; value.c_x = 0; value.c_y = 1; value.label = "PIPAL:"; break;
        case PEEP_ALARM_OPT:
            value.x = 3; value.y = 2; value.c_x = 0; value.c_y = 2; value.label = "PEEPAL:"; break;
        case VT_ALARM_OPT:
            value.x = 3; value.y = 3; value.c_x = 0; value.c_y = 3; value.label = "VTAL:"; break;

        case TRIM_OPT:
            value.x = 3; value.y = 1; value.c_x = 0; value.c_y = 1; value.label = "TRIM:"; break;
        case FIL_OPT:
            value.x = 3; value.y = 2; value.c_x = 0; value.c_y = 2; value.label = "FIL:"; break;
        case AUTO_OPT:
            value.x = 3; value.y = 3; value.c_x = 0; value.c_y = 3; value.label = "AUTO:"; break;
        case CD_OPT:
            value.x = 9; value.y = 3; value.c_x = 7; value.c_y = 3; value.label = "CD:"; break;

        case DP_OPT:
            value.x = 3; value.y = 1; value.c_x = 0; value.c_y = 1; value.label = "dp:"; break;
        case F_OPT:
            value.x = 3; value.y = 2; value.c_x = 0; value.c_y = 2; value.label = "f:"; break;
        case FF_OPT:
            value.x = 9; value.y = 2; value.c_x = 7; value.c_y = 2; value.label = "FF:"; break;
        case PA_OPT:
            value.x = 3; value.y = 3; value.c_x = 0; value.c_y = 3; value.label = "pa:"; break;
        case FA_OPT:
            value.x = 9; value.y = 3; value.c_x = 7; value.c_y = 3; value.label = "fa:"; break;
    }
    return value;
}

void printCursor(MenuV2& menu, int cursorCode, VariableParameters & parameters) {
    Pos cursorPosition = getCursorPosition(cursorCode);
    String valueToDisplay = getValueToDisplay(cursorCode, parameters, menu.menuState);
    clearDigits(menu, cursorPosition.c_x, cursorPosition.c_y, 1);
    clearDigits(menu, cursorPosition.x, cursorPosition.y, 3);
    if (menu.menuState.cursorCode == cursorCode) {
        if (menu.menuState.isEditingParam) {
            printBackDigit(menu, cursorPosition.c_x, cursorPosition.c_y);
        } else {
            printDigits(menu, cursorPosition.c_x, cursorPosition.c_y, ">");
        }
    }
    writeLine(menu, cursorPosition.y, cursorPosition.label, 1);
    printDigits(menu, cursorPosition.x, cursorPosition.y, valueToDisplay);
}

void displaySensorValues(int line, MenuV2& menu, VariableParameters& parameters) {
    char tempstr[5];
    writeLine(menu, line, "PIP:", 1);
    dtostrf(parameters.last_pressure_min, 2, 0, tempstr);
    writeLine(menu, line, "PEEP:", 12);
    dtostrf((parameters._mllastInsVol + parameters._mllastExsVol)/2.*parameters.respiratoryRate*0.001, 2, 1, tempstr);
    writeLine(menu, line, "VM:", 20);
}

void displayParametersSettings(MenuV2& menu, VariableParameters& parameters) {
    writeLine(menu, 0, "Parametros Principales", 1);

    if (parameters.vent_mode == VENTMODE_PCL) {
        printCursor(menu, MODE_OPT, parameters);
        printCursor(menu, PIP_OPT, parameters);
    } else if (parameters.vent_mode == VENTMODE_MAN ) {
        printCursor(menu, MODE_OPT, parameters);
        printCursor(menu, PERC_V_OPT, parameters);
    }
    printCursor(menu, BPM_OPT, parameters);
    printCursor(menu, IE_OPT, parameters);

    displaySensorValues(3, menu, parameters);
}

void displayAlarmSettings(MenuV2& menu, VariableParameters& parameters) {
    writeLine(menu, 0, "Alarmas", 1);
    printCursor(menu, PIP_ALARM_OPT, parameters);
    printCursor(menu, PEEP_ALARM_OPT, parameters);
    printCursor(menu, VT_ALARM_OPT, parameters);
}

void displaySettings(MenuV2& menu, VariableParameters& parameters) {
    writeLine(menu, 0, "Ajustes", 1);
    printCursor(menu, TRIM_OPT, parameters);
    printCursor(menu, FIL_OPT, parameters);
    printCursor(menu, AUTO_OPT, parameters);
    printCursor(menu, CD_OPT, parameters);
}

void displayPIDSettings(MenuV2& menu, VariableParameters& parameters) {
    writeLine(menu, 0, "Ajustes PID", 1);
    printCursor(menu, DP_OPT, parameters);
    printCursor(menu, F_OPT, parameters);
    printCursor(menu, FF_OPT, parameters);
    printCursor(menu, PA_OPT, parameters);
    printCursor(menu, FA_OPT, parameters);
}

void printMenu(MenuV2& menu, VariableParameters& parameters) {
    if ( menu.menuState.menu == MAIN ) { //All the titles
        displayMainMenu(menu);
    } else if ( menu.menuState.menu == PARAMETER ) {
        displayParametersSettings(menu, parameters);
    } else if ( menu.menuState.menu == ALARM ) {
        displayAlarmSettings(menu, parameters);
    } else if ( menu.menuState.menu == SETTINGS ) {
        displaySettings(menu, parameters);
    } else if ( menu.menuState.menu == PID_SETTINGS ) {
        displayPIDSettings(menu, parameters);
    }
}

void displayMenu(MenuV2& menu, VariableParameters& parameters) {
    if (menu.menuState.changedMenu) {
        menu.lcd->clear();
        printMenu(menu, parameters);
    } else {
        printCursor(menu, menu.menuState.cursorCode, parameters);
    }
}

void checkEncoder(MenuV2& menu, VariableParameters& parameters, long time) {
    checkKeyboard(menu.keyboardState, time);
    bool somethingChanged = menu.keyboardState.lastKeyPressedTime - time > 500;
    checkKeyboardActions(menu.keyboardState, menu.menuState, parameters);

    if (somethingChanged) {
        displayMenu(menu, parameters);
        if (menu.menuState.changedMenu) {
            menu.menuState.changedMenu = false;
        }
    }
}