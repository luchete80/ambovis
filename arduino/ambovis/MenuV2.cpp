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
    int size = 0;
    String label = "";
    String strValue = "";
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
            y = 0; break;
        case ALARM:
            y = 1; break;
        case SETTINGS:
            y = 2; break;
        case PID_SETTINGS:
            y = 3; break;
    }
    printDigits(menu, x, y , ">");
    writeLine(menu, 0, "Parametros", 1);
    writeLine(menu, 1, "Alarmas", 1);
    writeLine(menu, 2, "Ajustes", 1);
    writeLine(menu, 3, "Ajustes PID1", 1);
}

Pos getValueToDisplay(int code, VariableParameters& parameters, MenuState& menuState) {
    bool useEditedParam = menuState.cursorCode == code && menuState.isEditingParam;
    Pos displayValue;
    int value = 0;
    char tempstr[5];
    switch (code) {
        case PARAMETER:
            displayValue.c_x = 0; displayValue.c_y = 0; break;
        case ALARM:
            displayValue.c_x = 0; displayValue.c_y = 1; break;
        case SETTINGS:
            displayValue.c_x = 0; displayValue.c_y = 2; break;
        case PID_SETTINGS:
            displayValue.c_x = 0; displayValue.c_y = 3; break;
        case MODE_OPT:
            displayValue.x = 1; displayValue.y = 0; displayValue.size = 7;
            displayValue.c_x = 0; displayValue.c_y = 0; displayValue.label = "MOD:";
            value = useEditedParam ? menuState.editedParameter : parameters.vent_mode;
            displayValue.strValue = value == 0 ? String("MAN") : String("PCL");
            break;
        case PERC_V_OPT:
            displayValue.x = 10; displayValue.y = 0; displayValue.size = 6;
            displayValue.c_x = 9; displayValue.c_y = 0; displayValue.label = "V%:";
            value = useEditedParam ? menuState.editedParameter : parameters.percVolume;
            displayValue.strValue = String(value);
            break;
        case BPM_OPT:
            displayValue.x = 1; displayValue.y = 1; displayValue.size = 7;
            displayValue.c_x = 0; displayValue.c_y = 1; displayValue.label = "BPM:";
            value = useEditedParam ? menuState.editedParameter : parameters.respiratoryRate;
            displayValue.strValue = String(value);
            break;
        case IE_OPT:
            displayValue.x = 10; displayValue.y = 1; displayValue.size = 5;
            displayValue.c_x = 9; displayValue.c_y = 1; displayValue.label = "IE:";
            value = useEditedParam ? menuState.editedParameter : parameters.percInspEsp;
            displayValue.strValue = String(value);
            break;
        case PIP_OPT:
            displayValue.x = 10; displayValue.y = 0; displayValue.size = 6;
            displayValue.c_x = 9; displayValue.c_y = 0; displayValue.label = "PIP:";
            value = useEditedParam ? menuState.editedParameter : parameters.peakInspiratoryPressure;
            displayValue.strValue = String(value);
            break;
        case PIP_ALARM_OPT:
            displayValue.x = 1; displayValue.y = 0; displayValue.size = 9;
            displayValue.c_x = 0; displayValue.c_y = 0; displayValue.label = "PIPAL:";
            value = useEditedParam ? menuState.editedParameter : parameters.alarm_max_pressure;
            displayValue.strValue = String(value);
            break;
        case PEEP_ALARM_OPT:
            displayValue.x = 1; displayValue.y = 1; displayValue.size = 10;
            displayValue.c_x = 0; displayValue.c_y = 1; displayValue.label = "PEEPAL:";
            value = useEditedParam ? menuState.editedParameter : parameters.alarm_peep_pressure;
            displayValue.strValue = String(value);
            break;
        case VT_ALARM_OPT:
            displayValue.x = 1; displayValue.y = 2; displayValue.size = 8;
            displayValue.c_x = 0; displayValue.c_y = 2; displayValue.label = "VTAL:";
            value = useEditedParam ? menuState.editedParameter : parameters.alarm_vt;
            displayValue.strValue = String(value);
            break;
        case TRIM_OPT:
            displayValue.x = 1; displayValue.y = 0; displayValue.size = 10;
            displayValue.c_x = 0; displayValue.c_y = 0; displayValue.label = "TRIM:";
            value = useEditedParam ? menuState.editedParameter : parameters.p_trim;
            dtostrf((float(value-100)), 2, 0, tempstr);
            displayValue.strValue = String(tempstr) + String("e-3");
            break;
        case FIL_OPT:
            displayValue.x = 1; displayValue.y = 1; displayValue.size = 7;
            displayValue.c_x = 0; displayValue.c_y = 1; displayValue.label = "FIL:";
            value = useEditedParam ? menuState.editedParameter : parameters.fil;
            displayValue.strValue = value == 0 ? String("OFF") : String("ON");
            break;
        case AUTO_OPT:
            displayValue.x = 1; displayValue.y = 2; displayValue.size = 8;
            displayValue.c_x = 0; displayValue.c_y = 2; displayValue.label = "AUTO:";
            value = useEditedParam ? menuState.editedParameter : parameters.autopid;
            displayValue.strValue = value == 0 ? String("OFF") : String("ON");
            break;
        case CD_OPT:
            displayValue.x = 10; displayValue.y = 2; displayValue.size = 6;
            displayValue.c_x = 9; displayValue.c_y = 2; displayValue.label = "CD:";
            value = useEditedParam ? menuState.editedParameter : parameters.cd_opt;
            dtostrf(value*1.01972, 2, 1, tempstr);
            displayValue.strValue = String(tempstr);
            break;
        case DP_OPT:
            displayValue.x = 1; displayValue.y = 0; displayValue.size = 6;
            displayValue.c_x = 0; displayValue.c_y = 0; displayValue.label = "dp:";
            value = useEditedParam ? menuState.editedParameter : parameters.dp_opt;
            displayValue.strValue = String(value);
            break;
        case F_OPT:
            displayValue.x = 1; displayValue.y = 1; displayValue.size = 5;
            displayValue.c_x = 0; displayValue.c_y = 1; displayValue.label = "f:";
            value = useEditedParam ? menuState.editedParameter : parameters.f_min;
            displayValue.strValue = String(value);
            break;
        case FF_OPT:
            displayValue.x = 9; displayValue.y = 1; displayValue.size = 6;
            displayValue.c_x = 8; displayValue.c_y = 1; displayValue.label = "F:";
            value = useEditedParam ? menuState.editedParameter : parameters.f_max;
            displayValue.strValue = String(value);
            break;
        case PA_OPT:
            displayValue.x = 1; displayValue.y = 2; displayValue.size = 6;
            displayValue.c_x = 0; displayValue.c_y = 2; displayValue.label = "pa:";
            value = useEditedParam ? menuState.editedParameter : parameters.pa_opt;
            displayValue.strValue = String(value);
            break;
        case FA_OPT:
            displayValue.x = 9; displayValue.y = 2; displayValue.size = 6;
            displayValue.c_x = 8; displayValue.c_y = 2; displayValue.label = "fa:";
            value = useEditedParam ? menuState.editedParameter : parameters.fa_opt;
            displayValue.strValue = String(value);
            break;
    }
    return displayValue;
}

void printCursor(MenuV2& menu, int cursorCode, VariableParameters & parameters) {
    Pos valueToDisplay = getValueToDisplay(cursorCode, parameters, menu.menuState);
    clearDigits(menu, valueToDisplay.c_x, valueToDisplay.c_y, 1);
    clearDigits(menu, valueToDisplay.x, valueToDisplay.y, valueToDisplay.size);

    if (menu.menuState.cursorCode == cursorCode) {
        if (menu.menuState.isEditingParam) {
            printBackDigit(menu, valueToDisplay.c_x, valueToDisplay.c_y);
        } else {
            printDigits(menu, valueToDisplay.c_x, valueToDisplay.c_y, ">");
        }
    }
    printDigits(menu, valueToDisplay.x, valueToDisplay.y, valueToDisplay.label + valueToDisplay.strValue);
}

void displaySensorValues(int line, MenuV2& menu, VariableParameters& parameters) {
    char tempstr[5];
    writeLine(menu, line, "PIP:", 1);
    dtostrf(parameters.last_pressure_min, 2, 0, tempstr);
    writeLine(menu, line, "PEEP:", 8);
    dtostrf((parameters._mllastInsVol + parameters._mllastExsVol)/2.*parameters.respiratoryRate*0.001, 2, 1, tempstr);
    writeLine(menu, line, "VM:", 15);
}

void displayParametersSettings(MenuV2& menu, VariableParameters& parameters) {
    if (parameters.vent_mode == VENTMODE_PCL) {
        printCursor(menu, MODE_OPT, parameters);
        printCursor(menu, PIP_OPT, parameters);
    } else {
        printCursor(menu, MODE_OPT, parameters);
        printCursor(menu, PERC_V_OPT, parameters);
    }
    printCursor(menu, BPM_OPT, parameters);
    printCursor(menu, IE_OPT, parameters);

    displaySensorValues(3, menu, parameters);
}

void displayAlarmSettings(MenuV2& menu, VariableParameters& parameters) {
    printCursor(menu, PIP_ALARM_OPT, parameters);
    printCursor(menu, PEEP_ALARM_OPT, parameters);
    printCursor(menu, VT_ALARM_OPT, parameters);
}

void displaySettings(MenuV2& menu, VariableParameters& parameters) {
    printCursor(menu, TRIM_OPT, parameters);
    printCursor(menu, FIL_OPT, parameters);
    printCursor(menu, AUTO_OPT, parameters);
    printCursor(menu, CD_OPT, parameters);
}

void displayPIDSettings(MenuV2& menu, VariableParameters& parameters) {
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
        Pos valueToDisplay = getValueToDisplay(menu.menuState.previousCursorCode, parameters, menu.menuState);
        clearDigits(menu, valueToDisplay.c_x, valueToDisplay.c_y, 1);
        printCursor(menu, menu.menuState.cursorCode, parameters);
    }
}

void checkEncoder(MenuV2& menu, VariableParameters& parameters, long time) {
    checkKeyboard(menu.keyboardState, time);
    bool somethingChanged = menu.keyboardState.lastKeyPressedTime - time < 500;
    checkKeyboardActions(menu.keyboardState, menu.menuState, parameters);

    if (somethingChanged) {
        Serial.print("Keypressed count ");Serial.println(menu.keyboardState.count);
        displayMenu(menu, parameters);
        if (menu.menuState.changedMenu) {
            menu.menuState.changedMenu = false;
        }
    }
}