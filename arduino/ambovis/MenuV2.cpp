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

void initDisplay(Menu& menu) {
    menu.lcd.begin(20, 4);
    menu.lcd.clear();
    menu.lcd.setCursor(0, 0);
    menu.lcd.createChar(0, back2);
}

void writeLine(Menu& menu, int line, String message = "", int offsetLeft = 0) {
    menu.lcd.setCursor(offsetLeft, line);
    menu.lcd.print(message);
}

void clearDigits(Menu& menu, Pos pos) {
    for (int i=0; i< pos.off; i++) {
        menu.lcd.setCursor(pos.x+i, pos.y);
        menu.lcd.print(" ");
    }
}
void printBackDigit(Menu& menu, Pos pos) {
    menu.lcd.setCursor(pos.x, pos.y);
    menu.lcd.write(byte(0));
}

void printDigits(Menu& menu, Pos pos, String s) {
    menu.lcd.setCursor(pos.x, pos.y);
    menu.lcd.print(s);
}

void displayMainMenu(Menu& menu) {
    printDigits(menu, menu.menuState.cursor.cursorDisplay.cursor, ">");
    writeLine(menu, 0, "Seleccione un menu", 1);
    writeLine(menu, 1, "Parametros Principales", 1);
    writeLine(menu, 2, "Alarmas", 1);
    writeLine(menu, 3, "Ajustes", 1);
    writeLine(menu, 4, "Ajustes PID 1", 1);
}

String getValueToDisplay(int code, VariableParameters& parameters, MenuState& menuState) {
    bool useEditedParam = menuState.cursor.code == code && menuState.isEditingParam;
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

void printCursor(Menu& menu, CursorDisplay& cursorDisplay, VariableParameters & parameters) {
    String valueToDisplay = getValueToDisplay(cursorDisplay.code, parameters, menu.menuState);
    clearDigits(menu, cursorDisplay.cursor);
    clearDigits(menu, cursorDisplay.value);
    if (menu.menuState.cursor.code == cursorDisplay.code) {
        if (menu.menuState.isEditingParam) {
            printBackDigit(menu, cursorDisplay.cursor);
        } else {
            printDigits(menu, cursorDisplay.cursor, ">");
        }
    }
    writeLine(menu, cursorDisplay.cursor.y, cursorDisplay.label, 1);
    printDigits(menu, cursorDisplay.value, valueToDisplay);
}

void displaySensorValues(int line, Menu& menu, VariableParameters& parameters) {
    char tempstr[5];
    writeLine(menu, line, "PIP:", 1);
    dtostrf(parameters.last_pressure_min, 2, 0, tempstr);
    writeLine(menu, line, "PEEP:", 12);
    dtostrf((parameters._mllastInsVol + parameters._mllastExsVol)/2.*parameters.respiratoryRate*0.001, 2, 1, tempstr);
    writeLine(menu, line, "VM:", 20);
}

void displayParametersSettings(Menu& menu, VariableParameters& parameters) {
    writeLine(menu, 0, "Parametros Principales", 1);

    if (parameters.vent_mode == VENTMODE_PCL) {
        printCursor(menu, CD_P1, parameters);
        printCursor(menu, CD_P5, parameters);
    } else if (parameters.vent_mode == VENTMODE_MAN ) {
        printCursor(menu, CD_P1, parameters);
        printCursor(menu, CD_P2, parameters);
    }
    printCursor(menu, CD_P3, parameters);
    printCursor(menu, CD_P4, parameters);

    displaySensorValues(3, menu, parameters);
}

void displayAlarmSettings(Menu& menu, VariableParameters& parameters) {
    writeLine(menu, 0, "Alarmas", 1);
    printCursor(menu, CD_AL1, parameters);
    printCursor(menu, CD_AL2, parameters);
    printCursor(menu, CD_AL3, parameters);
}

void displaySettings(Menu& menu, VariableParameters& parameters) {
    writeLine(menu, 0, "Ajustes", 1);
    printCursor(menu, CD_S1, parameters);
    printCursor(menu, CD_S2, parameters);
    printCursor(menu, CD_S3, parameters);
    printCursor(menu, CD_S4, parameters);
}

void displayPIDSettings(Menu& menu, VariableParameters& parameters) {
    writeLine(menu, 0, "Ajustes PID", 1);
    printCursor(menu, CD_PS1, parameters);
    printCursor(menu, CD_PS2, parameters);
    printCursor(menu, CD_PS3, parameters);
    printCursor(menu, CD_PS4, parameters);
    printCursor(menu, CD_PS5, parameters);
}

void printMenu(Menu& menu, VariableParameters& parameters) {
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

void displayMenu(Menu& menu, VariableParameters& parameters) {
    if (menu.menuState.changedMenu) {
        menu.lcd.clear();
        printMenu(menu, parameters);
    } else {
        printCursor(menu, menu.menuState.cursor.cursorDisplay, parameters);
    }
}

void checkEncoder(Menu& menu, VariableParameters& parameters, unsigned long time) {
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