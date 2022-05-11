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

void clearDigits(Menu& menu, int x, int y, int pos=1) {
    for (int i=0; i<pos; i++) {
        menu.lcd.setCursor(x+i, y);
        menu.lcd.print(" ");
    }
}
void printBackDigit(Menu& menu, int x, int y) {
    menu.lcd.setCursor(x, y);
    menu.lcd.write(byte(0));
}

void printDigits(Menu& menu, Pos pos, String s) {
    menu.lcd.setCursor(pos.x, pos.y);
    menu.lcd.print(s);
}

void displayMainMenu(Menu& menu) {
    printDigits(menu, menu.menuState.cursor.start, ">");
    writeLine(menu, 0, "Seleccione un menu", 1);
    writeLine(menu, 1, "Parametros Principales", 1);
    writeLine(menu, 2, "Alarmas", 1);
    writeLine(menu, 3, "Ajustes", 1);
    writeLine(menu, 4, "Ajustes PID 1", 1);
}

void displaySensorValues(int line, Menu& menu, VariableParameters& parameters) {
    writeLine(menu, line, "PIP:-", 1);
    //    dtostrf(last_pressure_min, 2, 0, tempstr);
    writeLine(menu, line, "PEEP:12.1", 12);
    //    dtostrf((_mllastInsVol + _mllastExsVol)/2.*options.respiratoryRate*0.001, 2, 1, tempstr);
    writeLine(menu, line, "VM:11.5", 20);
}

void displayParametersSettings(Menu& menu, VariableParameters& parameters) {
    writeLine(menu, 0, "Parametros Principales", 1);

    printDigits(menu, menu.menuState.cursor.start, ">");

    if (parameters.vent_mode == VENTMODE_PCL) {
        writeLine(menu, 1, "MOD:", 1);
        printDigits(menu, C_P1.valuePos, getPrintableParameter(C_P1.code, menu.menuState, parameters));
        writeLine(menu, 1, "PIP:", 4);
        printDigits(menu, C_P5.valuePos, getPrintableParameter(C_P5.code, menu.menuState, parameters));
    } else if (parameters.vent_mode == VENTMODE_MAN ) {
        writeLine(menu, 1, "MOD: VCV", 1);
        printDigits(menu, C_P1.valuePos, getPrintableParameter(C_P1.code, menu.menuState, parameters));
        writeLine(menu, 1, "%V:", 4);
        printDigits(menu, C_P2.valuePos, getPrintableParameter(C_P2.code, menu.menuState, parameters));
        writeLine(menu, 1, "(Vt):", 8);
    }
    writeLine(menu, 2, "BPM:", 1);
    printDigits(menu, C_P3.valuePos, getPrintableParameter(C_P3.code, menu.menuState, parameters));
    writeLine(menu, 2, "IE:", 4);
    printDigits(menu, C_P4.valuePos, getPrintableParameter(C_P4.code, menu.menuState, parameters));
    writeLine(menu, 2, "ti:", 8);
    displaySensorValues(3, menu, parameters);
}

void displayAlarmSettings(Menu& menu, VariableParameters& parameters) {
    writeLine(menu, 0, "Alarmas", 1);
    writeLine(menu, 1, "PIP:", 1);
    writeLine(menu, 1, "PEEP:", 6);
    writeLine(menu, 2, "VT:", 1);
    writeLine(menu, 2, "VM:", 6);
    writeLine(menu, 3, "AMBU:", 1);
    writeLine(menu, 3, "CIC:", 6);
}

void displaySettings(Menu& menu, VariableParameters& parameters) {
    writeLine(menu, 0, "Ajustes", 1);
    writeLine(menu, 1, "Trim", 1);
    writeLine(menu, 2, "FIL:", 1);
    writeLine(menu, 3, "AUTO:", 1);
    writeLine(menu, 3, "CD:", 6);
}

void displayPIDSettings(Menu& menu, VariableParameters& parameters) {
    writeLine(menu, 0, "Ajustes PID", 1);
    writeLine(menu, 1, "DP", 1);
    writeLine(menu, 2, "f:", 1);
    writeLine(menu, 2, "F:", 6);
    writeLine(menu, 3, "Pa:", 1);
    writeLine(menu, 3, "Fa:", 6);
}

void displayEditedValue(Menu& menu, VariableParameters& parameters) {
    clearDigits(menu, menu.menuState.cursor.start.x, menu.menuState.cursor.start.y);
    clearDigits(menu, menu.menuState.cursor.valuePos.x, menu.menuState.cursor.valuePos.y);
    printBackDigit(menu, menu.menuState.cursor.start.x, menu.menuState.cursor.start.y);
    printDigits(menu, menu.menuState.cursor.valuePos, getPrintableParameter(menu.menuState.cursor.code, menu.menuState, parameters));
}

void refreshDisplay(Menu& menu, VariableParameters& parameters, bool shouldClearDisplay) {
    if (shouldClearDisplay) {
        menu.lcd.clear();
    }
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

void checkEncoder(Menu& menu, VariableParameters& parameters, unsigned long time) {

    checkKeyboard(menu.keyboardState, time);

    bool somethingChanged = menu.keyboardState.lastKeyPressedTime - time > 500;

    checkKeyboardActions(menu.keyboardState, menu.menuState, parameters);

    if (menu.menuState.changedMenu) {
        refreshDisplay(menu, parameters, true);
        menu.menuState.changedMenu = false;
    } else {
        if (menu.menuState.isEditingParam) {
            displayEditedValue(menu, parameters);
        }
        if (somethingChanged) {
            refreshDisplay(menu, parameters, false);
        }
    }
}
