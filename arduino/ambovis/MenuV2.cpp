//
// Created by Mirian Quinteros on 28/06/2022.
//

#include "MenuV2.h"

typedef struct display_value {
    int cursorOffset = 0;
    int valueOffset = 0;
    int line = 0;
    int size = 0; // Should be label size + value size. Eg: "V%:12.3" size is 7
    String label = "";
    String strValue = "";
} DisplayValue;

void initDisplay(MenuV2& menu) {
    menu.lcd->begin(20, 4);
    menu.lcd->clear();
    menu.lcd->setCursor(0, 0);
    menu.lcd->createChar(0, back);
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

void clearCursor(MenuV2& menu, int x, int y) {
    clearDigits(menu, x, y, 1);
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
    }
    printDigits(menu, x, y , ">");
    writeLine(menu, 0, "Parametros", 1);
    writeLine(menu, 1, "Alarmas", 1);
    writeLine(menu, 2, "Ajustes", 1);
}

DisplayValue buildConfiguration(int line, int cursorOffset, int valueSize, String label, String value) {
    DisplayValue displayValue;
    displayValue.line = line;
    displayValue.cursorOffset = cursorOffset;
    displayValue.valueOffset = cursorOffset + 1;
    displayValue.size = label.length() + valueSize;
    displayValue.label = label;
    displayValue.strValue = value;
    return displayValue;
}

DisplayValue getVentilationModeConfig(int value, bool isSetupReady) {
    int line = isSetupReady ? 0 : 1;
    return buildConfiguration(line, 0, 3, "MOD:", String("MAN"));
}

DisplayValue getPercVConfig(int value, bool isSetupReady) {
    int line = isSetupReady ? 0 : 1;
    return buildConfiguration(line, 9, 3, "V%:", String(value));
}

DisplayValue getBPMConfig(int value, bool isSetupReady) {
    int line = isSetupReady ? 1 : 2;
    return buildConfiguration(line, 0, 3, "BPM:", String(value));
}

DisplayValue getIEConfig(int value, bool isSetupReady) {
    int line = isSetupReady ? 1 : 2;
    return buildConfiguration(line, 9, 1, "IE:", String(value));
}

DisplayValue getPIPConfig(int value, bool isSetupReady) {
    int line = isSetupReady ? 0 : 1;
    return buildConfiguration(line, 9, 3, "PIP:", String(value));
}

DisplayValue getPIPAlarmConfig(int value) {
    return buildConfiguration(0, 0, 3, "PIPAL:", String(value));
}

DisplayValue getPEEPAlarmConfig(int value) {
    return buildConfiguration(1, 0, 3, "PEEPAL:", String(value));
}

DisplayValue getVTAlarmConfig(int value) {
    return buildConfiguration(2, 0, 3, "VTAL:", String(value));
}

DisplayValue getFilterConfig(int value) {
    String strValue = value == 0 ? String("OFF") : String("ON");
    return buildConfiguration(0, 0, 3, "FIL:", strValue);
}

DisplayValue getAutopidConfig(int value) {
    String strValue = value == 0 ? String("OFF") : String("ON");
    return buildConfiguration(1, 0, 3, "AUTO:", strValue);
}

DisplayValue getSetupReadyConfig() {
    return buildConfiguration(3, 0, 6, "READY", "");
}

DisplayValue getValueToDisplay(int code, VariableParameters parameters, MenuState& menuState) {
    bool useEditedParam = menuState.cursorCode == code && menuState.isEditingParam;
    DisplayValue displayValue;
    int value;
    switch (code) {
        case PARAMETER:
            displayValue.cursorOffset = 0; displayValue.line = 0; break;
        case ALARM:
            displayValue.cursorOffset = 0; displayValue.line = 1; break;
        case SETTINGS:
            displayValue.cursorOffset = 0; displayValue.line = 2; break;
        case MODE_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.vent_mode;
            displayValue = getVentilationModeConfig(value, menuState.setupReady);
            break;
        case PERC_V_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.percVolume;
            displayValue = getPercVConfig(value, menuState.setupReady);
            break;
        case BPM_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.respiratoryRate;
            displayValue = getBPMConfig(value, menuState.setupReady);
            break;
        case IE_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.percInspEsp;
            displayValue = getIEConfig(value, menuState.setupReady);
            break;
        case PIP_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.peakInspiratoryPressure;
            displayValue = getPIPConfig(value, menuState.setupReady);
            break;
        case PIP_ALARM_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.alarm_max_pressure;
            displayValue = getPIPAlarmConfig(value);
            break;
        case PEEP_ALARM_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.alarm_peep_pressure;
            displayValue = getPEEPAlarmConfig(value);
            break;
        case VT_ALARM_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.alarm_vt;
            displayValue = getVTAlarmConfig(value);
            break;
        case FIL_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.filter;
            displayValue = getFilterConfig(value);
            break;
        case AUTO_OPT:
            value = useEditedParam ? menuState.editedParameter : parameters.autopid;
            displayValue = getAutopidConfig(value);
            break;
        case END_SETUP:
            displayValue = getSetupReadyConfig();
    }
    return displayValue;
}

void printCursor(MenuV2& menu, int cursorCode, VariableParameters parameters) {
    DisplayValue displayValue = getValueToDisplay(cursorCode, parameters, menu.menuState);

    clearCursor(menu, displayValue.cursorOffset, displayValue.line);
    clearDigits(menu, displayValue.valueOffset, displayValue.line, displayValue.size);

    if (menu.menuState.cursorCode == cursorCode) {
        if (menu.menuState.isEditingParam) {
            printBackDigit(menu, displayValue.cursorOffset, displayValue.line);
        } else {
            printDigits(menu, displayValue.cursorOffset, displayValue.line, ">");
        }
    }
    printDigits(menu, displayValue.valueOffset, displayValue.line, displayValue.label + displayValue.strValue);
}

void displaySensorValues(int line, MenuV2& menu, VariableParameters parameters, SensorData sensorData) {
    char tempStr[5];
    dtostrf(sensorData.last_pressure_min, 2, 0, tempStr);
    writeLine(menu, line, "PEEP:" + String(tempStr), 0);

    dtostrf((sensorData._mlLastInsVol + sensorData._mlLastExsVol)/2.*parameters.respiratoryRate*0.001, 2, 1, tempStr);
    writeLine(menu, line, "VM:" + String(tempStr), 10);
}

void displayInitialParametersSettings(MenuV2& menu, VariableParameters parameters) {
    writeLine(menu, 0, "Parametros inic", 1);
    writeLine(menu, 1, "MOD:MAN", 1);
    printCursor(menu, BPM_OPT, parameters);
    printCursor(menu, IE_OPT, parameters);
    printCursor(menu, END_SETUP, parameters);
}

void displayParametersSettings(MenuV2& menu, VariableParameters parameters, SensorData& sensorData) {
    writeLine(menu, 0, "MOD:MAN", 1);
    printCursor(menu, PERC_V_OPT, parameters);
    printCursor(menu, BPM_OPT, parameters);
    printCursor(menu, IE_OPT, parameters);

    displaySensorValues(3, menu, parameters, sensorData);
}

void displayAlarmSettings(MenuV2& menu, VariableParameters parameters) {
    printCursor(menu, PIP_ALARM_OPT, parameters);
    printCursor(menu, PEEP_ALARM_OPT, parameters);
    printCursor(menu, VT_ALARM_OPT, parameters);
}

void displaySettings(MenuV2& menu, VariableParameters parameters, SensorData sensorData) {
    printCursor(menu, FIL_OPT, parameters);
    printCursor(menu, AUTO_OPT, parameters);

    char temp[5];
    dtostrf(sensorData.cdyn * 1.01972, 2, 1, temp);
    writeLine(menu, 2, "CD:" + String(temp), 1);
}

void printMenu(MenuV2& menu, VariableParameters parameters, SensorData sensorData) {
    if ( menu.menuState.menu == MAIN ) {
        displayMainMenu(menu);
    } else if ( menu.menuState.menu == PARAMETER ) {
        if (!menu.menuState.setupReady) {
            displayInitialParametersSettings(menu, parameters);
        } else {
            displayParametersSettings(menu, parameters, sensorData);
        }
    } else if ( menu.menuState.menu == ALARM ) {
        displayAlarmSettings(menu, parameters);
    } else if ( menu.menuState.menu == SETTINGS ) {
        displaySettings(menu, parameters, sensorData);
    }
}

void displayMenu(MenuV2& menu, VariableParameters parameters, SensorData sensorData) {
    if (menu.menuState.changedMenu) {
        menu.lcd->clear();
        printMenu(menu, parameters, sensorData);
        menu.menuState.changedMenu = false;
    } else {
        DisplayValue previousValue = getValueToDisplay(menu.menuState.previousCursorCode, parameters, menu.menuState);
        clearCursor(menu, previousValue.cursorOffset, previousValue.line);
        printCursor(menu, menu.menuState.cursorCode, parameters);
    }
}

void checkEncoder(MenuV2& menu, VariableParameters& parameters, SensorData& sensorData, long time) {
    bool somethingChanged = menu.keyboardState.lastKeyPressedTime - time < 500;

    if (somethingChanged) {
        checkKeyboardActions(menu.keyboardState, menu.menuState, parameters);
        displayMenu(menu, parameters, sensorData);
    }
}

void setupMenu(MenuV2& menuV2, VariableParameters& parameters, SensorData& sensorData, long time) {
    menuV2.menuState.menu = PARAMETER;
    menuV2.menuState.cursorCode = INIT_PARAM_MENU[0];
    menuV2.menuState.changedMenu = false;
    menuV2.lcd->clear();
    displayInitialParametersSettings(menuV2, parameters);
    do {
        checkKeyboard(menuV2.keyboardState, time);
        bool somethingChanged = menuV2.keyboardState.lastKeyPressedTime - time < 500;
        if (somethingChanged) {
            checkKeyboardActionForSetup(menuV2.keyboardState, menuV2.menuState, parameters);
            displayMenu(menuV2, parameters, sensorData);
        }
        delay(150);
    } while (!menuV2.menuState.setupReady);
    menuV2.menuState.cursorCode = PARAM_MENU[0];
}