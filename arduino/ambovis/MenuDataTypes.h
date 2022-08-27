//
// Created by Mirian Quinteros on 28/06/2022.
//

#ifndef AMBOVIS_MENUDATATYPES_H
#define AMBOVIS_MENUDATATYPES_H

#include "Arduino.h"
#include "MenuConstants.h"
#include "pinout.h"

typedef struct keyboard_state {
    int lastUpState = 0;
    int lastDownState = 0;
    int lastOKState = 0;
    int lastBackState = 0;
    int previousCount = 0;
    int count = 0;
    bool ok = false;
    bool back = false;
    unsigned long lastKeyPressedTime = 0;
    unsigned long backHoldTime = 0;
} KeyboardState;

typedef struct menu_state {
    int menu = 0;
    int editedParameter = 0;
    int cursorCode = 0;
    int previousCursorCode = 0;
    bool isEditingParam = false;
    bool changedMenu = false;
    bool setupReady = false;
    bool updatedOptions = false;
} MenuState;

typedef struct variable_parameters {
    int vent_mode;
    int alarm_max_pressure;
    int respiratoryRate;
    int alarm_peep_pressure;
    int percInspEsp;
    int alarm_vt;
    int peakInspiratoryPressure;
    int percVolume;
    int autopid;
    int filter;
} VariableParameters;

typedef struct sensor_data {
    float last_pressure_min;
    float last_pressure_max;
    int _mlLastInsVol;
    int _mlLastExsVol;
    int cdyn;
} SensorData;

static int SIZE_MENU = 4;
static int MAIN_MENU[] = {PARAMETER, ALARM, SETTINGS};
static int INIT_PARAM_MENU[] = {BPM_OPT, IE_OPT, END_SETUP};
static int PARAM_MENU[] = {PERC_V_OPT, BPM_OPT, IE_OPT};
static int ALARM_MENU[] = {PIP_ALARM_OPT, PEEP_ALARM_OPT, VT_ALARM_OPT};
static int SETTINGS_MENU[] = {FIL_OPT, AUTO_OPT};

#endif //AMBOVIS_MENUDATATYPES_H
