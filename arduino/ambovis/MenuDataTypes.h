//
// Created by Mirian Quinteros on 28/06/2022.
//

#ifndef AMBOVIS_MENUDATATYPES_H
#define AMBOVIS_MENUDATATYPES_H

#include "MenuConstants.h"
#include "pinout.h"

typedef struct keyboard_state {
    int previousCount = 0;
    int count = 0;
    bool ok = false;
    bool back = false;
    unsigned long lastKeyPressedTime = 0;
} KeyboardState;

typedef struct menu_state {
    int menu = 0;
    int editedParameter = 0;
    int cursorCode = 0;
    int previousCursorCode = 0;
    bool isEditingParam = false;
    bool changedMenu = false;
} MenuState;

typedef struct variable_parameters{
    int vent_mode;
    int alarm_max_pressure;
    int respiratoryRate;
    int alarm_peep_pressure;
    int percInspEsp;
    int alarm_vt;
    int peakInspiratoryPressure;
    int percVolume;
    int p_trim;
    int autopid;
    int fil;
    int max_accel;
    // for menu TBD
    int cd_opt;
    int dp_opt;
    int f_min;
    int f_max;
    int fa_opt;
    int pa_opt;

    // sensor values
    float last_pressure_min;
    int _mllastInsVol;
    int _mllastExsVol;

} VariableParameters;

static int SIZE_MENU = 4;
static int MAIN_MENU[] = {PARAMETER, ALARM, SETTINGS, PID_SETTINGS};
static int PARAM_MENU[] = {MODE_OPT, PERC_V_OPT, BPM_OPT, IE_OPT};
static int PARAM_MENU_PCV[] = {MODE_OPT, PIP_OPT, BPM_OPT, IE_OPT};
static int ALARM_MENU[] = {PIP_ALARM_OPT, PEEP_ALARM_OPT, VT_ALARM_OPT};
static int SETTINGS_MENU[] = {TRIM_OPT, FIL_OPT, AUTO_OPT, CD_OPT};
static int PID_SETTINGS_MENU[] = {DP_OPT, F_OPT, FF_OPT, PA_OPT, FA_OPT};

#endif //AMBOVIS_MENUDATATYPES_H
