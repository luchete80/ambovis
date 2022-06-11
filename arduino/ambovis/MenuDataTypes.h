//
// Created by Mirian Quinteros on 27/03/2022.
//

#ifndef AMBOVIS_MENUDATATYPES_H
#define AMBOVIS_MENUDATATYPES_H

#include "MenuConstants.h"
#include "pinout.h"
//#include "Arduino.h"

typedef struct {
    int count;
    bool ok;
    bool back;
    unsigned long lastKeyPressedTime;
} KeyboardState;

typedef struct {
    int x;
    int y;
    int off;
} Pos;

typedef struct {
    char label[10];
    int code;
    Pos cursor;
    Pos value;
} CursorDisplay;

typedef struct {
    int code;
    int min;
    int max;
    CursorDisplay cursorDisplay;
} Cursor;

typedef struct {
    int menu;
    int editedParameter;
    Cursor cursor;
    bool isEditingParam;
    bool changedMenu;
} MenuState;

typedef struct {
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

static CursorDisplay CD_PARAMS = {"Params", PARAMETER,1, 1, 0,0, 0, 0};
static CursorDisplay CD_ALARMS = {"Alarms", ALARM,1, 2, 0, 0, 0, 0};
static CursorDisplay CD_SETTINGS = {"Settings", SETTINGS, 1, 3, 0, 0, 0, 0};
static CursorDisplay CD_PID_SETTINGS = {"PID settings", PID_SETTINGS,1, 4, 0,0, 0, 0};

static Cursor C_PARAMS = { PARAMETER, 0, 3, CD_PARAMS};
static Cursor C_ALARMS = {ALARM,0, 4, CD_ALARMS};
static Cursor C_SETTINGS = {SETTINGS,0, 3, CD_SETTINGS};
static Cursor C_PID_SETTINGS = {PID_SETTINGS,0, 4, CD_PID_SETTINGS};

static CursorDisplay CD_P1 = {"MOD", MODE_OPT, 1, 1, 0,4, 1, 0};
static CursorDisplay CD_P2 = {"V", PERC_V_OPT, 6, 1, 0,10, 1, 0};
static CursorDisplay CD_P3 = {"BPM", BPM_OPT, 1, 2, 0,4, 2, 0};
static CursorDisplay CD_P4 = {"IE:1", IE_OPT, 6, 2, 0,10, 2, 0};
static CursorDisplay CD_P5 = {"PIP", PIP_OPT, 6, 1, 0,10, 1, 0};

static Cursor C_P1 = {MODE_OPT,0, 2, CD_P1};
static Cursor C_P2 = {PERC_V_OPT,450, 500, CD_P2};
static Cursor C_P3 = {BPM_OPT,5, 10, CD_P3};
static Cursor C_P4 = {IE_OPT,5, 10, CD_P4};
static Cursor C_P5 = {PIP_OPT,5, 10, CD_P5};

static CursorDisplay CD_AL1 = {"PIPAL", PIP_ALARM_OPT, 1, 1, 1,1, 5, 0};
static CursorDisplay CD_AL2 = {"PEEPAL",PEEP_ALARM_OPT, 2, 1, 1,2, 6, 0};
static CursorDisplay CD_AL3 = {"VTAL", VT_ALARM_OPT, 2, 1, 1,2, 6, 0};

static Cursor C_AL1 = {PIP_ALARM_OPT,900, 1010, CD_AL1};
static Cursor C_AL2 = {PEEP_ALARM_OPT,900, 1010, CD_AL2};
static Cursor C_AL3 = {VT_ALARM_OPT, 900, 1010, CD_AL3};

static CursorDisplay CD_S1 = {"TRIM", TRIM_OPT, 1, 1, 0,1, 4, 0};
static CursorDisplay CD_S2 = {"FIL", FIL_OPT, 2, 1, 0,2, 6, 0};
static CursorDisplay CD_S3 = {"AUTO", AUTO_OPT, 2, 1, 0,2, 6, 0};
static CursorDisplay CD_S4 = {"CD", CD_OPT, 2, 1, 0,2, 6, 0};

static Cursor C_S1 = {TRIM_OPT,20, 30, CD_S1};
static Cursor C_S2 = {FIL_OPT,10, 50, CD_S2};
static Cursor C_S3 = {AUTO_OPT,10, 50, CD_S3};
static Cursor C_S4 = {CD_OPT,10, 50, CD_S4};

static CursorDisplay CD_PS1 = {"DP",DP_OPT, 2, 1, 1,2, 6, 0};
static CursorDisplay CD_PS2 = {"f", F_OPT, 2, 1, 1,2, 6, 0};
static CursorDisplay CD_PS3 = {"F", FF_OPT, 2, 1, 1,2, 6, 0};
static CursorDisplay CD_PS4 = {"Pa", PA_OPT, 2, 1, 1,2, 6, 0};
static CursorDisplay CD_PS5 = {"Fa",FA_OPT, 2, 1, 1,2, 6, 0};

static Cursor C_PS1 = {DP_OPT,10, 50, CD_PS1};
static Cursor C_PS2 = {F_OPT,10, 50, CD_PS2};
static Cursor C_PS3 = {FF_OPT,10, 50, CD_PS3};
static Cursor C_PS4 = {PA_OPT,10, 50, CD_PS4};
static Cursor C_PS5 = {FA_OPT,10, 50, CD_PS5};

static int SIZE_MENU = 4;
static Cursor MAIN_MENU[] = {C_PARAMS, C_ALARMS, C_SETTINGS, C_PID_SETTINGS};
static Cursor PARAM_MENU[] = {C_P1, C_P2, C_P3, C_P4};
static Cursor PARAM_MENU_PCV[] = {C_P1, C_P5, C_P3, C_P4};
static Cursor ALARM_MENU[] = {C_AL1, C_AL2, C_AL3};
static Cursor SETTINGS_MENU[] = {C_S1, C_S2, C_S3, C_S4};
static Cursor PID_SETTINGS_MENU[] = {C_PS1, C_PS2, C_PS3, C_PS4, C_PS5};

#endif //AMBOVIS_MENUDATATYPES_H
