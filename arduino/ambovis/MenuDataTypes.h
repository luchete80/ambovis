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
    char codeName[4];
    int code;
    Pos start;
    Pos valuePos;
    int min;
    int max;
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
    int stepperAccelMax;
    int dpip_b;
    float dpip;
    int respiratoryRate;
    int alarm_peep_pressure;
    int stepperSpeedMax;
    int pfmin;
    float pf_min;
    float peep_fac;
    int percInspEsp;
    int alarm_vt;
    int min_accel;
    int pfmax;
    float pf_max;
    int peakInspiratoryPressure;
    int percVolume;
    int p_trim;
    int max_cd;
    int p_acc;
    int autopid;
    int max_speed;
    int f_acc_b;
    int f_acc;
    int peakEspiratoryPressure;
    bool filter;
    int max_accel;
    int min_pidk;
    int min_pidi;
    int min_pidd;
    int max_pidk;
    int max_pidi;
    int max_pidd;
    // for menu TBD
    int cd_opt;
    int dp_opt;
    int ff_opt;
    int fa_opt;
    int pa_opt;

} VariableParameters;

static Cursor C_PARAMS = { "PAR", PARAMETER,1, 1, 0,
                           0, 0, 0,
                           0, 3};
static Cursor C_ALARMS = {"ALR", ALARM,1, 2, 0,
                          0, 0, 0,
                          0, 4};
static Cursor C_SETTINGS = {"SET", SETTINGS,1, 3, 0,
                            0, 0, 0,
                            0, 3};
static Cursor C_PID_SETTINGS = {"PST", PID_SETTINGS,
                                1, 4, 0,
                                0, 0, 0,
                                0, 4};

static Cursor C_P1 = {"P1", MODE_OPT, 1, 1, 0,
                      4, 1, 0,
                      0, 2};
static Cursor C_P2 = {"P2", PERC_V_OPT, 6, 1, 0,
                      10, 1, 0,
                      450, 500};
static Cursor C_P3 = {"P3", BPM_OPT, 1, 2, 0,
                      4, 2, 0,
                      5, 10};
static Cursor C_P4 = {"P4", IE_OPT, 6, 2, 0,
                      10, 2, 0,
                      5, 10};
static Cursor C_P5 = {"P5", PIP_OPT, 6, 1, 0,
                      10, 1, 0,
                      5, 10};

static Cursor C_AL1 = {"AL1", PIP_ALARM_OPT, 1, 1, 1,
                       1, 5, 0,
                       900, 1010};
static Cursor C_AL2 = {"AL2", PEEP_ALARM_OPT, 2, 1, 1,
                       2, 6, 0,
                       900, 1010};
static Cursor C_AL3 = {"AL3", VT_ALARM_OPT, 2, 1, 1,
                       2, 6, 0,
                       900, 1010};
static Cursor C_AL4 = {"AL4", VM_ALARM_OPT, 2, 1, 1,
                       2, 6, 0,
                       900, 1010};

static Cursor C_AL5 = {"AL5", AMBU_ALARM_OPT, 2, 1, 1,
                       2, 6, 0,
                       900, 1010};

static Cursor C_S1 = {"S1", TRIM_OPT, 1, 1, 0,
                      1, 4, 0,
                      20, 30};
static Cursor C_S2 = {"S2", FIL_OPT, 2, 1, 0,
                      2, 6, 0,
                      10, 50};
static Cursor C_S3 = {"S3", AUTO_OPT, 2, 1, 0,
                      2, 6, 0,
                      10, 50};
static Cursor C_S4 = {"S4", CD_OPT, 2, 1, 0,
                      2, 6, 0,
                      10, 50};

static Cursor C_PS1 = {"PS1", DP_OPT, 2, 1, 1,
                       2, 6, 0,
                       10, 50};
static Cursor C_PS2 = {"PS2", F_OPT, 2, 1, 1,
                       2, 6, 0,
                       10, 50};
static Cursor C_PS3 = {"PS3", FF_OPT, 2, 1, 1,
                       2, 6, 0,
                       10, 50};
static Cursor C_PS4 = {"PS4", PA_OPT, 2, 1, 1,
                       2, 6, 0,
                       10, 50};
static Cursor C_PS5 = {"PS5", FA_OPT, 2, 1, 1,
                       2, 6, 0,
                       10, 50};

static int SIZE_MENU = 4;
static Cursor MAIN_MENU[] = {C_PARAMS, C_ALARMS, C_SETTINGS, C_PID_SETTINGS};
static Cursor PARAM_MENU[] = {C_P1, C_P2, C_P3, C_P4};
static Cursor PARAM_MENU_PCV[] = {C_P1, C_P5, C_P3, C_P4};
static Cursor ALARM_MENU[] = {C_AL1, C_AL2, C_AL3, C_AL4, C_AL5};
static Cursor SETTINGS_MENU[] = {C_S1, C_S2, C_S3, C_S4};
static Cursor PID_SETTINGS_MENU[] = {C_PS1, C_PS2, C_PS3, C_PS4, C_PS5};

#endif //AMBOVIS_MENUDATATYPES_H
