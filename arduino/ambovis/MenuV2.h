#ifndef _MENU_V2_H_
#define _MENU_V2_H_

#include "defaults.h"
#include <LiquidCrystal.h>
#include "MenuKeyboard.h"
#include "MenuConstants.h"

typedef struct {
    LiquidCrystal lcd;
    int currentSelection;
    bool updateDisplay;
    byte menuNumber;
    MenuKeyboard menuKeyboard;
} Menu;

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
} VariableParameters;

void writeLine(Menu& menu, int line, String message = "", int offsetLeft = 0);
void clearDigits(Menu& menu, int x, int y, int pos=1);
void selectDigit(Menu& menu, int x, int y);
void checkEncoder(Menu& menu, SystemState& systemState, VariableParameters& parameters);
void display(Menu& menu, SystemState& systemState, bool shouldClearDisplay);
void initDisplay(Menu& menu);

#endif
