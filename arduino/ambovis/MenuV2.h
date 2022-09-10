//
// Created by Mirian Quinteros on 28/06/2022.
//

#ifndef _MENU_V2_H_
#define _MENU_V2_H_

#include "defaults.h"
#include <LiquidCrystal.h>
#include "MenuDataTypes.h"
#include "MenuKeyboard.h"
#include "MenuSelector.h"

typedef struct menu_v2 {
    LiquidCrystal *lcd;
    KeyboardState keyboardState;
    MenuState menuState;
} MenuV2;

void initDisplay(MenuV2& menu);
void setupMenu(MenuV2& menu, VariableParameters& parameters, SensorData& sensorData, long time);
void writeLine(MenuV2& menu, int line, String message = "", int offsetLeft = 0);
void checkEncoder(MenuV2& menu, VariableParameters& parameters, SensorData& sensorData, long time);
void printMenu(MenuV2& menu, VariableParameters parameters, SensorData sensorData, long time);
#endif