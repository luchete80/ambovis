//
// Created by Mirian Quinteros on 28/06/2022.
//

#ifndef _MENU_V2_H_
#define _MENU_V2_H_

#include "defaults.h"
#ifdef LCD_I2C
#include "src/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#else
#include <LiquidCrystal.h>
#endif
#include "MenuDataTypes.h"
#include "MenuKeyboard.h"
#include "MenuSelector.h"

typedef struct menu_v2 {
#ifdef LCD_I2C
    LiquidCrystal_I2C *lcd;
#else
    LiquidCrystal *lcd;
#endif
    KeyboardState keyboardState;
    MenuState menuState;
} MenuV2;

void initDisplay(MenuV2& menu);
void setupMenu(MenuV2& menu, VariableParameters& parameters, long time);
void writeLine(MenuV2& menu, int line, String message = "", int offsetLeft = 0);
void checkEncoder(MenuV2& menu, VariableParameters& parameters, long time);
void printMenu(MenuV2& menu, VariableParameters& parameters);
#endif