#ifndef _MENU_V2_H_
#define _MENU_V2_H_

#include "defaults.h"
#include <LiquidCrystal.h>
#include "MenuDataTypes.h"
#include "MenuKeyboard.h"
#include "MenuSelector.h"

typedef struct {
    LiquidCrystal lcd;
    KeyboardState keyboardState;
    MenuState menuState;
} Menu;

void initDisplay(Menu& menu);
void writeLine(Menu& menu, int line, String message = "", int offsetLeft = 0);
void checkEncoder(Menu& menu, VariableParameters& parameters, long time);
void printMenu(Menu& menu, VariableParameters& parameters);
#endif
