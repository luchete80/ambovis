//
// Created by Mirian Quinteros on 28/06/2022.
//

#ifndef MENUSELECTOR_H
#define MENUSELECTOR_H

#include "Arduino.h"
#include "defaults.h"
#include "MenuDataTypes.h"

void checkKeyboardActions(KeyboardState& keyboardState, MenuState& menuState, VariableParameters& variables);
void checkKeyboardActionForSetup(KeyboardState& keyboardState, MenuState& menuState, VariableParameters& variables);

#endif //MENUSELECTOR_H
