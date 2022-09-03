//
// Created by Mirian Quinteros on 02/09/2022.
//

#ifndef AMBOVIS_INITIALACTIONS_H
#define AMBOVIS_INITIALACTIONS_H

#include <Arduino.h>
#include "defaults.h"
#include "pinout.h"
#include "src/AccelStepper/AccelStepper.h"

void initPins();
void waitForFluxDisconnected();
void searchHomePosition(AccelStepper* stepper);

#endif //AMBOVIS_INITIALACTIONS_H
