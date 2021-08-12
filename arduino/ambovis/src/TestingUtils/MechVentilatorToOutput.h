//
// Created by Mirian Quinteros on 07/08/2021.
//

#ifndef AMBOVIS_MECHVENTILATORTOOUTPUT_H
#define AMBOVIS_MECHVENTILATORTOOUTPUT_H
#include "Arduino.h"
#include "string.h"
#include <stddef.h>
#include <stdint.h>
#include "SerialValuePrinter.h"

using namespace std;

class MechVentilatorToOutput {

public:
    typedef struct {
        String method;
        short respiratoryRate;
        byte percInspEsp;
        short inputPip;
        String state;
        unsigned int cycle_position;
        unsigned long msecTimerCnt;
    } Values;

    MechVentilatorToOutput();
    void recordValues(Values& valuesToRecord);
    String getTitlesLine();

private:
    SerialValuePrinter valuePrinter;
    String titlesLine = "method,respiratoryRate, percInspEsp, inputPip, state, cycle_position, msecTimerCnt \n";

};


#endif //AMBOVIS_MECHVENTILATORTOOUTPUT_H
