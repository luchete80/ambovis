//
// Created by Mirian Quinteros on 07/08/2021.
//

#ifndef AMBOVIS_MECHVENTILATORTOOUTPUT_H
#define AMBOVIS_MECHVENTILATORTOOUTPUT_H
#include "Arduino.h"

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
        float pressure_p;
        int16_t adc0;
    } Values;

    MechVentilatorToOutput();
    void recordValues(Values& valuesToRecord);

private:
    String titlesLine = "method, pressure, adc0, respiratoryRate, percInspEsp, inputPip, state, cycle_position, msecTimerCnt \n";

};


#endif //AMBOVIS_MECHVENTILATORTOOUTPUT_H
