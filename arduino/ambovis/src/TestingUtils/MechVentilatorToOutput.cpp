//
// Created by Mirian Quinteros on 07/08/2021.
//

#include "MechVentilatorToOutput.h"

MechVentilatorToOutput::MechVentilatorToOutput() {
    Serial.print(this->titlesLine);
}

void MechVentilatorToOutput::recordValues(Values& valuesToRecord) {
    String str;
    str += valuesToRecord.method;
    str += ",";
    str += valuesToRecord.pressure_p;
    str += ",";
    str += valuesToRecord.adc0;
    str += ",";
    str += valuesToRecord.respiratoryRate;
    str += ",";
    str += valuesToRecord.percInspEsp;
    str += ",";
    str += valuesToRecord.inputPip;
    str += ",";
    str += valuesToRecord.state;
    str += ",";
    str += valuesToRecord.cycle_position;
    str += ",";
    str += valuesToRecord.msecTimerCnt;

    Serial.println(str);
}
