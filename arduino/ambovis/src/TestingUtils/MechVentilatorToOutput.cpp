//
// Created by Mirian Quinteros on 07/08/2021.
//

#include "MechVentilatorToOutput.h"

MechVentilatorToOutput::MechVentilatorToOutput() {
    this->valuePrinter.print(this->getTitlesLine());
}

void MechVentilatorToOutput::recordValues(Values& valuesToRecord) {
    String str;
    str += valuesToRecord.method;
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
    str += "\n";
    this->valuePrinter.print(str);
}

String MechVentilatorToOutput::getTitlesLine() {
    return this->titlesLine;
}
