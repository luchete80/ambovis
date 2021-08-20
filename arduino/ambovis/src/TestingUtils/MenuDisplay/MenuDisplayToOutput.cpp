//
// Created by Mirian Quinteros on 16/08/2021.
//

#include "MenuDisplayToOutput.h"
#include "../PrintUtils.h"

MenuDisplayToOutput::MenuDisplayToOutput() {
    //Serial.print(this->titlesLine);	//This is printing garbage and then crashing 
}

void MenuDisplayToOutput::print(Params params) {
    String str = params.functionName;
    str += ",";
    str += floatToStr(params.last_pressure_max);
    str += ",";
    str += floatToStr(params.last_pressure_min);
    str += ",";
    str += params.timeoutIns;
    str += ",";
    str += params.timeoutEsp;
    str += ",";
    str += params.pf_min;
    str += ",";
    str += params.pf_max;
    str += ",";
    str += params.p_acc;
    str += ",";
    str += params.min_cd;
    str += ",";
    str += params.max_cd;
    str += ",";
    str += params.min_speed;
    str += ",";
    str += params.max_speed;
    str += ",";
    str += params.min_accel;
    str += ",";
    str += params.max_accel;
    str += ",";
    str += params.min_pidk;
    str += ",";
    str += params.max_pidk;
    str += ",";
    str += params.min_pidi;
    str += ",";
    str += params.max_pidi;
    str += ",";
    str += params.min_pidd;
    str += ",";
    str += params.max_pidd;

    Serial.println(str);
}