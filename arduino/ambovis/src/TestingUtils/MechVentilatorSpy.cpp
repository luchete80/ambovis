//
// Created by Mirian Quinteros on 08/08/2021.
//

#include "MechVentilatorSpy.h"

MechVentilatorSpy::MechVentilatorSpy(AccelStepper *_stepper, AutoPID *pid, VentilationOptions_t options) :
MechVentilation(_stepper, pid, options) {
    MechVentilatorToOutput mechVentilatorToOutput;
    this->toOutput = mechVentilatorToOutput;
}

void MechVentilatorSpy::update(void) {
    MechVentilatorToOutput::Values values = collectValues();
    values.method = "MechVentilatorSpy::update";
    this->toOutput.recordValues(values);
    MechVentilation::update();
}

void MechVentilatorSpy::change_config(VentilationOptions_t ventilationOptions) {
    MechVentilation::change_config(ventilationOptions);
    MechVentilatorToOutput::Values values = collectValues();
    values.method = "MechVentilatorSpy::change_config";
    this->toOutput.recordValues(values);
}

void MechVentilatorSpy::stop() {
    MechVentilation::stop();
    MechVentilatorToOutput::Values values = collectValues();
    values.method = "MechVentilatorSpy::stop";
    this->toOutput.recordValues(values);
}

MechVentilatorToOutput::Values MechVentilatorSpy::collectValues() {
    MechVentilatorToOutput::Values values;
    values.method = "not_defined";
    values.state = this->getState();
    values.respiratoryRate = this->getRPM();
    values.percInspEsp = this->getIERel();
    values.cycle_position = this->getCycleNum();
    values.msecTimerCnt = this->getMSecTimerCnt();
    return values;
}