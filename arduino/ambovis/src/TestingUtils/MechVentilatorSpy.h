//
// Created by Mirian Quinteros on 08/08/2021.
//

#ifndef AMBOVIS_MECHVENTILATORSPY_H
#define AMBOVIS_MECHVENTILATORSPY_H
#include "../../MechVentilation.h"
#include "MechVentilatorToOutput.h"
#include "SerialValuePrinter.h"

class MechVentilatorSpy: public MechVentilation {
public:
    MechVentilatorSpy(AccelStepper *_stepper,
                      AutoPID *pid,
                      VentilationOptions_t options);
    void update(void);
    void change_config(VentilationOptions_t ventilationOptions);
    void stop();
private:
    MechVentilatorToOutput::Values collectValues();
    MechVentilatorToOutput toOutput;
};


#endif //AMBOVIS_MECHVENTILATORSPY_H
