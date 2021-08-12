//
// Created by Mirian Quinteros on 08/08/2021.
//

#ifndef AMBOVIS_SERIALVALUEPRINTER_H
#define AMBOVIS_SERIALVALUEPRINTER_H
#include "string.h"

class SerialValuePrinter{
public:
    void print(String str) {
        Serial.print(str);
    }
};


#endif //AMBOVIS_SERIALVALUEPRINTER_H
