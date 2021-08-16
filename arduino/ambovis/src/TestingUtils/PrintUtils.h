//
// Created by Mirian Quinteros on 15/08/2021.
//

#ifndef AMBOVIS_PRINTUTILS_H
#define AMBOVIS_PRINTUTILS_H

char* floatToStr(float floatValue) {
    char tempStr[5];
    dtostrf(floatValue, 2, 0, tempStr);
    return *tempStr;
}

#endif //AMBOVIS_PRINTUTILS_H
