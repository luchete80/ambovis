//
// Created by Mirian Quinteros on 11/08/2021.
//

#ifndef AMBOVIS_MENUDISPLAYTOOUTPUT_H
#define AMBOVIS_MENUDISPLAYTOOUTPUT_H
#include "Arduino.h"

class MenuDisplayToOutput {

public:
    typedef struct {
        String functionName;
        float last_pressure_max;
        float last_pressure_min;
        unsigned int timeoutIns;
        unsigned int timeoutEsp;
        float pf_min; //description :
        float pf_max;
        byte p_acc;
        int min_cd;
        int max_cd;
        int min_speed;
        int max_speed;
        int min_accel;
        int max_accel;
        int min_pidk;
        int max_pidk;
        int min_pidi;
        int max_pidi;
        int min_pidd;
        int max_pidd;
    } Params;

    MenuDisplayToOutput();
    void print( Params params );

private:
    String titlesLine = "method, last pressure max, last pressure min, timeoutIns, timeoutExp, pf_min, pf_max, p_acc, min_cd, max_cd, min_speed, max_speed, min_accel, max_accel, min_pidk, max_pidk, min_pidi, max_pidi, min_pidd, max_pidd";

};


#endif //AMBOVIS_MENUDISPLAYTOOUTPUT_H
