//
// Created by Mirian Quinteros on 27/02/2022.
//

#ifndef ALARMVALIDATION_H
#define ALARMVALIDATION_H

byte getAlarmState(int vt, int alarm_vt, float last_pressure_max, float last_pressure_min, byte alarm_max_pressure, byte alarm_peep_pressure) {
    byte isalarmvt_on = vt < alarm_vt ? 1 : 0;
    byte alarm_state;
    if ( last_pressure_max > alarm_max_pressure + 1 ) {
        if ( last_pressure_min < alarm_peep_pressure - 1) {
            if (!isalarmvt_on)  alarm_state = 3;
            else                alarm_state = 13;
        } else {
            if (!isalarmvt_on)  alarm_state = 2;
            else                alarm_state = 12;
        }
    } else {
        if ( last_pressure_min < alarm_peep_pressure - 1 ) {
            if (!isalarmvt_on) alarm_state = 1;
            else               alarm_state = 11;
        } else {
            if (!isalarmvt_on)  alarm_state = 0;
            else                alarm_state = 10;
        }
    }
    return alarm_state;
}
#endif //ALARMVALIDATION_H
