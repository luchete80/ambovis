//
// Created by Mirian Quinteros on 28/08/2022.
//

#include "alarms.h"

short getAlarmState(bool is_alarm_vt_on, float last_pressure_max, float last_pressure_min, byte alarm_max_pressure, byte alarm_peep_pressure) {
    short alarm_state;
    if ( last_pressure_max > alarm_max_pressure + 1 ) {
        if ( last_pressure_min < alarm_peep_pressure - 1) {
            if (!is_alarm_vt_on) {
                alarm_state = 3;
            } else {
                alarm_state = 13;
            }
        } else {
            if (!is_alarm_vt_on) {
                alarm_state = 2;
            } else {
                alarm_state = 12;
            }
        }
    } else {
        if ( last_pressure_min < alarm_peep_pressure - 1 ) {
            if (!is_alarm_vt_on) {
                alarm_state = 1;
            } else {
                alarm_state = 11;
            }
        } else {
            if (!is_alarm_vt_on) {
                alarm_state = 0;
            } else {
                alarm_state = 10;
            }
        }
    }
    return alarm_state;
}
