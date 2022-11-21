//
// Created by Mirian Quinteros on 28/08/2022.
//

#include "alarms.h"

short get_alarm_state(float last_pressure_max, float last_pressure_min, short alarm_max_pressure, short alarm_peep_pressure) {
    if ( last_pressure_max > alarm_max_pressure + 1 ) {
        if ( last_pressure_min < alarm_peep_pressure - 1 ) {
            return 3;
        } else {
            return 2;
        }
    } else {
        if ( last_pressure_min < alarm_peep_pressure - 1 ) {
            return 1;
        } else {
            return 0;
        }
    }
}

bool calc_alarm_vt_is_on(int ml_last_ins_vol, int ml_last_exp_vol, int alarm_vt_limit) {
    int vt = (ml_last_ins_vol + ml_last_exp_vol) / 2;
    return vt < alarm_vt_limit;
}
