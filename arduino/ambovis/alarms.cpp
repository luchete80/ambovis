//
// Created by Mirian Quinteros on 28/08/2022.
//

#include "alarms.h"

bool debounce(bool last) {
    bool current = digitalRead(PIN_MUTE);
    if (last != current) {
        delay(50);
        current = digitalRead(PIN_MUTE);
    }
    return current;
}

void check_buzzer_mute(bool& last_mute, unsigned long& mute_count_time, bool& buzzmuted, unsigned long time) {
    bool curr_mute = debounce(last_mute);
    if (last_mute == HIGH && curr_mute == LOW && !buzzmuted) {
        mute_count_time = time;
        buzzmuted = true;
    }
    last_mute = curr_mute;
    if (buzzmuted) {
        if (time > mute_count_time + TIME_MUTE) {
            buzzmuted = false;
        }
    }
}

short getAlarmState(float last_pressure_max, float last_pressure_min, short alarm_max_pressure, short alarm_peep_pressure) {
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
