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

bool debounce(bool last, int pin) {
    bool current = digitalRead(pin);
    if (last != current) {
        delay(50);
        current = digitalRead(pin);
    }
    return current;
}

bool check_buzzer_mute(bool last_mute, bool buzzmuted, unsigned long mute_count, unsigned long time)  {
    bool curr_mute = debounce(last_mute, PIN_MUTE);
    if (!buzzmuted && last_mute == HIGH && curr_mute == LOW) {
        mute_count = time;
        buzzmuted = true;
    }
    last_mute = curr_mute;
    if (buzzmuted && (time > mute_count + TIME_MUTE)) { //each count is every 500 ms
        buzzmuted = false;
    }
    return buzzmuted;
}

void set_alarm_buzzer(short alarm_state, bool buzzmuted, unsigned long& timebuzz, bool& isbuzzeron) {
    //            if (alarm_state > 0) {
    //                if (!buzzmuted) {
    //                    if (millis() > timebuzz + TIME_BUZZER) {
    //                        timebuzz=millis();
    //                        isbuzzeron=!isbuzzeron;
    //                        if (isbuzzeron) {
    //                            digitalWrite(PIN_BUZZER,BUZZER_LOW);
    //                        } else {
    //                            digitalWrite(PIN_BUZZER,BUZZER_HIGH);
    //                        }
    //                    }
    //                } else {  //buzz muted
    //                    digitalWrite(PIN_BUZZER,!BUZZER_LOW);
    //                }
    //            } else {//state > 0
    //                digitalWrite(PIN_BUZZER,!BUZZER_LOW);
    //                isbuzzeron=true;        //Inverted logic
    //            }
}