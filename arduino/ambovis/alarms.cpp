//
// Created by Mirian Quinteros on 28/08/2022.
//

#include "alarms.h"

void show_power_led() {
    if (!digitalRead(PIN_POWEROFF)) {
        digitalWrite(YELLOW_LED, HIGH);
    } else {
        digitalWrite(YELLOW_LED, LOW);
    }
}

short get_alarm_state(
        float last_pressure_max,
        float last_pressure_min,
        byte alarm_max_pressure,
        byte alarm_peep_pressure
        ) {
    if ( last_pressure_max > alarm_max_pressure + 1 ) {
        return last_pressure_min < alarm_peep_pressure - 1 ? PEEP_PIP_ALARM : PIP_ALARM;
    } else {
        return last_pressure_min < alarm_peep_pressure - 1 ? PEEP_ALARM : NO_ALARM;
    }
}

bool calc_alarm_vt_is_on(
        int ml_last_ins_vol,
        int ml_last_exp_vol,
        int alarm_vt_limit
        ) {
    int vt = (ml_last_ins_vol + ml_last_exp_vol) / 2;
    return vt < alarm_vt_limit;
}

AlarmData& check_alarms(Ventilation_Status_t vent_status, AlarmData& alarm_data) {
    alarm_data.alarm_state = get_alarm_state(vent_status.last_max_pressure, vent_status.last_min_pressure,
                                         alarm_data.alarm_max_pressure, alarm_data.alarm_peep_pressure);
    alarm_data.is_alarm_vt_on = calc_alarm_vt_is_on(vent_status.ml_last_ins_vol, vent_status.ml_last_exp_vol, alarm_data.alarm_vt);
    return alarm_data;
}

bool debounce(bool last, int pin) {
    bool current = digitalRead(pin);
    if (last != current) {
        delay(50);
        current = digitalRead(pin);
    }
    return current;
}

Buzzer_State_t& check_buzzer_mute(
        Buzzer_State_t& buzzer_state,
        unsigned long time
        ) {
    bool curr_mute = debounce(buzzer_state.last_mute, PIN_MUTE);
    if (!buzzer_state.buzz_muted && buzzer_state.last_mute == HIGH && curr_mute == LOW) {
        buzzer_state.mute_count = time;
        buzzer_state.buzz_muted = true;
    }
    buzzer_state.last_mute = curr_mute;
    if (buzzer_state.buzz_muted && (time > buzzer_state.mute_count + TIME_MUTE)) { //each count is every 500 ms
        buzzer_state.buzz_muted = false;
    }
    return buzzer_state;
}

Buzzer_State_t& set_alarm_buzzer(short alarm_state, Buzzer_State_t& buzzer_state) {
//                if (alarm_state > 0) {
//                    if (!buzzer_state.buzz_muted) {
//                        if (millis() > buzzer_state.time_buzz + TIME_BUZZER) {
//                            buzzer_state.time_buzz=millis();
//                            buzzer_state.is_buzzer_on=!buzzer_state.is_buzzer_on;
//                            if (buzzer_state.is_buzzer_on) {
//                                digitalWrite(PIN_BUZZER,BUZZER_LOW);
//                            } else {
//                                digitalWrite(PIN_BUZZER,BUZZER_HIGH);
//                            }
//                        }
//                    } else {  //buzz muted
//                        digitalWrite(PIN_BUZZER,!BUZZER_LOW);
//                    }
//                } else {//state > 0
//                    digitalWrite(PIN_BUZZER,!BUZZER_LOW);
//                    buzzer_state.is_buzzer_on=true;        //Inverted logic
//                }
    return buzzer_state;
}