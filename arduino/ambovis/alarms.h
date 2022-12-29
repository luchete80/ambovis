//
// Created by Mirian Quinteros on 28/08/2022.
//

#ifndef AMBOVIS_ALARMS_H
#define AMBOVIS_ALARMS_H

#include "Arduino.h"
#include "pinout.h"

short get_alarm_state(float last_pressure_max, float last_pressure_min, short alarm_max_pressure, short alarm_peep_pressure);
bool calc_alarm_vt_is_on(int ml_last_ins_vol, int ml_last_exp_vol, int alarm_vt_limit);
bool check_buzzer_mute(bool last_mute, bool buzzmuted, unsigned long mute_count, unsigned long time);
void set_alarm_buzzer(short alarm_state, bool buzzmuted, unsigned long& timebuzz, bool& isbuzzeron);

#endif //AMBOVIS_ALARMS_H
