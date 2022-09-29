//
// Created by Mirian Quinteros on 28/08/2022.
//

#ifndef AMBOVIS_ALARMS_H
#define AMBOVIS_ALARMS_H

#include "Arduino.h"
#include "pinout.h"
#include "defaults.h"

short getAlarmState(float last_pressure_max, float last_pressure_min, short alarm_max_pressure, short alarm_peep_pressure);
void check_buzzer_mute(bool& last_mute, unsigned long& mute_count_time, bool& buzzmuted, unsigned long time);

#endif //AMBOVIS_ALARMS_H
