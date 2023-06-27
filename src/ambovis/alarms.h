//
// Created by Mirian Quinteros on 28/08/2022.
//

#ifndef AMBOVIS_ALARMS_H
#define AMBOVIS_ALARMS_H

#include "Arduino.h"
#include "pinout.h"
#include "MechanicalVentilation.h"

typedef struct alarm_data {
    bool is_alarm_vt_on = false;
    short alarm_state = 0;
    uint8_t alarm_max_pressure = 35;
    uint8_t alarm_peep_pressure = 5;
    uint8_t alarm_vt = 20;
} AlarmData;

typedef struct buzzer_state {
    bool last_mute;
    bool buzz_muted;
    unsigned long mute_count;
    unsigned long time_buzz;
    bool is_buzzer_on;
} Buzzer_State_t;

void show_power_led();
AlarmData& check_alarms(Ventilation_Status_t vent_status, AlarmData& alarm_data);
Buzzer_State_t& check_buzzer_mute(Buzzer_State_t& buzzer_state, unsigned long time);
Buzzer_State_t& set_alarm_buzzer(short alarm_state, Buzzer_State_t& buzzer_state);

#endif //AMBOVIS_ALARMS_H
