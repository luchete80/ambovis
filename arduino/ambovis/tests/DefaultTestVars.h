//
// Created by Mirian Quinteros on 23/10/2021.
//

#ifndef AMBOVIS_DEFAULTVARS_H
#define AMBOVIS_DEFAULTVARS_H

byte max_sel,min_sel; //According to current selection
unsigned long lastButtonPress;
int bck_state ;     // current state of the button
int last_bck_state; // previous state of the button
int startPressed ;    // the moment the button was pressed
int endPressed ;      // the moment the button was released
int holdTime ;        // how long the button was hold
int idleTime ;        // how long the button was idle

int curr_sel, old_curr_sel;
byte encoderPos; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
byte oldEncPos; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
byte old_menu_pos,old_menu_num;
bool show_changed_options; //Only for display
bool update_options;
char tempstr[5],tempstr2[5];
byte menu_number;
byte p_trim;
unsigned long last_vent_time;

byte Cdyn;
float _mlInsVol,_mlExsVol;
int _mllastInsVol,_mllastExsVol;
unsigned int _timeoutIns;
unsigned int _timeoutEsp;
byte cycle_pos; //0 to 127
bool display_needs_update;
float last_pressure_max,last_pressure_min,last_pressure_peep;

byte alarm_max_pressure,alarm_peep_pressure;
int alarm_vt;
bool autopid;
bool change_pid_params;
bool filter;
bool isitem_sel;
unsigned long last_cycle;
int max_accel,min_accel;
int max_speed, min_speed;
int min_pidk,max_pidk;
int min_pidi,max_pidi;
int min_pidd,max_pidd;
byte pfmin,pfmax;
float pf_min,pf_max;
float peep_fac;
bool sleep_mode;
bool put_to_sleep,wake_up;
unsigned long print_bat_time;
int min_cd,max_cd;
unsigned long time;

#endif //AMBOVIS_DEFAULTVARS_H
