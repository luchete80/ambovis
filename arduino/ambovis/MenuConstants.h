//
// Created by Mirian Quinteros on 28/06/2022.
//
#ifndef _MENU_CONSTANTS_H_
#define _MENU_CONSTANTS_H_

#define MAIN 0
#define PARAMETER 1
#define ALARM 2
#define SETTINGS 3

// space to add more options
#define MODE_OPT 10
#define PERC_V_OPT 11
#define BPM_OPT 12
#define IE_OPT 13
#define PIP_OPT 14
// space to add more options
#define PIP_ALARM_OPT 20
#define PEEP_ALARM_OPT 21
#define VT_ALARM_OPT 22
// space to add more options
#define FIL_OPT 31
#define AUTO_OPT 32
#define CD_OPT 33
// space to add more options

#define END_SETUP 100

#define VENTMODE_VCL 0
#define VENTMODE_PCL 1
#define VENTMODE_MAN 2

static const byte back[8] = {
        0b00100,
        0b01000,
        0b11111,
        0b01001,
        0b00101,
        0b00001,
        0b00001,
        0b11111
};

#endif //_MENU_CONSTANTS_H_
