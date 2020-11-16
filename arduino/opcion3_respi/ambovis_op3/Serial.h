#ifndef _SERIAL_H 
#define _SERIAL_H

#include <Arduino.h>

#define TIME_     0
#define P_        1
#define FLUX_     2
#define ALARM_    3
#define VT_       4
#define VI_       5
#define VE_       6

extern boolean newData;
const byte numChars = 32;
void showNewData();
extern char receivedChars[numChars]; // an array to store the received datae;
extern int integerFromPC [5];

byte recvWithEndMarker();
void parseData();

#endif
