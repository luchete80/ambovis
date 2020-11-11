#ifndef _SERIAL_H 
#define _SERIAL_H

#include <Arduino.h>

extern boolean newData;
const byte numChars = 32;
void showNewData();
extern char receivedChars[numChars]; // an array to store the received datae;

byte recvWithEndMarker();

#endif
