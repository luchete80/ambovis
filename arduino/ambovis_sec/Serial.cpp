#include "Serial.h"

extern boolean newData = false;
char receivedChars[numChars];

byte recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();
    
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
   }
  return ndx;
}

  void showNewData() {
  if (newData == true) {
    //Serial.print("This just in ... ");
    //Serial.println(receivedChars);
    newData = false;
  }
}
