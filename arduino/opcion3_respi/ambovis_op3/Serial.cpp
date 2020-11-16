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

void parseData() {
  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(receivedChars, ","); // this continues where the previous call left off
  integerFromPC[0] = atoi(strtokIndx);     // convert this part to an integer

  for (int i=1;i<7;i++) {
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      integerFromPC[i] = atoi(strtokIndx);     // convert this part to an integer
  }

}
