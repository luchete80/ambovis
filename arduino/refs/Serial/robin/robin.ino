#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

char a[10],b[10];

const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
char messageFromPC[32] = {0};
int last_t;
int integerFromPC [3];
float floatFromPC = 0.0;

char recvChar;
char endMarker = '>';
boolean newData = false;


void setup() {
  Serial.begin(115200);
  Serial.println("<Arduino is ready>");
  u8g2.begin();

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  //receivedChars={"1,10,3,4"};
  //parseData();
  Serial.print(integerFromPC[0]);Serial.print(" ");Serial.print(integerFromPC[1]);Serial.println();
  
}

int count=0;
byte escala=32;
byte x[128],y[64];

void loop() {
  recvWithEndMarker();
  showNewData();
  parseData();
  y[integerFromPC[0]]=integerFromPC[1];
  //y[64]=integerFromPC[1];
  //if (last_t!=integerFromPC [0])
  //{
      u8g2.firstPage();
      do {
          //u8g2.drawPixel(126,20);
          u8g2.drawStr( 0, 0, "p: ");
          //u8g2.drawStr( 20, 0, integerFromPC[0]);
          //u8g2.drawStr( 40, 0, integerFromPC[1]);
          //u8g2.drawStr( 20, 0, Serial.parseInt());
          //u8g2.drawStr( 45, 0, a);
          //u8g2.drawStr( 0, 10, receivedChars);
          drawY();
      } while ( u8g2.nextPage() );
  //}
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  // if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

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
}

void showNewData() {
  if (newData == true) {
    //Serial.print("This just in ... ");
    //Serial.println(receivedChars);
    newData = false;
  }
}

void drawY(){
  u8g2.drawPixel(0, y[0]);
  for(int i=0; i<128+1; i++){
      u8g2.drawPixel(i, y[i]);
      //u8g.drawLine(i-1, y[i-1], i, y[i]);
  }  
}

//From https://forum.arduino.cc/index.php?topic=288234.0
//So far there has been no attempt to parse the received data, but that is easy to do once all of the data has been received. 
//The code in this short demo assumes that the data "This is a test, 1234, 45.3" has been received and placed in the array receivedChars. 
//It uses the function strtok() to spilt the data at the commas and the functions atoi() and atof() to convert the ascii data into an integer and a float respectively.
//EL ORIGINAL ES ASI
void parseData() {

    // split the data into its parts
    
  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(receivedChars, ","); // this continues where the previous call left off
  integerFromPC[0] = atoi(strtokIndx);     // convert this part to an integer
  
  for (int i=1;i<3;i++) {
  strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
  integerFromPC[i] = atoi(strtokIndx);     // convert this part to an integer
  }

}
