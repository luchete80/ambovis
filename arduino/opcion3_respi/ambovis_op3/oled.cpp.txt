#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>

//#define DEBUG 1


U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

char a[10],b[10];

const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
char messageFromPC[32] = {0};
int last_t;
int integerFromPC [4];
float floatFromPC = 0.0;

int buzzer=3; //pin
unsigned long timebuzz=0;
bool isbuzzeron=false;
#define TIME_BUZZER 500

#define GREEN_LED   4
#define YELLOW_LED  5
#define RED_LED     6

enum _state {NO_ALARM=0,PEEP_ALARM=1,PIP_ALARM=2,PEEP_PIP_ALARM=3};
bool wait4statechg=false;

_state state;

char recvChar;
char endMarker = '>';
boolean newData = false;
byte valsreaded=0;
byte last_x=0;

void setup() {
  Serial.begin(115200);
  Serial.println("<Arduino is ready>");
  u8g2.begin();
  pinMode(buzzer, OUTPUT); //Set buzzerPin as output
  pinMode(GREEN_LED, OUTPUT); //Set buzzerPin as output
  pinMode(YELLOW_LED, OUTPUT); //Set buzzerPin as output
  pinMode(RED_LED, OUTPUT); //Set buzzerPin as output
  
  digitalWrite(buzzer, LOW); //Set buzzerPin as output
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  //receivedChars={"1,10,3,4"};
  //gData();
  Serial.print(integerFromPC[0]);Serial.print(" ");Serial.print(integerFromPC[1]);Serial.println();
}

int count=0;
byte escala=32;
byte x[128],y[64];

byte rx[128],ry[128];
char buffer[10];


void loop() {
  recvWithEndMarker();
  //recvWithStartEndMarkers();
  showNewData();
  //y[integerFromPC[0]]=integerFromPC[1];

//   for (int i=0;i<3;i++)
//      integerFromPC[3]=Serial.parseInt();
      
  //y[64]=integerFromPC[1];
  //if (last_t!=integerFromPC [0])
  //{

    
      u8g2.firstPage();
      do {
//          u8g2.drawStr( 50, 10, receivedChars);
//        //Full buffer mode: u8g2.clear() will work, but u8g2.clearBuffer() might be sufficient
//          //u8g2.drawPixel(126,20);
//          u8g2.drawStr( 0, 0, "r: ");
//          itoa(valsreaded, buffer, 10);
//          u8g2.drawStr( 20, 0,buffer);
//          
//          u8g2.drawStr( 0, 10, "x: ");
//          itoa(integerFromPC[0], buffer, 10);
//          u8g2.drawStr( 20, 10, buffer);
//          
          for (int i=0;i<6;i++)
              u8g2.drawLine(0,13+i*10,2,13+i*10); 
                   
          u8g2.drawStr( 10, 0, "p: ");
          itoa(integerFromPC[1], buffer, 10);
          u8g2.drawStr( 30,0, buffer);
//          
          u8g2.drawStr( 80, 0, "st: ");
          itoa(state, buffer, 10);
          u8g2.drawStr( 100, 0, buffer);
//          
//          u8g2.drawStr( 40, 0, integerFromPC[1]);
//          u8g2.drawStr( 20, 0, Serial.parseInt());
//          u8g2.drawStr( 45, 0, a);
//        
          if (state == 1 ) {
            u8g2.drawStr( 60, 20, "AL peep ");
          } else if (state == 2 ){
            u8g2.drawStr( 60, 20, "AL pip");
          } else if (state == 3){
            u8g2.drawStr( 60, 20, "AL pip/peep");
          }
          drawY2();
          newData = false;
      } while ( u8g2.nextPage() );

      if (last_x<5){
        valsreaded=0;
        wait4statechg=false;
        state=0;
      }
  //}
  parseData();
    
    //state = integerFromPC[2];

    switch (state){
        case NO_ALARM:
          digitalWrite(GREEN_LED,HIGH); digitalWrite(YELLOW_LED,LOW); digitalWrite(RED_LED,LOW);      
        break;
        case PEEP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(YELLOW_LED,HIGH); digitalWrite(RED_LED,LOW);      
        break;
        case PIP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(YELLOW_LED,LOW); digitalWrite(RED_LED,HIGH);      
        break;  
        case PEEP_PIP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(YELLOW_LED,HIGH); digitalWrite(RED_LED,HIGH);      
        break;
      }
    
    //Check buzzer
    if (int(state) > 0) {
        if (millis() > timebuzz + TIME_BUZZER) {
            timebuzz=millis();
            digitalWrite(buzzer,isbuzzeron);
            isbuzzeron=!isbuzzeron;
        }
    } else {
      digitalWrite(buzzer,1); //Inverted logic
      isbuzzeron=true;        //Inverted logic
    }
    
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
 // if (Serial.available() > 0) {
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
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
      u8g2.drawPixel(i, 63 - y[i]);
      //u8g.drawLine(i-1, y[i-1], i, y[i]);
  }  
}

void drawY2(){
  u8g2.drawPixel(0, y[0]);
  for(int i=1; i<valsreaded+1; i++){
      //u8g2.drawPixel(rx[i], 63 - ry[i]);
      u8g2.drawLine(rx[i-1], 63 - ry[i-1], rx[i], 63 - ry[i]);
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

  //for (int i=1;i<3;i++) {
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  integerFromPC[1] = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL,","); // this continues where the previous call left off
  integerFromPC[2] = atoi(strtokIndx);     // convert this part to an integer

  if (integerFromPC[2]!=0 && !wait4statechg) {
    state=integerFromPC[2];
    wait4statechg=true;
  }
  
  strtokIndx = strtok(NULL,NULL); // this continues where the previous call left off
  integerFromPC[3] = atoi(strtokIndx);     // convert this part to an integer

  
  //}

  if (valsreaded > 0) {
     if ( integerFromPC[0] != last_x && /*integerFromPC[0] < 127 */ integerFromPC[0] < last_x+10) {
         valsreaded+=1;
         last_x=integerFromPC[0];
         rx[valsreaded]=integerFromPC[0];
         ry[valsreaded]=integerFromPC[1];     
     }
  } else {
             valsreaded+=1;
         last_x=integerFromPC[0];
         rx[valsreaded]=integerFromPC[0];
         ry[valsreaded]=integerFromPC[1];       
    }
}

//void parseData() {
//
//    // split the data into its parts
//   
//  char * strtokIndx; // this is used by strtok() as an index
// 
//  strtokIndx = strtok(receivedChars,",");      // get the first part - the string
//  strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
// 
//  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
//  integerFromPC = atoi(strtokIndx);     // convert this part to an integer
// 
//  //strtokIndx = strtok(NULL, ",");
//  strtokIndx = strtok(NULL, NULL);
//  floatFromPC = atof(strtokIndx);     // convert this part to a float
//
//}
