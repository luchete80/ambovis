#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define TFT_CLK 13
#define TFT_MISO 12
#define TFT_MOSI 11
#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8

//SPI HARDWARE SHOULD BE PINS CLK #13 AND MISO #11

//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO); //THIS SHOULD NEVER BE USED!
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

bool lcd_cleaned=false;

char a[10],b[10];

const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
char messageFromPC[32] = {0};
int last_t;
int integerFromPC [5];
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


int count=0;
byte escala=32;
byte x[128],y[64];

byte rx[128],ry[128];
int  ry2[128];
int yflux[2];
int yvt[2];
char buffer[10];

void setup() {
  Serial.begin(115200);
  Serial.println("ILI9341 Test!"); 
 
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);

}


void loop(void) {
//  for(uint8_t rotation=0; rotation<4; rotation++) {
//    tft.setRotation(rotation);
//    testText();

//  }
//  Serial.println(testLines(ILI9341_CYAN));
  //tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  //tft.println(1234.56);
  //tft.setRotation(1); 
  recvWithEndMarker();
  //recvWithStartEndMarkers();
  showNewData();
  //y[integerFromPC[0]]=integerFromPC[1];

//   for (int i=0;i<3;i++)
//      integerFromPC[3]=Serial.parseInt();
      
  //y[64]=integerFromPC[1];
  //if (last_t!=integerFromPC [0])
  //{

////          u8g2.drawStr( 50, 10, receivedChars);
////        //Full buffer mode: u8g2.clear() will work, but u8g2.clearBuffer() might be sufficient
////          //u8g2.drawPixel(126,20);
////          u8g2.drawStr( 0, 0, "r: ");
////          itoa(valsreaded, buffer, 10);
////          u8g2.drawStr( 20, 0,buffer);
////          
////          u8g2.drawStr( 0, 10, "x: ");
////          itoa(integerFromPC[0], buffer, 10);
////          u8g2.drawStr( 20, 10, buffer);
////          
//          for (int i=0;i<6;i++)
//              u8g2.drawLine(0,13+i*10,2,13+i*10); 
//                   
//          u8g2.drawStr( 10, 0, "p: ");
//          itoa(integerFromPC[1], buffer, 10);
//          u8g2.drawStr( 30,0, buffer);
////          
//          u8g2.drawStr( 80, 0, "st: ");
//          itoa(state, buffer, 10);
//          u8g2.drawStr( 100, 0, buffer);
////          
////          u8g2.drawStr( 40, 0, integerFromPC[1]);
////          u8g2.drawStr( 20, 0, Serial.parseInt());
////          u8g2.drawStr( 45, 0, a);
////        
//          if (state == 1 ) {
//            u8g2.drawStr( 60, 20, "AL peep ");
//          } else if (state == 2 ){
//            u8g2.drawStr( 60, 20, "AL pip");
//          } else if (state == 3){
//            u8g2.drawStr( 60, 20, "AL pip/peep");
//          }
          tft.setRotation(1);
          drawY2(ILI9341_GREEN);
//          newData = false;

      if (last_x<10 && !lcd_cleaned){
        lcd_cleaned=true;
        valsreaded=0;
        wait4statechg=false;
        state=0;
        //tft.fillRect(0,240-last_x, 320,240-last_x+10, ILI9341_BLACK);
        tft.fillScreen(ILI9341_BLACK);

      }
      if (last_x>10 && lcd_cleaned){
        lcd_cleaned=false;
        }
  //}
  parseData();

  tft.setRotation(3);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
    
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


unsigned long testText() {
  tft.fillScreen(ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
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

void drawY2(uint16_t color){// THERE IS NO NEED TO REDRAW ALL IN EVERY FRAME WITH COLOR TFT
//  u8g2.drawPixel(0, y[0]);
//tft.fillRect(rx[0],0, rx[valsreaded],63, ILI9341_WHITE);
  for(int i=1; i<valsreaded+1; i++){
      //u8g2.drawPixel(rx[i], 63 - ry[i]);
//      u8g2.drawLine(rx[i-1], 63 - ry[i-1], rx[i], 63 - ry[i]);
        int y=integerFromPC[2]/6;
        //tft.drawLine(63 - ry[i-1],240-rx[i-1], 63 - ry[i], 240-rx[i], color);
        //tft.drawLine(260-ry2[i-1],240-rx[i-1], 260-ry2[i], 240-rx[i], ILI9341_BLUE);
  } 

  tft.drawLine(63 - ry[valsreaded-1], 240-rx[valsreaded-1], 63 - ry[valsreaded], 240-rx[valsreaded], color);
  tft.drawLine(180-yflux[0],          240-rx[valsreaded-1], 180-yflux[1], 240-rx[valsreaded], ILI9341_RED);
  tft.drawLine(310-yvt[0], 240-rx[valsreaded-1], 310-yvt[1], 240-rx[valsreaded], ILI9341_BLUE);
  //tft.drawLine(310-ry2[valsreaded-1], 240-rx[valsreaded-1], 310-ry2[valsreaded], 240-rx[valsreaded], ILI9341_BLUE);
        
  //yield();
}

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
 
  strtokIndx = strtok(NULL,","); // this continues where the previous call left off
  integerFromPC[3] = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL,NULL); // this continues where the previous call left off
  integerFromPC[4] = atoi(strtokIndx);     // convert this part to an integer

  if (integerFromPC[4]!=0 && !wait4statechg) {
    state=integerFromPC[4];
    wait4statechg=true;
  }
  
  //}

  if (valsreaded > 0) {
     if ( integerFromPC[0] != last_x && /*integerFromPC[0] < 127 */ integerFromPC[0] < last_x+10) {
         valsreaded+=1;
         last_x=integerFromPC[0];
         rx[valsreaded]=integerFromPC[0];
         ry[valsreaded]=integerFromPC[1];     
         //ry2[valsreaded]=int(float(integerFromPC[3])*0.15); 
         yvt[0]=yvt[1];yvt[1]=int(float(integerFromPC[3])*0.15);   
         yflux[0]=yflux[1];yflux[1]=int(float(integerFromPC[2])*0.05);     
     }
  } else {
         valsreaded+=1;
         last_x=integerFromPC[0];
         rx[valsreaded]=integerFromPC[0];
         ry[valsreaded]=integerFromPC[1];       
         //ry2[valsreaded]=int(float(integerFromPC[3])*0.15); 
         yvt[0]=yvt[1];yvt[1]=int(float(integerFromPC[3])*0.15);   
         yflux[0]=yflux[1];yflux[1]=integerFromPC[2]; 
    }
    Serial.print(integerFromPC[1]);Serial.print(",");Serial.print(integerFromPC[2]);Serial.print(",");Serial.println(integerFromPC[3]);
}
