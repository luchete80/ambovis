#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define ILI9341_LIGHTGREY 0xC618 /* 192, 192, 192 */
#define ILI9341_DARKGREY 0x7BEF /* 128, 128, 128 */


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

int axispos[3]; //from each graph

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
int valsreaded_[3];
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

  axispos[0]=63;  axispos[1]=160;  axispos[2]=260;

}


void loop(void) {

	recvWithEndMarker();
	  //recvWithStartEndMarkers();
	showNewData();

	  tft.setRotation(1);
	  drawY2(ILI9341_GREEN);
//          newData = false;

      if (last_x<10 && !lcd_cleaned){
        lcd_cleaned=true;
        valsreaded=0;
        for (int i=0;i<3;i++) 
          valsreaded_[i]=0;
        wait4statechg=false;
        state=0;
        //tft.fillRect(0,240-last_x, 320,240-last_x+10, ILI9341_BLACK);
        tft.fillScreen(ILI9341_BLACK);
        //AXIS
        for (int i=0;i<3;i++)
          tft.drawLine(axispos[i],0, axispos[i], 240, ILI9341_DARKGREY);
      }
      if (last_x>10 && lcd_cleaned){
        lcd_cleaned=false;
        }
  //}
  parseData();

	tft.setRotation(0);
	tft.setCursor(0, 0);
	tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(2);
	itoa(state, buffer, 10);
	tft.println(buffer);
	//  //tft.println("Hello World!");
	//  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(3);
	//  tft.println(1234.56);
	//  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
	//    
		//state = integerFromPC[2];

    switch (state){
        case NO_ALARM:
          digitalWrite(GREEN_LED,HIGH); digitalWrite(YELLOW_LED,LOW); digitalWrite(RED_LED,LOW);      
        break;
        case PEEP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(YELLOW_LED,HIGH); digitalWrite(RED_LED,LOW);  
          tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2); 
          tft.setCursor(160, 0);   
          tft.println("PEEP AL");
        break;
        case PIP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(YELLOW_LED,LOW); digitalWrite(RED_LED,HIGH);      
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(160, 0);   
          tft.println("PIP AL");
		  break;  
        case PEEP_PIP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(YELLOW_LED,HIGH); digitalWrite(RED_LED,HIGH);      
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(160, 0);   
          tft.println("PIP AL");
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


void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

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

  tft.drawLine(axispos[0] - ry[valsreaded-1], 240-rx[valsreaded-1], axispos[0] - ry[valsreaded], 240-rx[valsreaded], color);
  tft.drawLine(axispos[1]-yflux[0],           240-rx[valsreaded-1], axispos[1]-yflux[1],          240-rx[valsreaded], ILI9341_RED);
  tft.drawLine(axispos[2]-yvt[0],             240-rx[valsreaded-1], axispos[2]-yvt[1],            240-rx[valsreaded], ILI9341_BLUE);

       
  //yield();
}

void parseData() {

	// split the data into its parts

	char * strtokIndx; // this is used by strtok() as an index

	strtokIndx = strtok(receivedChars, ","); // this continues where the previous call left off
	integerFromPC[0] = atoi(strtokIndx);     // convert this part to an integer

	for (int i=1;i<4;i++) {
	strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
	integerFromPC[i] = atoi(strtokIndx);     // convert this part to an integer
	}
	strtokIndx = strtok(NULL,NULL); // this continues where the previous call left off
	integerFromPC[4] = atoi(strtokIndx);     // convert this part to an integer

	if (integerFromPC[0]!=0 && !wait4statechg) {
	state=integerFromPC[0];
	wait4statechg=true;
	}


	if (valsreaded > 0) {
	 if ( integerFromPC[1] != last_x && /*integerFromPC[0] < 127 */ integerFromPC[0] < last_x+10) {
		 valsreaded+=1;
		 last_x=integerFromPC[1];
		 rx[valsreaded]=integerFromPC[1];
		 ry[valsreaded]=integerFromPC[2];     
		 //ry2[valsreaded]=int(float(integerFromPC[3])*0.15); 
		 yflux[0]=yflux[1];yflux[1]=int(float(integerFromPC[3])*0.05);    
		 yvt[0]	=yvt[1];yvt[1]=		int(float(integerFromPC[4])*0.15);   
	 }
	} else {
		 valsreaded+=1;
		 last_x=integerFromPC[1];
		 rx[valsreaded]=integerFromPC[1];
		 ry[valsreaded]=integerFromPC[2];       
		 //ry2[valsreaded]=int(float(integerFromPC[3])*0.15); 
		 yflux[0]=yflux[1];yflux[1]=integerFromPC[3]; 
		 yvt[0]=yvt[1];yvt[1]=int(float(integerFromPC[4])*0.15);   
		 }
    Serial.print(integerFromPC[1]);Serial.print(",");Serial.print(integerFromPC[2]);Serial.print(",");Serial.println(integerFromPC[3]);
}
