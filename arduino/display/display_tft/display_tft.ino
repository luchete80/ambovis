#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define ILI9341_LIGHTGREY 0xC618 /* 192, 192, 192 */
#define ILI9341_DARKGREY 0x7BEF /* 128, 128, 128 */

#define DEBUG 1
//enum serialpos={ALARM_=0,TIME_,PRESSURE_,FLUX_,VT_};
//enum serialpos {TIME_=0,PRESSURE_,FLUX_,VT_,ALARM_};//ORIGINAL
#define TIME_     0
#define P_        1
#define FLUX_     2
#define ALARM_    3
#define VT_       4
#define VI_       5
#define VE_       6

bool ve_readed,vi_readed;

float diff_var[]={10., 10., 800.,0.,200.}; 
int vi,ve,vt;
unsigned long cyclenum;
bool vols_drawn;

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

unsigned long time_last_show=0;

char a[10],b[10];

const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
int last_t;
int integerFromPC [7];
float floatFromPC = 0.0;
int last_vals[7][2];

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
int xgra[3][2];
char buffer[10];

void setup() {
  Serial.begin(250000);
  Serial.println("ILI9341 Test!"); 

  pinMode(buzzer, OUTPUT); //Set buzzerPin as output
  pinMode(GREEN_LED, OUTPUT); //Set buzzerPin as output
  pinMode(YELLOW_LED, OUTPUT); //Set buzzerPin as output
  pinMode(RED_LED, OUTPUT); //Set buzzerPin as output

  
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);

  axispos[0]=63;  axispos[1]=160;  axispos[2]=260;
  cyclenum=0;

}


void loop(void) {

	recvWithEndMarker();
	  //recvWithStartEndMarkers();
	showNewData();

	  tft.setRotation(1);
	  drawY2(ILI9341_GREEN);
//          newData = false;

    if (last_x<10 && !lcd_cleaned){
        for (int i=0;i<3;i++)
          xgra[i][0]=xgra[i][1]=240;
        cyclenum++;
        for (int i=0;i<6;i++)
          last_vals[i][0]=last_vals[i][1]=0;

        //rx[0]=rx[1]=240;
        lcd_cleaned=true;
        vi_readed=false;
        ve_readed=false;
        vols_drawn=false;
        vi=ve=0;
        valsreaded=0;
        for (int i=0;i<3;i++) 
          valsreaded_[i]=0;
        state=0;
        wait4statechg=false;
        //tft.fillRect(0,240-last_x, 320,240-last_x+10, ILI9341_BLACK);
        tft.fillScreen(ILI9341_BLACK);
        //AXIS
        for (int i=0;i<3;i++)
          tft.drawLine(axispos[i],0, axispos[i], 240, ILI9341_LIGHTGREY);


//        tft.setRotation(0);
//        //itoa(integerFromPC[5], buffer, 10);
//        itoa(vi, buffer, 10);
//        tft.setCursor(160, 180);
//        tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
//        tft.println("Vi: ");tft.setCursor(200, 180);tft.println(buffer);

//        itoa(ve, buffer, 10);
//        tft.setCursor(160, 200);
//        tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
//        tft.println("Ve: ");tft.setCursor(200, 200);tft.println(buffer);
//
//        itoa((vi+ve)/2, buffer, 10);
//        tft.setCursor(160, 220);
//        tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
//        tft.println("VT: ");tft.setCursor(200, 220);tft.println(buffer);
        
    }
    if (last_x>10 && lcd_cleaned){
        lcd_cleaned=false;
    }

        if (!vols_drawn && ve_readed && vi_readed) {
              tft.setRotation(0);
            //itoa(integerFromPC[5], buffer, 10);
            itoa(vi, buffer, 10);
            tft.setCursor(160, 180);
            tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
            tft.println("Vi: ");tft.setCursor(200, 180);tft.println(buffer);
    
                    itoa(ve, buffer, 10);
            tft.setCursor(160, 200);
            tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
            tft.println("Ve: ");tft.setCursor(200, 200);tft.println(buffer);
    
            itoa((vi+ve)/2, buffer, 10);
            tft.setCursor(160, 220);
            tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
            tft.println("VT: ");tft.setCursor(200, 220);tft.println(buffer);
            vols_drawn=true;
        }
        
  parseData();

  #ifdef DEBUG
  if (millis()-time_last_show>200) {
      tft.setRotation(0);
      tft.fillRect(150,310,240,320, ILI9341_BLACK);
      tft.setCursor(150, 310);
      tft.setTextColor(ILI9341_RED);  tft.setTextSize(1);
      //for (int i=0;i<5;i++) {
        //tft.setCursor(150+20*i, 310);
        itoa(integerFromPC[4], buffer, 10);
        tft.println(buffer);//tft.println(",");
        time_last_show=millis();

  }
  #endif
  
    switch (state){
        case NO_ALARM:
          digitalWrite(GREEN_LED,HIGH); digitalWrite(YELLOW_LED,LOW); digitalWrite(RED_LED,LOW);      
        break;
        case PEEP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(YELLOW_LED,HIGH); digitalWrite(RED_LED,LOW);  
          tft.setRotation(0);
          tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(1.5); 
          tft.setCursor(160, 0);   
          tft.println("PEEP AL");
        break;
        case PIP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(YELLOW_LED,LOW); digitalWrite(RED_LED,HIGH);      
          tft.setRotation(0);
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(160, 0);   
          tft.println("PIP AL");
		  break;  
        case PEEP_PIP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(YELLOW_LED,HIGH); digitalWrite(RED_LED,HIGH);      
          tft.setRotation(0);
          tft.setTextColor(ILI9341_RED); tft.setTextSize(1.5); 
          tft.setCursor(160, 0);   
          tft.println("PIP AL");
          tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(1.5); 
          tft.setCursor(160, 10);   
          tft.println("PEEP AL");
		  break;
      }

    //tone( pin number, frequency in hertz, duration in milliseconds);
    tone(buzzer,1500,200);
    //Check buzzer
    if (int(state) > 0) {
        if (millis() > timebuzz + TIME_BUZZER) {
            timebuzz=millis();
            //digitalWrite(buzzer,isbuzzeron);
            isbuzzeron=!isbuzzeron;
            if (isbuzzeron)
              //tone(buzzer,500,TIME_BUZZER);
              tone(buzzer,200);
            else 
              noTone(buzzer);
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

  if (abs(ry[valsreaded]-ry[valsreaded-1])<20) {tft.drawLine(axispos[0] - ry[valsreaded-1], 240-rx[valsreaded-1], axispos[0] - ry[valsreaded],  240-rx[valsreaded], color);}
  tft.drawLine(axispos[1]-yflux[0],             240-rx[valsreaded-1], axispos[1]-yflux[1],          240-rx[valsreaded], ILI9341_RED);
  tft.drawLine(axispos[2]-yvt[0],               240-rx[valsreaded-1], axispos[2]-yvt[1],            240-rx[valsreaded], ILI9341_CYAN);
  //tft.drawLine(axispos[1]-yflux[0],           240-xgra[FLUX_][0], axispos[1]-yflux[1],          240-xgra[FLUX_][1], ILI9341_RED);
  //tft.drawLine(axispos[2]-yvt[0],             240-xgra[VT_][0],   axispos[2]-yvt[1],            240-xgra[VT_][1],   ILI9341_CYAN);

       
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
	strtokIndx = strtok(NULL,","); // this continues where the previous call left off
	integerFromPC[4] = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL,","); // this continues where the previous call left off
  integerFromPC[5] = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL,","); // this continues where the previous call left off
  integerFromPC[6] = atoi(strtokIndx);     // convert this part to an integer
  
	if (integerFromPC[ALARM_]!=0 && !wait4statechg) {
  	state=integerFromPC[ALARM_];
  	wait4statechg=true;
	}

	 if ( integerFromPC[TIME_] != last_x /*&& abs(integerFromPC[P_])<ry[valsreaded]+10 */&& /*/*integerFromPC[0] < 127 */ integerFromPC[TIME_] < last_x+20) {
		 valsreaded+=1;
		 last_x=integerFromPC[TIME_];
		 rx[valsreaded]=integerFromPC[TIME_];
		 ry[valsreaded]=integerFromPC[P_];     
   }

  if (integerFromPC[VI_] != vi && !vi_readed) {
      vi=integerFromPC[VI_];
      vi_readed=true;
  }

  if (integerFromPC[VE_] != vi && !ve_readed) {
      ve=integerFromPC[VE_];
      ve_readed=true;
  }

  if ( integerFromPC[FLUX_] != 0 && abs(integerFromPC[FLUX_]) < abs(last_vals[FLUX_][1])+diff_var[FLUX_]) {
    yflux[0]=yflux[1];yflux[1]=int(float(integerFromPC[FLUX_])*0.05);
    last_vals[FLUX_][0]=last_vals[FLUX_][1];last_vals[FLUX_][1]=integerFromPC[FLUX_];
    xgra[FLUX_][0]=xgra[FLUX_][1];xgra[FLUX_][1]=integerFromPC[TIME_];
  }
   if ( integerFromPC[VT_] != 0 && abs(integerFromPC[VT_]) < abs(last_vals[VT_][1])+diff_var[VT_]) {
    yvt[0]=yvt[1];yvt[1]=int(float(integerFromPC[VT_])*0.075);
    last_vals[VT_][0]=last_vals[VT_][1];last_vals[VT_][1]=integerFromPC[VT_];
    xgra[VT_][0]=xgra[VT_][1];xgra[VT_][1]=integerFromPC[TIME_];
  }
    
    //Serial.print(integerFromPC[1]);Serial.print(",");Serial.print(integerFromPC[2]);Serial.print(",");Serial.println(integerFromPC[3]);
}
