#include "display.h"
#include "MechVentilation.h"

bool lcd_cleaned=false;

unsigned long time_last_show=0;

char a[10],b[10];

const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
int last_t;
int integerFromPC [5];
float floatFromPC = 0.0;

int axispos[]={63,160,260}; //from each graph

int buzzer=3; //pin
unsigned long timebuzz=0;
bool isbuzzeron=false;

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

//void setup() {
//  Serial.begin(250000);
//  Serial.println("ILI9341 Test!"); 
//
//  pinMode(buzzer, OUTPUT); //Set buzzerPin as output
//  pinMode(GREEN_LED, OUTPUT); //Set buzzerPin as output
//  pinMode(YELLOW_LED, OUTPUT); //Set buzzerPin as output
//  pinMode(RED_LED, OUTPUT); //Set buzzerPin as output
//
//  
//  tft.begin();
//  tft.fillScreen(ILI9341_BLACK);
//
//  axispos[0]=63;  axispos[1]=160;  axispos[2]=260;
//
//}


void tft_draw(void) {

    valsreaded+=1;
    last_x=cycle_pos;
    rx[valsreaded]=cycle_pos;
    ry[valsreaded]=pressure_p;     

    yflux[0]=yflux[1];yflux[1]=int(_flux*0.05);
    yvt[0]=yvt[1];yvt[1]=int(_mllastInsVol*0.15);

  
  	tft.setRotation(1);
  	drawY2(ILI9341_GREEN);
  
  	if (last_x<10 && !lcd_cleaned){
  		lcd_cleaned=true;
  		valsreaded=0;
  		for (int i=0;i<3;i++) 
  		  valsreaded_[i]=0;
  		state=0;
  		wait4statechg=false;
  		//tft.fillRect(0,240-last_x, 320,240-last_x+10, ILI9341_BLACK);
  		tft.fillScreen(ILI9341_BLACK);
  		//AXIS
  		for (int i=0;i<3;i++)
  		  tft.drawLine(axispos[i],0, axispos[i], 240, ILI9341_DARKGREY);
  		}
  		if (last_x>10 && lcd_cleaned){
  		lcd_cleaned=false;
	}
  

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
      //}
      //tft.println(receivedChars);
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

void drawY2(uint16_t color){// THERE IS NO NEED TO REDRAW ALL IN EVERY FRAME WITH COLOR TFT

  tft.drawLine(axispos[0] - ry[valsreaded-1], 240-rx[valsreaded-1], axispos[0] - ry[valsreaded], 240-rx[valsreaded], color);
  tft.drawLine(axispos[1]-yflux[0],           240-rx[valsreaded-1], axispos[1]-yflux[1],          240-rx[valsreaded], ILI9341_RED);
  tft.drawLine(axispos[2]-yvt[0],             240-rx[valsreaded-1], axispos[2]-yvt[1],            240-rx[valsreaded], ILI9341_BLUE);

}
