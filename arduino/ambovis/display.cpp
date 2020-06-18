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
int axispos[]={100,170,300}; //from each graph
byte state_r;
int buzzer=3; //pin

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
void print_vols();
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
    ry[valsreaded]=pressure_p*2.;     

    yflux[0]=yflux[1];yflux[1]=int(_flux*0.035);
    yvt[0]=yvt[1];yvt[1]=int((_mlInsVol - _mlExsVol)*0.1);

  
  	tft.setRotation(1);
  	drawY2(ILI9341_GREEN);
  
  	if (last_x<10 && !lcd_cleaned){
  		lcd_cleaned=true;
  		valsreaded=0;
  		for (int i=0;i<3;i++) 
  		  valsreaded_[i]=0;
  		wait4statechg=false;
      print_vols();
      tft.setRotation(1);
  		//tft.fillRect(0,240-last_x, 320,240-last_x+10, ILI9341_BLACK);
  		//tft.fillScreen(ILI9341_BLACK);
  		//AXIS
      tft.fillRect(0,0,60,100, ILI9341_BLACK);
  		for (int i=0;i<3;i++)
  		  tft.drawLine(axispos[i],0, axispos[i], 240, ILI9341_DARKGREY);
  		}
  		if (last_x>10 && lcd_cleaned){
  		lcd_cleaned=false;
	}
  
    Serial.println(state_r);
    if (alarm_state>9) {
        digitalWrite(RED_LED,HIGH);
        digitalWrite(RED_LED,LOW);
        tft.setRotation(0);
        tft.setTextColor(ILI9341_ORANGE); tft.setTextSize(2); 
        tft.setCursor(150, 40);   
        tft.println("VT AL");
        state_r=alarm_state-10;
    } else {
        digitalWrite(RED_LED,LOW);
        digitalWrite(RED_LED,LOW);
        state_r=alarm_state;
    }
    switch (state_r){
        case NO_ALARM:
            if (state==0) //state_r!=10
            digitalWrite(GREEN_LED,HIGH); digitalWrite(YELLOW_LED,LOW); digitalWrite(RED_LED,LOW);   
          break;
        case PEEP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(YELLOW_LED,HIGH); digitalWrite(RED_LED,LOW);  
          tft.setRotation(0);
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(150, 20);   
          tft.println("PEEP AL");
        break;
        case PIP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(YELLOW_LED,LOW); digitalWrite(RED_LED,HIGH);      
          tft.setRotation(0);
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(150, 0);   
          tft.println("PIP AL");
      break;  
        case PEEP_PIP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(YELLOW_LED,HIGH); digitalWrite(RED_LED,HIGH);      
          tft.setRotation(0);
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(150, 0);   
          tft.println("PIP AL");
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(150, 20);   
          tft.println("PEEP AL");
      break;
      }


}//loop

void print_vols(){
    tft.setRotation(0);
    tft.fillRect(180,160,50,80, ILI9341_BLACK);
    //itoa(integerFromPC[5], buffer, 10);
    itoa(_mllastInsVol, buffer, 10);
    tft.setCursor(150, 180);
    tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
    tft.println("Vi: ");tft.setCursor(190, 180);tft.println(buffer);
    
    itoa(_mllastExsVol, buffer, 10);
    tft.setCursor(150, 200);
    tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
    tft.println("Ve: ");tft.setCursor(190, 200);tft.println(buffer);
    
    itoa((_mllastInsVol + _mllastExsVol)/2, buffer, 10);
    tft.setCursor(150, 220);
    tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
    tft.println("VT: ");tft.setCursor(190, 220);tft.println(buffer);
 
  }
void drawY2(uint16_t color){// THERE IS NO NEED TO REDRAW ALL IN EVERY FRAME WITH COLOR TFT
  if (rx[valsreaded]>rx[valsreaded-1]) {//to avoid draw entire line to the begining at the end of the cycle
  tft.fillRect(0,240-rx[valsreaded]-5, 320,5, ILI9341_BLACK);
  tft.drawLine(axispos[0]- ry[valsreaded-1], 240-rx[valsreaded-1], axispos[0] - ry[valsreaded],   240-rx[valsreaded], color);
  tft.drawLine(axispos[1]-yflux[0],           240-rx[valsreaded-1], axispos[1]-yflux[1],          240-rx[valsreaded], ILI9341_MAGENTA);
  tft.drawLine(axispos[2]-yvt[0],             240-rx[valsreaded-1], axispos[2]-yvt[1],            240-rx[valsreaded], ILI9341_BLUE);
  }
}
