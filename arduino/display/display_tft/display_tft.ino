#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "src/TimerOne/TimerOne.h"

#define ILI9341_LIGHTGREY 0xC618 /* 192, 192, 192 */
#define ILI9341_DARKGREY 0x7BEF /* 128, 128, 128 */

#define TIME_MUTE 30 //seconds
//#define DEBUG 1
//enum serialpos={ALARM_=0,TIME_,PRESSURE_,FLUX_,VT_};
//enum serialpos {TIME_=0,PRESSURE_,FLUX_,VT_,ALARM_};//ORIGINAL
#define TIME_     0
#define P_        1
#define FLUX_     2
#define ALARM_    3
#define VT_       4
#define VI_       5
#define VE_       6

//READ THIS FOR TONE FLREQ
//https://electronics.stackexchange.com/questions/47114/tone-and-reading-data-from-serial-are-colliding

bool ve_readed,vi_readed;
bool state_chg;
float diff_var[]={10., 20., 800.,0.,300.}; 
int vi,ve,vt;
unsigned long cyclenum;
bool vols_drawn;
int mute_count;

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
#define VT_LED      7

#define PIN_MUTE    2

enum _state {NO_ALARM=0,PEEP_ALARM=1,PIP_ALARM=2,PEEP_PIP_ALARM=3,PEEP_ALARM_VT=11,PIP_ALARM_VT=12,PEEP_PIP_ALARM_VT=13};
bool wait4statechg=false;

int state,pre_state,state_r,last_state;//pre_state is to verification

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
int yp[2];
int yflux[2];
int yvt[2];
int xgra[5][2];
char buffer[10];

//MUTE
boolean last_mute,curr_mute;
boolean buzzmuted;

boolean debounce(boolean last, int pin) {
    boolean current = digitalRead(pin);
    if (last != current) {
        delay(50);
        current = digitalRead(pin);
    }
    return current;
}

void setup() {
    Serial.begin(250000);
    Serial.println("ILI9341 Test!"); 

    pinMode(VT_LED, OUTPUT); //Set buzzerPin as output
    pinMode(PIN_MUTE, INPUT);
    pinMode(buzzer, OUTPUT); //Set buzzerPin as output
    pinMode(GREEN_LED, OUTPUT); //Set buzzerPin as output
    pinMode(YELLOW_LED, OUTPUT); //Set buzzerPin as output
    pinMode(RED_LED, OUTPUT); //Set buzzerPin as output
  
    
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
  
    axispos[0]=63;  axispos[1]=160;  axispos[2]=260;
    cyclenum=0;
    buzzmuted=false;
    last_mute=LOW;

    Timer1.initialize(500000); //MILLIS CANNOT BE USED
    Timer1.attachInterrupt(timer1Isr);
    mute_count=0;
}

void loop(void) {

  	recvWithEndMarker();
  	  //recvWithStartEndMarkers();
  	showNewData();

    curr_mute = debounce ( last_mute, PIN_MUTE );         //Debounce for Up button
    if (last_mute== LOW && curr_mute == HIGH && !buzzmuted){
        mute_count=0;
        buzzmuted=true;
    }
    last_mute = curr_mute;
    if(buzzmuted) {
        if (mute_count > 2*TIME_MUTE)  //each count is every 500 ms
        buzzmuted=false;
    }
    
	  tft.setRotation(1);
	  drawY2(ILI9341_GREEN);

    if (last_x<10 && !lcd_cleaned){
        for (int i=0;i<5;i++)
          xgra[i][0]=xgra[i][1]=0;
        cyclenum++;
        for (int i=0;i<6;i++)
          last_vals[i][0]=last_vals[i][1]=0;
        pre_state=0;
        state=0;
        lcd_cleaned=true;
        vi_readed=false;
        ve_readed=false;
        vols_drawn=false;
        vi=ve=0;
        valsreaded=0;
        state_chg=false;
        //Serial.print("buzzmutd: ");Serial.println(buzzmuted);
        for (int i=0;i<3;i++) 
          valsreaded_[i]=0;
        wait4statechg=false;
        //tft.fillRect(0,240-last_x, 320,240-last_x+10, ILI9341_BLACK);
        tft.fillScreen(ILI9341_BLACK);
        state=last_state;
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
            tft.setCursor(150, 180);
            tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
            tft.println("Vi: ");tft.setCursor(190, 180);tft.println(buffer);
    
                    itoa(ve, buffer, 10);
            tft.setCursor(150, 200);
            tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
            tft.println("Ve: ");tft.setCursor(190, 200);tft.println(buffer);
    
            itoa((vi+ve)/2, buffer, 10);
            tft.setCursor(150, 220);
            tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
            tft.println("VT: ");tft.setCursor(190, 220);tft.println(buffer);
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
    if (state>9) {
      digitalWrite(VT_LED,HIGH);
       digitalWrite(GREEN_LED,LOW);
      tft.setRotation(0);
      tft.setTextColor(ILI9341_ORANGE); tft.setTextSize(2); 
      tft.setCursor(150, 40);   
      tft.println("VT AL");
      state_r=state-10;
    } else {
      digitalWrite(VT_LED,LOW);
      digitalWrite(GREEN_LED,LOW);
      state_r=state;
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
    
//
//    if (state > 0) {
//          if (!buzzmuted) {
////              if (millis() > timebuzz + TIME_BUZZER) {
////                  timebuzz=millis();
////                  isbuzzeron=!isbuzzeron;
////                  if (isbuzzeron){
//                      tone(buzzer,488.28125);
////                  }   
////                  else {
////                      noTone(buzzer);
////                  }
////              }
//          } else {  //buzz muted
//              noTone(buzzer);
//          }
//    } else {//state > 0
//      noTone(buzzer);
//      isbuzzeron=true;        //Inverted logic
//    }
    

}

void timer1Isr(void)
{
    if (state > 0) {
        if (!buzzmuted) {
            isbuzzeron=!isbuzzeron;
            if (isbuzzeron){
                analogWrite(buzzer, 127);
            } else {
                analogWrite(buzzer, 0);
            }
        } 
    } else {
          analogWrite(buzzer, 0);
      }
    if (buzzmuted) {
      mute_count+=1;  
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

  //if (ry[valsreaded]!= 0 && abs(ry[valsreaded]-ry[valsreaded-1])<20) {tft.drawLine(axispos[0] - ry[valsreaded-1], 240-rx[valsreaded-1], axispos[0] - ry[valsreaded],  240-rx[valsreaded], color);}
  tft.drawLine(axispos[0]-yp[0], 240-xgra[P_][0], axispos[0]-yp[1],  240-xgra[P_][1], color);
    
  //tft.drawLine(axispos[1]-yflux[0],             240-rx[valsreaded-1], axispos[1]-yflux[1],  240-rx[valsreaded], ILI9341_MAGENTA);
  //tft.drawLine(axispos[2]-yvt[0],               240-rx[valsreaded-1], axispos[2]-yvt[1],    240-rx[valsreaded], ILI9341_CYAN);
  tft.drawLine(axispos[1]-yflux[0], 240-xgra[FLUX_][0], axispos[1]-yflux[1],  240-xgra[FLUX_][1], ILI9341_MAGENTA);
  tft.drawLine(axispos[2]-yvt[0],   240-xgra[VT_][0],   axispos[2]-yvt[1],    240-xgra[VT_][1],   ILI9341_CYAN);

       
  //yield();
}

void parseData() {
	char * strtokIndx; // this is used by strtok() as an index

	strtokIndx = strtok(receivedChars, ","); // this continues where the previous call left off
	integerFromPC[0] = atoi(strtokIndx);     // convert this part to an integer

	for (int i=1;i<7;i++) {
    	strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    	integerFromPC[i] = atoi(strtokIndx);     // convert this part to an integer
	}
  
	if (integerFromPC[ALARM_]!=0 && !wait4statechg) {
  	pre_state++;
	}

 if (pre_state > 4 && !wait4statechg) {
      state=integerFromPC[ALARM_];
      state_chg=true;
      wait4statechg=true;
  }

	 if ( integerFromPC[TIME_] != last_x /*&& abs(integerFromPC[P_])<ry[valsreaded]+10 */&& /*/*integerFromPC[0] < 127 */ integerFromPC[TIME_] < last_x+20) {
		 valsreaded+=1;
		 last_x=integerFromPC[TIME_];
		 rx[valsreaded]=integerFromPC[TIME_];
		 ry[valsreaded]=integerFromPC[P_];     
   }

  if ( integerFromPC[P_] != 0 && abs(integerFromPC[P_]) < abs(last_vals[P_][1])+diff_var[P_] && integerFromPC[TIME_] > xgra[P_][1]) {
    yp[0]=yp[1];yp[1]=int(float(integerFromPC[P_])*2.);
    last_vals[P_][0]=last_vals[P_][1];last_vals[P_][1]=integerFromPC[P_];
    xgra[P_][0]=xgra[P_][1];xgra[P_][1]=integerFromPC[TIME_];
  }
  
  if (integerFromPC[VI_] != vi && !vi_readed) {
      vi=integerFromPC[VI_];
      vi_readed=true;
  }

  if (integerFromPC[VE_] != vi && !ve_readed) {
      ve=integerFromPC[VE_];
      ve_readed=true;
  }

  if ( integerFromPC[FLUX_] != 0 && abs(integerFromPC[FLUX_]) < abs(last_vals[FLUX_][1])+diff_var[FLUX_] && integerFromPC[TIME_] > xgra[FLUX_][1]) {
    yflux[0]=yflux[1];yflux[1]=int(float(integerFromPC[FLUX_])*0.04);
    last_vals[FLUX_][0]=last_vals[FLUX_][1];last_vals[FLUX_][1]=integerFromPC[FLUX_];
    xgra[FLUX_][0]=xgra[FLUX_][1];xgra[FLUX_][1]=integerFromPC[TIME_];
  }
   if ( integerFromPC[VT_] != 0 && abs(integerFromPC[VT_]) < abs(last_vals[VT_][1])+diff_var[VT_] && integerFromPC[TIME_] > xgra[VT_][1]) {
    yvt[0]=yvt[1];yvt[1]=int(float(integerFromPC[VT_])*0.07);
    last_vals[VT_][0]=last_vals[VT_][1];last_vals[VT_][1]=integerFromPC[VT_];
    xgra[VT_][0]=xgra[VT_][1];xgra[VT_][1]=integerFromPC[TIME_];
  }
    
    //Serial.print(integerFromPC[1]);Serial.print(",");Serial.print(integerFromPC[2]);Serial.print(",");Serial.println(integerFromPC[3]);
}
