#include "display.h"
#include "MechVentilation.h"

#define MAX_CURVES_Y    200
#define LEGEND_Y        260 //Begining of the legend on Y AXIS
bool lcd_cleaned=false;

unsigned long time_last_show=0;

char a[10],b[10];

const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
int last_t;
int integerFromPC [5];
float floatFromPC = 0.0;
int axispos[]={100,170,300}; //from each graph, from 0 to 320 (display height, IN PORTRAIT MODE)
byte state_r;
int buzzer=3; //pin

enum _state {NO_ALARM=0,PEEP_ALARM=1,PIP_ALARM=2,PEEP_PIP_ALARM=3};

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

void tft_draw(byte alarm_state, SensorParams sensorParams) {
    //Serial.println(cycle_pos);Serial.println(ry[valsreaded]);
    last_x = cycle_pos;
    rx[valsreaded] = cycle_pos;
    ry[valsreaded] = sensorParams.pressure_p * 2.;

    yflux[0]=yflux[1];
    yflux[1]=int(sensorParams.flow_f * 0.035);
    yvt[0]=yvt[1];
    yvt[1]=int((_mlInsVol - _mlExsVol)*0.1);

    #if TESTING_MODE_DISABLED
    tft.setRotation(1);
    if (valsreaded > 0)
        drawY2(ILI9341_GREEN);
    #endif//TESTING_MODE_DISABLED
    valsreaded+=1;
    if (last_x>117 && !lcd_cleaned){//NO PONER UN VALOR MENOR QUE 10
    		lcd_cleaned=true;
    		valsreaded=0;
    		for (int i=0;i<2;i++) 
    		    valsreaded_[i]=0;
        print_vols();
        print_bat();
        drawing_cycle = !drawing_cycle;
        Serial.println("Drawing cycle: " + String(drawing_cycle));
        #if TESTING_MODE_DISABLED
        tft.fillRect(180,280,70,50, ILI9341_RED);    
        if (ended_whilemov){
          tft.setCursor(150, 300);tft.println("ENDErr");
        }
        else {
          tft.setCursor(150, 300);tft.println("ENDOk");    
        }
        tft.setRotation(1);
        tft.fillRect(0,0,60,100, ILI9341_BLACK); //FOR ALARMS, UPPER RIRHT
        tft.fillRect(0, 240 , 320, 10, ILI9341_GREEN);//x,y,lengthx,lentgthy
        #endif //TESTING_MODE_DISABLED
  	} else {
  	    lcd_cleaned=false;
  	}

    check_alarms(alarm_state);
    
}//loop

void drawY2(uint16_t color){// THERE IS NO NEED TO REDRAW ALL IN EVERY FRAME WITH COLOR TFT
  int x_start = 240 - (int) drawing_cycle * 120;
  if ( rx[valsreaded] > rx[valsreaded-1] ) {//to avoid draw entire line to the begining at the end of the cycle
    #if TESTING_MODE_DISABLED
    for (int i=0;i<2;i++)
      tft.drawLine(axispos[i], x_start - rx[valsreaded-1], axispos[i], x_start - rx[valsreaded], ILI9341_DARKGREY);           //X AXIS 
    tft.fillRect(0, x_start - rx[valsreaded] - 10, MAX_CURVES_Y, 10, ILI9341_BLACK);                                //CLEAN PREVIOUS CURVE x,y,lengthx,lentgthy

    tft.drawLine(axispos[0]- ry[valsreaded-1], x_start - rx[valsreaded-1], axispos[0] - ry[valsreaded],   x_start - rx[valsreaded], color);
    tft.drawLine(axispos[1]- yflux[0],         x_start - rx[valsreaded-1], axispos[1] - yflux[1],         x_start - rx[valsreaded], ILI9341_MAGENTA);
      tft.drawLine(axispos[0]- ry[valsreaded-1], 240-rx[valsreaded-1], axispos[0] - ry[valsreaded],   240-rx[valsreaded], color);
      tft.drawLine(axispos[1]-yflux[0],           240-rx[valsreaded-1], axispos[1]-yflux[1],          240-rx[valsreaded], ILI9341_MAGENTA);
      tft.drawLine(axispos[2]-yvt[0],             240-rx[valsreaded-1], axispos[2]-yvt[1],            240-rx[valsreaded], ILI9341_BLUE);
      #endif//TESTING_MODE_DISABLED
  }
}

void check_alarms(byte alarm_state) {
  
      //Serial.println(state_r);
    if (alarm_state>9) {
        digitalWrite(RED_LED,HIGH);
        digitalWrite(GREEN_LED,LOW);
        #if TESTING_MODE_DISABLED
        tft.setRotation(0);
        tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
        tft.setCursor(150, 40);   
        tft.println("VT AL");
        #endif //TESTING_MODE_DISABLED
        state_r=alarm_state-10;
    } else {
        digitalWrite(RED_LED,LOW);  
        state_r=alarm_state;
    }
    switch (state_r){
        case NO_ALARM:
            if (alarm_state==0){ //state_r!=10
            digitalWrite(GREEN_LED,HIGH); digitalWrite(RED_LED,LOW);   }
          break;
        case PEEP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(RED_LED,HIGH);
          #if TESTING_MODE_DISABLED
          tft.setRotation(0);
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(150, 20);   
          tft.println("PEEP AL");
          #endif //TESTING_MODE_DISABLED
        break;
        case PIP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(RED_LED,HIGH);
          #if TESTING_MODE_DISABLED
          tft.setRotation(0);
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(150, 0);   
          tft.println("PIP AL");
          #endif //TESTING_MODE_DISABLED
      break;  
        case PEEP_PIP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(RED_LED,HIGH);
          #if TESTING_MODE_DISABLED
          tft.setRotation(0);
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(150, 0);   
          tft.println("PIP AL");
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(150, 20);   
          tft.println("PEEP AL");
          #endif //TESTING_MODE_DISABLED
      break;
      }
}

void print_bat(){
    float level,level_perc;
    level=0.;
    #if TESTING_MODE_DISABLED
    tft.setRotation(0);
    //tft.fillRect(180,150,70,20, ILI9341_BLACK);//ONLY BAT LEVEL
    tft.fillRect(180,250,70,50, ILI9341_BLACK);
    #endif //TESTING_MODE_DISABLED

    float fac=0.0279;  //5./(1024.*0.175)
    //Vt > 24V   =>   PC = 100%
    //Vmin < Vt < 24V   =>   PC[%] = (Vt[V]-Vmin)/(24-Vmin)*100
    //Vt < Vmin   =>   PC = 0%
    for (int i=0;i<40;i++){
        level+=float(analogRead(PIN_BAT_LEV));
        //Serial.println(analogRead(PIN_BAT_LEV));
    }
    level*=fac/40.;
    if (level > 24.0) level_perc =100.;
    else {
        if (level > 22.0) level_perc = (level - 22.)/(24.-22.0) * 100.;
        else              level_perc =0.;
      }
    dtostrf(level_perc, 2, 0, buffer);
    //dtostrf(level, 2, 1, buffer);
    //Serial.print("Bat level: ");Serial.println(level);
    #if TESTING_MODE_DISABLED
    tft.setCursor(130, 260);tft.println("Bat:");
    tft.setCursor(180, 260);tft.println(buffer);
    tft.setCursor(220, 260);tft.println("%");
    #endif //TESTING_MODE_DISABLED
    dtostrf(level, 1, 2, buffer);
    //Temporary
    //Serial.print("Bat level: ");Serial.println(level);
    //tft.setCursor(150, 280);tft.println("Vo:");
    //tft.setCursor(180, 280);tft.println(buffer);

}

void print_vols(){
    #if TESTING_MODE_DISABLED
    tft.setRotation(0);
    tft.fillRect(40,LEGEND_Y,40,80, ILI9341_RED); //Here x is the first value (in the less width dimension)

    itoa(_mllastInsVol, buffer, 10);
    tft.setCursor(0, LEGEND_Y); //Before: 150,180 at right 
    tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
    tft.println("Vi: ");tft.setCursor(40, LEGEND_Y);tft.println(buffer); //Before 190,180
    #endif //TESTING_MODE_DISABLED
    
    itoa(_mllastExsVol, buffer, 10);
    #if TESTING_MODE_DISABLED
    tft.setCursor(0, LEGEND_Y + 20);
    tft.println("Ve: ");tft.setCursor(40, LEGEND_Y + 20);tft.println(buffer);
    
    itoa((_mllastInsVol + _mllastExsVol)/2, buffer, 10);
    tft.setCursor(0, LEGEND_Y + 40);
    tft.println("VT: ");tft.setCursor(40, LEGEND_Y + 40);tft.println(buffer);
    #endif //TESTING_MODE_DISABLED

  }
