#include "display.h"
#include "MechVentilation.h"

#define MIN_CURVES_Y    60
#define MAX_CURVES_Y    250
#define CLEAN_Y         200
#define LEGEND_Y        260 //Begining of the legend on Y AXIS
bool lcd_cleaned=false;
int axispos[]={130,200}; //from each graph, from 0 to 320 (display height, IN PORTRAIT MODE)
byte state_r;

enum _state {NO_ALARM=0,PEEP_ALARM=1,PIP_ALARM=2,PEEP_PIP_ALARM=3};

_state state;

byte valsreaded=0;

byte rx[128],ry[128];
int yflux[2];
int yvt[2];
char buffer[10];


void tft_draw() {
    byte last_x=cycle_pos;
    rx[valsreaded]=cycle_pos;
    ry[valsreaded]=pressure_p*2.;     

    yflux[0]=yflux[1];
    yflux[1]=int(flow_f*0.035);
    yvt[0]=yvt[1];
    yvt[1]=int((_mlInsVol - _mlExsVol)*0.1);

    tft.setRotation(1);
    if (valsreaded > 0) {
        drawY2(ILI9341_GREEN);
    }
    valsreaded+=1;

    if (last_x>117 && !lcd_cleaned) {//NO PONER UN VALOR MENOR QUE 10
        lcd_cleaned=true;
       //tft.fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
        valsreaded=0;
        print_vols();
        print_bat();
        
        //TODO: DO IT ONLY WHEN CHANGE!
        tft.fillRect(10,10,150,15, ILI9341_BLACK); 
        if (!digitalRead(PIN_POWEROFF)){
          tft.setCursor(10, 10);tft.println("CORTE ENERGIA");         
        }
        
        dtostrf(vlevel, 1, 2, buffer);
        tft.setCursor(100, 80);tft.println("Vmpx:");
        tft.fillRect(180,80,50,50, ILI9341_BLACK);
        tft.setCursor(180, 80);tft.println(buffer);
        
        drawing_cycle = !drawing_cycle;

        tft.fillRect(180,280,70,50, ILI9341_BLACK);
        if (ended_whilemov){
          tft.setCursor(150, 300);tft.println("ENDErr");
        }
        else {
          tft.setCursor(150, 300);tft.println("ENDOk");    
        }
        tft.setRotation(1);
        tft.fillRect(0,0,60,100, ILI9341_BLACK); //FOR ALARMS, UPPER RIRHT
        tft.fillRect(0, 240 , 320, 10, ILI9341_GREEN);//x,y,lengthx,lentgthy

    } else {
        lcd_cleaned=false;
    }

    check_alarms();
    
}//loop

void drawY2(uint16_t color) {// THERE IS NO NEED TO REDRAW ALL IN EVERY FRAME WITH COLOR TFT
  int x_start = 240 - (int) drawing_cycle * 120;
  if ( rx[valsreaded] > rx[valsreaded-1] ) {//to avoid draw entire line to the begining at the end of the cycle
    for (int i=0;i<2;i++)
      tft.drawLine(axispos[i], x_start - rx[valsreaded-1], axispos[i], x_start - rx[valsreaded], ILI9341_DARKGREY);           //X AXIS 
    tft.fillRect(MIN_CURVES_Y, x_start - rx[valsreaded] - 10, CLEAN_Y, 10, ILI9341_BLACK);                                //CLEAN PREVIOUS CURVE x,y,lengthx,lentgthy
    //Serial.print("ry[valsreaded-1]");Serial.println(ry[valsreaded-1]);
    //Serial.print("ry[valsreaded ]");Serial.println(ry[valsreaded ]);
//
//    Serial.print("ry[valsreaded-1]");Serial.println(ry[valsreaded-1]);
//    Serial.print("ry[valsreaded]");Serial.println(ry[valsreaded]);
      if      (ry[valsreaded] > 250 || ry[valsreaded-1] > 250 ){
        ry[valsreaded -1 ] = ry[valsreaded] = 0;
      }
      //if      (ry[valsreaded-1] > MAX_CURVES_Y) ry[valsreaded-1] = MAX_CURVES_Y;
//    else if (ry[valsreaded-1] < 0) ry[valsreaded-1] = 0;
      //if      (ry[valsreaded] > MAX_CURVES_Y) ry[valsreaded] = MAX_CURVES_Y;
//    else if (ry[valsreaded] < 0)            ry[valsreaded] = 0;
//    if      (ry[valsreaded] > MAX_CURVES_Y)   ry[valsreaded] = MAX_CURVES_Y;
//    else if (ry[valsreaded] < MIN_CURVES_Y)   ry[valsreaded] = MIN_CURVES_Y;

    tft.drawLine(axispos[0]- ry[valsreaded-1], x_start - rx[valsreaded-1], axispos[0] - ry[valsreaded],   x_start - rx[valsreaded], color);
    tft.drawLine(axispos[1]- yflux[0],         x_start - rx[valsreaded-1], axispos[1] - yflux[1],         x_start - rx[valsreaded], ILI9341_MAGENTA);

  }
}

void check_alarms(){
  
    if (alarm_state>9) {
        digitalWrite(RED_LED,HIGH);
        digitalWrite(GREEN_LED,LOW);
        tft.setRotation(0);
        tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
        tft.setCursor(150, 40);   
        tft.println("VT AL");
        state_r=alarm_state-10;
    } else {
        digitalWrite(RED_LED,LOW);  
        state_r=alarm_state;
    }
    switch (state_r) {
        case NO_ALARM:
            if (alarm_state==0){ //state_r!=10
            digitalWrite(GREEN_LED,HIGH); digitalWrite(RED_LED,LOW);   }
          break;
        case PEEP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(RED_LED,HIGH);  
          tft.setRotation(0);
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(150, 20);   
          tft.println("PEEP AL");
        break;
        case PIP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(RED_LED,HIGH);      
          tft.setRotation(0);
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(150, 0);   
          tft.println("PIP AL");
      break;  
        case PEEP_PIP_ALARM:
          digitalWrite(GREEN_LED,LOW); digitalWrite(RED_LED,HIGH);      
          tft.setRotation(0);
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(150, 0);   
          tft.println("PIP AL");
          tft.setTextColor(ILI9341_RED); tft.setTextSize(2); 
          tft.setCursor(150, 20);   
          tft.println("PEEP AL");
      break;
      }
}

float calc_bat(const int &iter) {
  unsigned short count = iter;
  float level= 0.;
  float fdiv = (float)(BATDIV_R1 + BATDIV_R2)/(float)BATDIV_R2;
  float fac=1.1/1024.*fdiv;  //5./(1024.*0.175) //TODO: HACER AL COMIENZO
  
  for (int i=0;i<count;i++){
    level+=float(analogRead(PIN_BAT_LEV));
    //Serial.println(analogRead(PIN_BAT_LEV));
    }
  level*=fac/count;
  return level;
}

void print_float(const int &row, const int &col, const float &val) {
  dtostrf(val, 2, 1, buffer); //DEBUG
  //tft.setCursor(130, 260);tft.println("Bat:");
  tft.setCursor(col, row);tft.println(buffer);
  //tft.setCursor(100, 80);tft.println("Vmpx:");
}

void print_bat() {
    float level = 0.;
    tft.setRotation(0);
    //tft.fillRect(180,150,70,20, ILI9341_BLACK);//ONLY BAT LEVEL
    //TODO: Make this calcs at setup
    float fdiv = (float)(BATDIV_R1 + BATDIV_R2)/(float)BATDIV_R2;
    tft.fillRect(180,250,70,50, ILI9341_BLACK);
    float fac=1.1/1024.*fdiv;  //5./(1024.*0.175) //TODO: HACER AL COMIENZO
    
    //Vt > 24V   =>   PC = 100%
    //Vmin < Vt < 24V   =>   PC[%] = (Vt[V]-Vmin)/(24-Vmin)*100
    //Vt < Vmin   =>   PC = 0%
    unsigned short count = 5;
    for (int i=0;i<count;i++) {
        level+=float(analogRead(PIN_BAT_LEV));
    }

    level*=fac/count;

    dtostrf(level, 2, 1, buffer); //DEBUG
    tft.setCursor(130, 260);tft.println("Bat:");
    tft.setCursor(180, 260);tft.println(buffer);
    //tft.setCursor(220, 260);tft.println("%");

    dtostrf(level, 1, 2, buffer);
}

void print_vols() {
    tft.setRotation(0);
    tft.fillRect(40,LEGEND_Y,60,80, ILI9341_BLACK); //Here x is the first value (in the less width dimension)

    itoa(_mllastInsVol, buffer, 10);
    tft.setCursor(0, LEGEND_Y); //Before: 150,180 at right 
    tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
    tft.println("Vi: ");tft.setCursor(40, LEGEND_Y);tft.println(buffer); //Before 190,180
    
    itoa(_mllastExsVol, buffer, 10);
    tft.setCursor(0, LEGEND_Y + 20);
    tft.println("Ve: ");tft.setCursor(40, LEGEND_Y + 20);tft.println(buffer);
    
    itoa((_mllastInsVol + _mllastExsVol)/2, buffer, 10);
    tft.setCursor(0, LEGEND_Y + 40);
    tft.println("VT: ");tft.setCursor(40, LEGEND_Y + 40);tft.println(buffer);
 
}
