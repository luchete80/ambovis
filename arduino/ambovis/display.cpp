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

void tft_draw(void) {
    //Serial.println(cycle_pos);Serial.println(ry[valsreaded]);
    last_x=cycle_pos;
    rx[valsreaded]=cycle_pos;
    ry[valsreaded]=pressure_p*2.;     

    yflux[0]=yflux[1];yflux[1]=int(flow_f*0.035);
    yvt[0]=yvt[1];yvt[1]=int((_mlInsVol - _mlExsVol)*0.1);

    
    tft.setRotation(1);
    if (valsreaded > 0)
        drawY2(ILI9341_GREEN);
    valsreaded+=1;
    
  	if (last_x<5 && !lcd_cleaned){
    		lcd_cleaned=true;
    		valsreaded=0;
    		for (int i=0;i<3;i++) 
    		    valsreaded_[i]=0;
        print_vols();
        print_bat();
        tft.setRotation(1);
        tft.fillRect(0,0,60,100, ILI9341_BLACK); //FOR ALARMS, UPPER RIRHT
        tft.fillRect(0, 240 , 320, 10, ILI9341_GREEN);//x,y,lengthx,lentgthy

		} else {
		    lcd_cleaned=false;
		}


    
    check_alarms();
    
}//loop

void drawY2(uint16_t color){// THERE IS NO NEED TO REDRAW ALL IN EVERY FRAME WITH COLOR TFT

  if ( rx[valsreaded] > rx[valsreaded-1] ) {//to avoid draw entire line to the begining at the end of the cycle
          for (int i=0;i<3;i++)
            tft.drawLine(axispos[i], 240-rx[valsreaded-1], axispos[i], 240-rx[valsreaded], ILI9341_DARKGREY);
            tft.fillRect(0, 240 - rx[valsreaded] - 10, 320, 10, ILI9341_BLACK);//CLEAN PREVIOUS CURVE x,y,lengthx,lentgthy
            //tft.fillRect(0, 240 - rx[valsreaded-1] + 1, 320, rx[valsreaded]-rx[valsreaded-1], ILI9341_BLACK);//CLEAN PREVIOUS CURVE x,y,lengthx,lentgthy
            
            tft.drawLine(axispos[0]- ry[valsreaded-1], 240-rx[valsreaded-1], axispos[0] - ry[valsreaded],   240-rx[valsreaded], color);
            tft.drawLine(axispos[1]-yflux[0],           240-rx[valsreaded-1], axispos[1]-yflux[1],          240-rx[valsreaded], ILI9341_MAGENTA);
            tft.drawLine(axispos[2]-yvt[0],             240-rx[valsreaded-1], axispos[2]-yvt[1],            240-rx[valsreaded], ILI9341_BLUE);

  }
}

void check_alarms(){
  
      //Serial.println(state_r);
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
    switch (state_r){
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

void print_bat(){
    float level,level_perc;
    level=0.;
    tft.setRotation(0);
    //tft.fillRect(180,150,70,20, ILI9341_BLACK);//ONLY BAT LEVEL
    tft.fillRect(180,250,70,50, ILI9341_BLACK);    float fac=0.0279;  //5./(1024.*0.175)
    
    //Vt > 24V   =>   PC = 100%
    //Vmin < Vt < 24V   =>   PC[%] = (Vt[V]-Vmin)/(24-Vmin)*100
    //Vt < Vmin   =>   PC = 0%
    for (int i=0;i<40;i++){
        level+=float(analogRead(PIN_BAT_LEV));
        Serial.println(analogRead(PIN_BAT_LEV));
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
    tft.setCursor(150, 260);tft.println("Bat:");
    tft.setCursor(200, 260);tft.println(buffer);
    tft.setCursor(220, 260);tft.println("%");

    dtostrf(level, 1, 2, buffer);
    //Temporary
    //Serial.print("Bat level: ");Serial.println(level);
    tft.setCursor(150, 280);tft.println("Vo:");
    tft.setCursor(180, 280);tft.println(buffer);

}
void print_vols(){
    tft.setRotation(0);
    tft.fillRect(180,160,70,80, ILI9341_BLACK);
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
