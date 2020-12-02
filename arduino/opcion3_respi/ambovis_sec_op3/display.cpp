#include "display.h"
#include "MechVentilation.h"
#include "Serial.h"

//bool lcd_cleaned=false;

unsigned long time_last_show=0;

char a[10],b[10];

bool state_chg;

int last_t;
int integerFromPC [5];
float floatFromPC = 0.0;
int axispos[]={100,170,300}; //from each graph
//byte state_r;
int buzzer=3; //pin

#include "Serial.h"

#ifdef DISPLAY_SEC

float diff_var[]={35., 20., 800.,0.,300.}; 

byte rx[128],ry[128];
int  ry2[128];
int yflux[2];
int yvt[2];
char buffer[10];
int yp[2];
int vi,ve,vt;
bool ve_readed,vi_readed;

#endif
void print_params();
void print_press();
void parseNfilterData();

//enum _state {NO_ALARM=0,PEEP_ALARM=1,PIP_ALARM=2,PEEP_PIP_ALARM=3};

#define NO_ALARM        0
#define PEEP_ALARM      1
#define PIP_ALARM       2
#define PEEP_PIP_ALARM  3

bool wait4statechg=false;
//_state state;
int state,pre_state,state_r,last_state;//pre_state is to verification

char recvChar;
char endMarker = '>';


int valsreaded_[3];
//byte last_x=0;


int count=0;
byte escala=32;
byte x[128],y[64];




void tft_draw(void) {
    //Serial.println(cycle_pos);Serial.println(ry[valsreaded]);

    parseNfilterData();
    last_x=cycle_pos;
    //last_x=integerFromPC[TIME_];
    //rx[valsreaded]=cycle_pos;
    rx[valsreaded]=integerFromPC[TIME_];

    //ry[valsreaded]=pressure_p*2.;     
//    if (integerFromPC[P_]!= 0){
//      Serial.print("P leido: ");Serial.println(integerFromPC[P_]);
//      ry[valsreaded]=integerFromPC[P_]*2.;    
//      valsreaded+=1;
//      last_x=integerFromPC[TIME_];
//    }


    Serial.print("flux: ");Serial.println(yflux[1]);
    yvt[0]=yvt[1];yvt[1]=int((_mlInsVol - _mlExsVol)*0.1);

    
    tft.setRotation(1);
    if (valsreaded > 0)
        drawY2();
    
  	if (last_x<5 && !tft_cleaned){
        //#ifdef DEBUG_UPDATE
        Serial.print("last_x: ");Serial.println(last_x);
        Serial.println("Cleaning");
        //#endif
    		tft_cleaned=true;
    		valsreaded=0;
    		for (int i=0;i<3;i++) 
    		    valsreaded_[i]=0;
        print_vols();
        print_bat();
        tft.setRotation(1);
        tft.fillRect(0,0,60,100, ILI9341_BLACK); //FOR ALARMS, UPPER RIRHT
        tft.fillRect(0, 240 , 320, 10, ILI9341_GREEN);//x,y,lengthx,lentgthy
        print_params();
        print_press();

		} else {
		    tft_cleaned=false;
		}


    
    check_alarms();
    
    
    
}//loop

void drawY2(){// THERE IS NO NEED TO REDRAW ALL IN EVERY FRAME WITH COLOR TFT
  #ifdef DEBUG_UPDATE
  Serial.print("Valsreaded: ");Serial.println(valsreaded);
  //Serial.print("rx(valsreaded) & rx(valsreaded-1): ");Serial.print(rx[valsreaded]);Serial.print(",");Serial.print(rx[valsreaded-1]);Serial.print(",");
  #endif
  if ( rx[valsreaded] > rx[valsreaded-1] ) {//to avoid draw entire line to the begining at the end of the cycle
          for (int i=0;i<3;i++)
            tft.drawLine(axispos[i], 240-rx[valsreaded-1], axispos[i], 240-rx[valsreaded], ILI9341_DARKGREY); //Ejes
            tft.fillRect(0, 240 - rx[valsreaded] - 10, 320, 10, ILI9341_BLACK);//CLEAN PREVIOUS CURVE x,y,lengthx,lentgthy
           
            tft.drawLine(axispos[0]-yp[0],              240-rx[valsreaded-1], axispos[0]-yp[1],             240-rx[valsreaded], ILI9341_GREEN);
            tft.drawLine(axispos[1]-yflux[0],           240-rx[valsreaded-1], axispos[1]-yflux[1],          240-rx[valsreaded], ILI9341_MAGENTA);
            //tft.drawLine(axispos[2]-yvt[0],             240-rx[valsreaded-1], axispos[2]-yvt[1],            240-rx[valsreaded], ILI9341_BLUE);

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
    tft.fillRect(180,300,70,20, ILI9341_RED);    float fac=0.0279;  //5./(1024.*0.175)
    
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
    tft.setCursor(130, 300);tft.println("Bat:");
    tft.setCursor(180, 300);tft.println(buffer);
    tft.setCursor(220, 300);tft.println("%");

    dtostrf(level, 1, 2, buffer);
    //Temporary
    //Serial.print("Bat level: ");Serial.println(level);
    //tft.setCursor(150, 280);tft.println("Vo:");
    //tft.setCursor(180, 280);tft.println(buffer);

}
void print_press(){
    tft.setRotation(0);
    tft.fillRect(180,100,70,20, ILI9341_BLACK);
    //itoa(integerFromPC[5], buffer, 10);
    itoa(last_pressure_min, buffer, 10);
    tft.setCursor(150, 100);  //Y es la posicion vertical aca
    tft.setTextColor(ILI9341_RED);  tft.setTextSize(2);
    tft.println("PEEP: ");tft.setCursor(200, 100);tft.println(buffer);
  }


void print_params(){
    tft.setRotation(0);
    tft.fillRect(180,120,70,80, ILI9341_BLUE);
    //itoa(integerFromPC[5], buffer, 10);
    itoa(options.respiratoryRate, buffer, 10);
    tft.setCursor(150, 120);  //Y es la posicion vertical aca
    tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
    tft.println("ti: ");tft.setCursor(200, 120);tft.println(buffer);
  }
void print_vols(){
    tft.setRotation(0);
    tft.fillRect(180,220,70,80, ILI9341_RED);
    //itoa(integerFromPC[5], buffer, 10);
    itoa(_mllastInsVol, buffer, 10);
    tft.setCursor(150, 220);
    tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
    tft.println("Vi: ");tft.setCursor(190, 220);tft.println(buffer);


    
    itoa(_mllastExsVol, buffer, 10);
    tft.setCursor(150, 240);
    tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
    tft.println("Ve: ");tft.setCursor(190, 240);tft.println(buffer);
    
    itoa((_mllastInsVol + _mllastExsVol)/2, buffer, 10);
    tft.setCursor(150, 260);
    tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
    tft.println("VT: ");tft.setCursor(190, 260);tft.println(buffer);
 
  }





void parseNfilterData() {
  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(receivedChars, ","); // this continues where the previous call left off
  integerFromPC[0] = atoi(strtokIndx);     // convert this part to an integer

  for (int i=1;i<7;i++) {
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      integerFromPC[i] = atoi(strtokIndx);     // convert this part to an integer
  }
  
   //Serial.print("integerFromPC[TIME_]");Serial.print(integerFromPC[TIME_]);Serial.print(" - "); Serial.print("last_x: ");Serial.println(last_x);
   if ( integerFromPC[TIME_] != last_x /*&& abs(integerFromPC[P_])<ry[valsreaded]+10 */&& integerFromPC[0] < 127 && integerFromPC[TIME_] < last_x+20) {
     valsreaded+=1;
     last_x=integerFromPC[TIME_];
     rx[valsreaded]=integerFromPC[TIME_];
     ry[valsreaded]=integerFromPC[P_];     
   }
  //Serial.print("time y xgra");Serial.print(integerFromPC[TIME_]);Serial.print(",");Serial.print(xgra[P_][1]);
  
  if ( integerFromPC[P_] != 0 && abs(integerFromPC[P_]) < ( abs(last_vals[P_][1])+diff_var[P_] ) /*&& integerFromPC[TIME_] > last_x  && integerFromPC[TIME_] > xgra[P_][1]*/ ) {
    Serial.print("yp0 y 1: ");Serial.print(yp[0]);Serial.print(",");Serial.print(yp[1]);Serial.print(", lastvals(p,1)");Serial.println(last_vals[P_][1]);
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
  //Serial.print("time y xgra flux");Serial.print(integerFromPC[TIME_]);Serial.print(",");Serial.print(xgra[FLUX_][1]);
  if ( integerFromPC[FLUX_] != 0 && abs(integerFromPC[FLUX_]) < abs(last_vals[FLUX_][1])+diff_var[FLUX_] /* && integerFromPC[TIME_] > xgra[FLUX_][1]*/) {
    //yflux[0]=yflux[1];yflux[1]=int(float(integerFromPC[FLUX_])*0.04);
    yflux[0]=yflux[1];yflux[1]=int(float(integerFromPC[FLUX_]-127)*6*0.04);//SI VIENE COMO BYTE
    Serial.print("yflux: ");Serial.println(yflux[1]);
    last_vals[FLUX_][0]=last_vals[FLUX_][1];last_vals[FLUX_][1]=integerFromPC[FLUX_];
    xgra[FLUX_][0]=xgra[FLUX_][1];xgra[FLUX_][1]=integerFromPC[TIME_];
  }
   if ( integerFromPC[VT_] != 0 && abs(integerFromPC[VT_]) < abs(last_vals[VT_][1])+diff_var[VT_] && integerFromPC[TIME_] > xgra[VT_][1]) {
    yvt[0]=yvt[1];yvt[1]=int(float(integerFromPC[VT_])*0.07);
    last_vals[VT_][0]=last_vals[VT_][1];last_vals[VT_][1]=integerFromPC[VT_];
    xgra[VT_][0]=xgra[VT_][1];xgra[VT_][1]=integerFromPC[TIME_];
  }
    
    //Serial.print("integers: ");Serial.print(integerFromPC[0]);Serial.print(",");Serial.print(integerFromPC[1]);Serial.print(",");Serial.print(integerFromPC[2]);Serial.print(",");Serial.println(integerFromPC[3]);
}
