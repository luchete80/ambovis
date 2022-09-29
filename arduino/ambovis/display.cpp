#include "display.h"
#include "MechVentilation.h"

#define MIN_CURVES_Y    60
#define CLEAN_Y         200
#define LEGEND_Y        260 //Begining of the legend on Y AXIS
#define ILI9341_DARKGREY 0x7BEF /* 128, 128, 128 */
bool lcd_cleaned=false;
int axispos[]={130,200}; //from each graph, from 0 to 320 (display height, IN PORTRAIT MODE)
byte state_r;

enum _state {NO_ALARM=0,PEEP_ALARM=1,PIP_ALARM=2,PEEP_PIP_ALARM=3};

byte valsreaded=0;
byte rx[128],ry[128];
int yflux[2];
int yvt[2];
char buffer[10];

void check_alarms(Adafruit_ILI9341& tft, short alarm_state);
void drawY2(Adafruit_ILI9341& tft, bool drawing_cycle, uint16_t color);
void print_vols(Adafruit_ILI9341& tft);
void printMessageWhenEndedWhileStepperMoving(Adafruit_ILI9341& tft);

void tft_draw(Adafruit_ILI9341& tft, SensorData& sensorData, bool& drawing_cycle, float fac, short alarm_state) {
    byte last_x=cycle_pos;
    rx[valsreaded]=cycle_pos;
    ry[valsreaded]=sensorData.pressure_p*2.;

    yflux[0]=yflux[1];
    yflux[1]=int(sensorData.flow_f*0.035);
    yvt[0]=yvt[1];
    yvt[1]=int((sensorData.ml_ins_vol - sensorData.ml_exs_vol)*0.1);

    tft.setRotation(1);
    if (valsreaded > 0) {
        drawY2(tft, drawing_cycle, ILI9341_GREEN);
    }
    valsreaded+=1;

    if (last_x>117 && !lcd_cleaned) {//NO PONER UN VALOR MENOR QUE 10
        lcd_cleaned=true;
        valsreaded=0;
        print_vols(tft);
        print_bat(tft, fac);
        
        tft.fillRect(10,10,150,15, ILI9341_BLACK);
        if (!digitalRead(PIN_POWEROFF)) {
            tft.setCursor(10, 10);
            tft.println("CORTE ENERGIA");
        }
        
        dtostrf(sensorData.v_level, 1, 2, buffer);
        tft.setCursor(100, 80);tft.println("Vmpx:");
        tft.fillRect(180,80,50,50, ILI9341_BLACK);
        tft.setCursor(180, 80);tft.println(buffer);
        
        drawing_cycle = !drawing_cycle;
        tft.fillRect(180,280,70,50, ILI9341_BLACK);

        printMessageWhenEndedWhileStepperMoving(tft);

        tft.setRotation(1);
        tft.fillRect(0, 0, 60, 100, ILI9341_BLACK); //FOR ALARMS, UPPER RIGHT
        tft.fillRect(0, 240, 320, 10, ILI9341_GREEN);//x,y,lengthx,lentgthy

    } else {
        lcd_cleaned=false;
    }

    check_alarms(tft, alarm_state);
}

void drawY2(Adafruit_ILI9341& tft, bool drawing_cycle, uint16_t color) {// THERE IS NO NEED TO REDRAW ALL IN EVERY FRAME WITH COLOR TFT
    int x_start = 240 - (int) drawing_cycle * 120;
    if ( rx[valsreaded] > rx[valsreaded-1] ) {//to avoid draw entire line to the begining at the end of the cycle
        for (int i=0; i < 2; i++) {
            tft.drawLine(axispos[i], x_start - rx[valsreaded-1], axispos[i], x_start - rx[valsreaded], ILI9341_DARKGREY); //X AXIS
        }
        tft.fillRect(MIN_CURVES_Y, x_start - rx[valsreaded] - 10, CLEAN_Y, 10, ILI9341_BLACK);   //CLEAN PREVIOUS CURVE x,y,lengthx,lentgthy
        if (ry[valsreaded] > 250 || ry[valsreaded-1] > 250 ) {
            ry[valsreaded-1] = ry[valsreaded] = 0;
        }

        int16_t x0 = axispos[0]- ry[valsreaded-1];
        int16_t y0 = x_start - rx[valsreaded-1];
        int16_t x1 = axispos[0] - ry[valsreaded];
        int16_t y1 = x_start - rx[valsreaded];
        tft.drawLine(x0, y0, x1, y1, color);

        int16_t j0 = axispos[1]- yflux[0];
        int16_t k0 = x_start - rx[valsreaded-1];
        int16_t j1 = axispos[1] - yflux[1];
        int16_t k1 = x_start - rx[valsreaded];
        tft.drawLine(j0, k0, j1, k1, ILI9341_MAGENTA);
    }
}

void printMessageWhenEndedWhileStepperMoving(Adafruit_ILI9341& tft) {
    if (ended_whilemov) {
        tft.setCursor(150, 300);tft.println("ENDErr");
    } else {
        tft.setCursor(150, 300);tft.println("ENDOk");
    }
}

void showVTAlarm(Adafruit_ILI9341& tft) {
    digitalWrite(RED_LED,HIGH);
    digitalWrite(GREEN_LED,LOW);
    tft.setRotation(0);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(2);
    tft.setCursor(150, 40);
    tft.println("VT AL");
}

void showPeepAlarm(Adafruit_ILI9341& tft) {
    digitalWrite(GREEN_LED,LOW);
    digitalWrite(RED_LED,HIGH);
    tft.setRotation(0);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(2);
    tft.setCursor(150, 20);
    tft.println("PEEP AL");
}

void showPipAlarm(Adafruit_ILI9341& tft) {
    digitalWrite(GREEN_LED,LOW);
    digitalWrite(RED_LED,HIGH);
    tft.setRotation(0);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(2);
    tft.setCursor(150, 0);
    tft.println("PIP AL");
}

void check_alarms(Adafruit_ILI9341& tft, short alarm_state) {
    if (is_alarm_vt_on) {
        showVTAlarm(tft);
    } else {
        digitalWrite(RED_LED,LOW);
    }
    switch (alarm_state) {
        case NO_ALARM:
            if (!is_alarm_vt_on) {
                digitalWrite(GREEN_LED,HIGH);
                digitalWrite(RED_LED,LOW);
            }
            break;
        case PEEP_ALARM:
            showPeepAlarm(tft);
            break;
        case PIP_ALARM:
            showPipAlarm(tft);
            break;
        case PEEP_PIP_ALARM:
            showPipAlarm(tft);
            showPeepAlarm(tft);
            break;
    }
}

float calc_bat(const int &iter, float fac) {
    unsigned short count = iter;
    float level= 0.;

    for (int i=0;i<count;i++) {
        level+=float(analogRead(PIN_BAT_LEV));
    }
    level*=fac/count;
    return level;
}

void print_float(Adafruit_ILI9341& tft, const int &row, const int &col, const float &val) {
    dtostrf(val, 2, 1, buffer);
    tft.setCursor(col, row);
    tft.println(buffer);
}

void print_bat(Adafruit_ILI9341& tft, float fac) {
    tft.setRotation(0);
    tft.fillRect(180,250,70,50, ILI9341_BLACK);
    //Vt > 24V   =>   PC = 100%
    //Vmin < Vt < 24V   =>   PC[%] = (Vt[V]-Vmin)/(24-Vmin)*100
    //Vt < Vmin   =>   PC = 0%
    float level = calc_bat(BATTERY_READ, fac);

    dtostrf(level, 2, 1, buffer);
    tft.setCursor(130, 260);tft.println("Bat:");
    tft.setCursor(180, 260);tft.println(buffer);
}

void print_vols(Adafruit_ILI9341& tft) {
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
