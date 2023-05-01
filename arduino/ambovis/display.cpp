#include "display.h"

char buffer[10];

void show_alarms(Adafruit_ILI9341& tft, AlarmData& alarm_data);
bool drawCycles(Display_Data_t& display);
void print_vols(Adafruit_ILI9341& tft, Ventilation_Status_t status);

void clean_tft(Display_Data_t& display) {
    display.tft.fillScreen(ILI9341_BLACK);
}

void init_empty_tft(Display_Data_t& display) {
    digitalWrite(TFT_SLEEP, HIGH);
    display.tft.begin();
    clean_tft(display);
}

void init_display_tft(Display_Data_t& display) {
    init_empty_tft(display);
    display.tft.setTextColor(ILI9341_BLUE);
    display.tft.setTextSize(4);
    display.tft.setCursor(10, 40);     display.tft.println("RespirAR");
    display.tft.setCursor(10, 80);     display.tft.println("FIUBA");
}

void print_poweroff_msg(Adafruit_ILI9341& tft) {
    tft.fillRect(10, 10, 130, 15, ILI9341_BLACK);
    tft.setCursor(10, 10);
    tft.setTextSize(1);
    if (!digitalRead(PIN_POWEROFF)) {
        tft.setCursor(10, 10);
        tft.println("CORTE ENERGIA");
    }
}

void print_vmpx(Adafruit_ILI9341& tft, float vlevel) {
    tft.setTextSize(2);
    dtostrf(vlevel, 1, 2, buffer);
    tft.setCursor(25, 30);
    tft.println("Vmpx:");
    tft.fillRect(80, 30, 50, 20, ILI9341_BLACK);
    tft.setCursor(80, 30);
    tft.println(buffer);
}

void print_error(Adafruit_ILI9341& tft, bool ended_whilemov) {
    tft.fillRect(180, 280, 70, 50, ILI9341_BLACK);
    tft.setCursor(150, 300);
    tft.println(ended_whilemov ? "ENDErr" : "ENDOk");
}

void init_new_cycle(Display_Data_t& display) {
    display.is_second_cycle = !display.is_second_cycle;
    display.x_offset = display.is_second_cycle ? 120 : 0;
    //clean beginning of next cycle
    display.tft.fillRect(display.x_offset, AY0 - CLEAN_Y_UP, display.rx1, CLEAN_Y_DOWN, ILI9341_BLACK);
    display.tft.fillRect(display.x_offset, AY1 - CLEAN_Y_UP, display.rx1, CLEAN_Y_DOWN, ILI9341_BLACK);
    display.valsreaded = 0;
}

void tft_draw(Display_Data_t& display, SensorData& sensorData, Ventilation_Status_t& status, AlarmData& alarm_data) {
    display.rx0 = display.rx1;
    display.rx1 = status.cycle_pos;
    display.ry0 = display.ry1;
    display.ry1 = sensorData.pressure_p*2.;

    display.yflux_0 = display.yflux_1;
    display.yflux_1 = int(sensorData.flow_f*0.035);

    if (display.valsreaded > 0) {
        bool is_same_cycle = drawCycles(display);
        if (!is_same_cycle) {
            init_new_cycle(display);
            print_vols(display.tft, status);
            print_bat(display);
            print_poweroff_msg(display.tft);
            print_vmpx(display.tft, sensorData.v_level);
            print_error(display.tft, status.ended_while_moving);
        }
    }
    display.valsreaded+=1;
    show_alarms(display.tft, alarm_data);
}

bool drawCycles(Display_Data_t& display) {
    display.tft.drawLine(0, AY0, END_X, AY0, ILI9341_DARKGREY);
    display.tft.drawLine(0, AY1, END_X, AY1, ILI9341_DARKGREY);
    if (display.rx1 > 117) {
        return true;
    }
    if (display.rx1 > display.rx0) {
        int16_t x0 = display.x_offset + display.rx0;
        display.tft.fillRect(x0, AY0 - CLEAN_Y_UP, CLEAN_X, CLEAN_Y_DOWN, ILI9341_BLACK);
        int16_t y0 = AY0 - display.ry0;
        int16_t x1 = display.x_offset + display.rx1;
        int16_t y1 = AY0 - display.ry1;
        display.tft.drawLine(x0, y0, x1, y1, ILI9341_GREEN);

        int16_t j0 = display.x_offset + display.rx0;
        display.tft.fillRect(j0, AY1 - CLEAN_Y_UP, CLEAN_X, CLEAN_Y_DOWN, ILI9341_BLACK);
        int16_t k0 = AY1 - display.yflux_0;
        int16_t j1 = display.x_offset + display.rx1;
        int16_t k1 = AY1 - display.yflux_1;
        display.tft.drawLine(j0, k0, j1, k1, ILI9341_MAGENTA);
        return true;
    }
    return false;
}
void config_alarms(Adafruit_ILI9341& tft) {
    digitalWrite(RED_LED,HIGH);
    digitalWrite(GREEN_LED,LOW);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(2);
}
void showVTAlarm(Adafruit_ILI9341& tft) {
    config_alarms(tft);
    tft.setCursor(ALARM_POS_X, 40);
    tft.println("VT AL");
}

void showPeepAlarm(Adafruit_ILI9341& tft) {
    config_alarms(tft);
    tft.setCursor(ALARM_POS_X, 20);
    tft.println("PEEP AL");
}

void showPipAlarm(Adafruit_ILI9341& tft) {
    config_alarms(tft);
    tft.setCursor(ALARM_POS_X, 0);
    tft.println("PIP AL");
}

void show_alarms(Adafruit_ILI9341& tft, AlarmData& alarm_data) {
    if (alarm_data.is_alarm_vt_on) {
        showVTAlarm(tft);
    } else {
        digitalWrite(RED_LED,LOW);
    }
    switch (alarm_data.alarm_state) {
        case NO_ALARM:
            if (!alarm_data.is_alarm_vt_on) {
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

float calc_bat(const int &iter) {
    unsigned short count = iter;
    float level= 0.;

    for (int i=0;i<count;i++) {
        level+=float(analogRead(PIN_BAT_LEV));
    }
    level*=FAC/count;
    return level;
}

void print_float(Adafruit_ILI9341& tft, const int &row, const int &col, const float &val) {
    dtostrf(val, 2, 1, buffer);
    tft.setCursor(col, row);
    tft.println(buffer);
}

void print_bat(Display_Data_t& display) {
    display.tft.fillRect(BAT_VAL_X,250,70,50, ILI9341_BLACK);
    //Vt > 24V   =>   PC = 100%
    //Vmin < Vt < 24V   =>   PC[%] = (Vt[V]-Vmin)/(24-Vmin)*100
    //Vt < Vmin   =>   PC = 0%
    float level = calc_bat(BATTERY_READ);

    dtostrf(level, 2, 1, buffer);
    display.tft.setCursor(BAT_X, BAT_Y); display.tft.println("Bat:");
    display.tft.setCursor(BAT_VAL_X, BAT_Y); display.tft.println(buffer);
}

void print_vols(Adafruit_ILI9341& tft, Ventilation_Status_t vent_status) {
    tft.fillRect(LEGEND_X, LEGEND_Y,60,80, ILI9341_BLACK); //Here x is the first value (in the less width dimension)

    itoa(vent_status.ml_last_ins_vol, buffer, 10);
    tft.setCursor(0, LEGEND_Y); //Before: 150,180 at right 
    tft.setTextColor(ILI9341_ORANGE);  tft.setTextSize(2);
    tft.println("Vi: ");tft.setCursor(LEGEND_X, LEGEND_Y);tft.println(buffer); //Before 190,180
    
    itoa(vent_status.ml_last_exp_vol, buffer, 10);
    tft.setCursor(0, LEGEND_Y + 20);
    tft.println("Ve: ");tft.setCursor(LEGEND_X, LEGEND_Y + 20);tft.println(buffer);
    
    itoa((vent_status.ml_last_ins_vol + vent_status.ml_last_exp_vol)/2, buffer, 10);
    tft.setCursor(0, LEGEND_Y + 40);
    tft.println("VT: ");tft.setCursor(LEGEND_X, LEGEND_Y + 40);tft.println(buffer);
}
