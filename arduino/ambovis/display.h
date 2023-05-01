#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "pinout.h"
#include "sensorcalculation.h"
#include "alarms.h"
#include "MechanicalVentilation.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

typedef struct tft_draw_data {
    Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
    byte valsreaded = 0;
    int yflux_0 = 0;
    int yflux_1 = 0;
    int rx0 = 0;
    int rx1 = 0;
    int ry0 = 0;
    int ry1 = 0;
    int16_t x_offset = 0;
    bool is_second_cycle = false;
    unsigned long lastShowSensor = 0;
    unsigned long print_bat_time = 0;
} Display_Data_t;

void init_display_tft(Display_Data_t& display);
void clean_tft(Display_Data_t& display);
void print_bat(Display_Data_t& display);
void print_float(Adafruit_ILI9341& tft, const int &row, const int &col, const float &val);
float calc_bat(const int &iter);
void tft_draw(Display_Data_t& display, SensorData& sensorData, Ventilation_Status_t& status, AlarmData& alarm_data);
#endif
