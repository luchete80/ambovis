#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "pinout.h"
#include "sensorcalculation.h"
#include "alarms.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

extern AlarmData alarm_data;

void init_display_tft(Adafruit_ILI9341& tft);
void clean_tft(Adafruit_ILI9341& tft);
void print_bat(Adafruit_ILI9341& tft, float fac);
void print_float(Adafruit_ILI9341& tft, const int &row, const int &col, const float &val);
float calc_bat(const int &iter, float fac);
void tft_draw(Adafruit_ILI9341& tft, SensorData& sensorData, bool& drawing_cycle, float fac, short alarm_state);
#endif
