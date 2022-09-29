#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "pinout.h"
#include "defaults.h"
#include "sensorcalculation.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

extern bool is_alarm_vt_on;
void print_bat(Adafruit_ILI9341& tft, float fac);
void print_float(Adafruit_ILI9341& tft, const int &row, const int &col, const float &val);
float calc_bat(const int &iter, float fac);
void tft_draw(Adafruit_ILI9341& tft, SensorData& sensorData, bool& drawing_cycle, float fac, short alarm_state);
#endif
