#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "pinout.h"
#include "defaults.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define ILI9341_DARKGREY 0x7BEF /* 128, 128, 128 */

//SPI HARDWARE SHOULD BE PINS CLK #13 AND MISO #11, WITH THIS CONSTRUCTOR!
//extern Adafruit_ILI9341 tft; //= Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
extern bool drawing_cycle;//TODO: MOVE TO CLASS MEMBER
void print_bat(Adafruit_ILI9341& tft, float fac);
void print_float(Adafruit_ILI9341& tft, const int &row, const int &col, const float &val);
float calc_bat(const int &iter, float fac);
void tft_draw(Adafruit_ILI9341& tft, float fac);
#endif
