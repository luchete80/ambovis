#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "pinout.h"
#include "defaults.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define ILI9341_LIGHTGREY 0xC618 /* 192, 192, 192 */
#define ILI9341_DARKGREY 0x7BEF /* 128, 128, 128 */

//#define DEBUG 1
//enum serialpos={ALARM_=0,TIME_,PRESSURE_,FLUX_,VT_};
//enum serialpos {TIME_=0,PRESSURE_,FLUX_,VT_,ALARM_};//ORIGINAL

// TODO MOVE TO SOME CLASSS
extern byte valsreaded;
extern byte last_x;
extern bool tft_cleaned;
extern int last_vals[7][2];;
extern int xgra[5][2];

//SPI HARDWARE SHOULD BE PINS CLK #13 AND MISO #11, WITH THIS CONSTRUCTOR!
extern Adafruit_ILI9341 tft; //= Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);


void drawY2(uint16_t color);
void print_bat();
void tft_draw();
void print_vols();
void check_alarms();
#endif
