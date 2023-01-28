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
#define TIME_     0
#define PRESSURE_ 1
#define FLUX_     2
#define VT_       4
#define ALARM_    3

//SPI HARDWARE SHOULD BE PINS CLK #13 AND MISO #11, WITH THIS CONSTRUCTOR!
extern Adafruit_ILI9341 tft; //= Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
extern bool drawing_cycle;//TODO: MOVE TO CLASS MEMBER
void drawY2(uint16_t color);
void print_bat();
void print_float(const int &row, const int &col, const float &val);
float calc_bat(const int &iter);
void tft_draw();
void print_vols();
void check_alarms();

extern byte cycle_pos;
extern float pressure_p;
extern float _flux, flow_f;
extern float _mlInsVol,_mlExsVol;
extern int _mllastInsVol,_mllastExsVol;
extern bool ended_whilemov;

#endif
