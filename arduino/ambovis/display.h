#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "pinout.h"
#include "defaults.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define ILI9341_LIGHTGREY 0xC618 /* 192, 192, 192 */
#define ILI9341_DARKGREY 0x7BEF /* 128, 128, 128 */

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
void tft_draw();
void print_vols();
void check_alarms();
#endif
