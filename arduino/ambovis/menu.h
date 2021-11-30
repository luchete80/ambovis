#ifndef _MENU_H_
#define _MENU_H_

#include "defaults.h"
#include "Sensors.h"
#if TESTING_MODE_DISABLED
#ifdef LCD_I2C
#include "src/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#else
#include <LiquidCrystal.h>
#endif

#ifdef LCD_I2C
extern LiquidCrystal_I2C lcd;
#else
extern LiquidCrystal lcd;
#endif
#endif //TESTING_MODE_DISABLED

typedef struct {
    unsigned long lastButtonPress;
    int bck_state;
    int last_bck_state ; // previous state of the button
    int startPressed ;    // the moment the button was pressed
    int endPressed ;      // the moment the button was released
    int holdTime ;        // how long the button was hold
    bool change_sleep;
    int pressed=0;  //0 nothing , 1 enter, 2 bck
    int curr_sel= 1; //COMPRESSION
    int old_curr_sel = 1;
    byte encoderPos = 1; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
    byte oldEncPos = 1; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
    byte menu_number = 0;
    bool isitem_sel;
    bool show_changed_options; //Only for display
    bool update_options;
    byte p_trim;
    bool clear_all_display;
} MenuState;

void init_display();
void writeLine(int line, String message = "", int offsetLeft = 0);
void check_encoder(MenuState& menuState, SystemState& systemState, SensorParams& sensorParams);
void display_lcd(MenuState& menuState, byte ventilationMode, SensorParams& sensorParams);
void check_bck_state(MenuState& menuState, SystemState& systemState);

#endif