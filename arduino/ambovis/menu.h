#ifndef _MENU_H_
#define _MENU_H_

#include "defaults.h"
#include <LiquidCrystal.h>
#include "MechVentilation.h"
#include <arduino.h>
#include "pinout.h"

extern LiquidCrystal lcd;
//extern byte max_sel,min_sel; //According to current selection
//extern unsigned long lastButtonPress;

typedef struct keyboard_data {
    unsigned long last_button_pressed;
    int bck_state;     // current state of the button
    int last_bck_state; // previous state of the button
    int start_pressed;    // the moment the button was pressed
    int end_pressed;      // the moment the button was released
    int hold_time;        // how long the button was hold
//    int idleTime;        // how long the button was idle
    int curr_sel;
    int old_curr_sel;
    byte selection;
    byte old_selection;
    char temp_str[5];
    byte max_sel;
    byte min_sel;
    bool is_item_selected;
    bool change_sleep;
    int pressed;
} Keyboard_data_t;

//extern int curr_sel, old_curr_sel;
//extern byte encoderPos; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
//extern byte oldEncPos; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
extern bool show_changed_options; //Only for display
extern bool update_options;
//extern char tempstr[5];
//extern bool isitem_sel;
extern byte menu_number;
extern byte p_trim;

void writeLine(int line, String message = "", int offsetLeft = 0);
void check_encoder(Keyboard_data_t& keyboard_data, unsigned long time);
void display_lcd(Keyboard_data_t keyboard_data);
void init_display();
//void check_bck_state(Keyboard_data_t keyboard_data);

//void check_updn_button(int pin, byte *var, bool incr_decr);


#endif
