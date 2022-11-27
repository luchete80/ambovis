#ifndef _MENU_H_
#define _MENU_H_

#include "defaults.h"
#include <LiquidCrystal.h>
#include "MechVentilation.h"
#include <arduino.h>
#include "pinout.h"

static byte MAIN_MENU = 0;
static byte PARAMETERS_MENU = 1;
static byte ALARMS_MENU = 2;
static byte SETTINGS_MENU = 3;
extern LiquidCrystal lcd;
//extern byte max_sel,min_sel; //According to current selection
//extern unsigned long lastButtonPress;

typedef struct keyboard_data {
    unsigned long last_button_pressed;
    int bck_state;     // current state of the back button
    int last_bck_state; // previous state of the button
    int start_pressed;    // the moment the button was pressed
    int end_pressed;      // the moment the button was released
    int hold_time;        // how long the button was hold
    byte selection;
    byte old_selection;
    bool is_item_selected;
    bool change_sleep;
    int pressed;
} Keyboard_data_t;

typedef struct menu_state {
    bool show_changed_options; // Includes cursor change
    bool update_options; // only when a parameter value is modified
    byte menu_number;
    byte menu_position;
    byte old_menu_position;
    bool initial_menu_completed;
    bool is_initial_menu;
} Menu_state_t;

//extern bool show_changed_options; //Only for display
//extern bool update_options;
//extern byte menu_number;
//extern byte menu_position;
//extern byte old_menu_position;
//extern bool initial_menu_completed;

void writeLine(int line, String message = "", int offsetLeft = 0);
void check_encoder(Keyboard_data_t& keyboard_data, Menu_state_t& menu_state, unsigned long time);
void display_lcd(Keyboard_data_t& keyboard_data, Menu_state_t& menu_state);
void init_display();
void initialize_menu(Keyboard_data_t& keyboard_data, Menu_state_t& menu_state);
void check_bck_state(Keyboard_data_t& keyboard_data, unsigned long time);

//void check_updn_button(int pin, byte *var, bool incr_decr);


#endif
