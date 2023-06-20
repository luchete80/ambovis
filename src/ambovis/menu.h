#ifndef _MENU_H_
#define _MENU_H_

#include "alarms.h"
#include <Arduino.h>
#include <LiquidCrystal.h>
#include "MechanicalVentilation.h"

static byte MENU_NUMBER_LIST[] = {PARAMETERS_MENU, ALARMS_MENU};
extern LiquidCrystal lcd;

typedef struct keyboard_data {
    unsigned long last_button_pressed;
    int bck_state;     // current state of the back button
    int last_bck_state; // previous state of the button
    unsigned long start_pressed;    // the moment the button was pressed
    int hold_time;        // how long the button was hold
    uint8_t selection;
    uint8_t old_selection;
    bool change_sleep;
    byte pressed;
} Keyboard_data_t;

typedef struct menu_state {
    uint8_t edited_value;
    uint8_t* value_to_edit;
    bool is_item_selected;
    bool show_changed_options; // Includes cursor change
    bool update_options; // only when a parameter value is modified
    uint8_t menu_number;
    uint8_t menu_position;
    uint8_t old_menu_position;
    bool initial_menu_completed;
    bool is_initial_menu;
    bool clear_display;
} Menu_state_t;

void init_display_lcd();
void initialize_menu(Keyboard_data_t& keyboard_data, Menu_state_t& menu_state, Ventilation_Config_t& config);
void writeLine(int line, String message = "", int offsetLeft = 0);
void check_buttons(Keyboard_data_t& keyboard_data, unsigned long time);
void check_encoder(Keyboard_data_t& keyboard_data, Menu_state_t& menu_state,
                   Ventilation_Config_t& config, AlarmData& alarm_data, unsigned long time);
void display_lcd(Menu_state_t& menu_state, Ventilation_Config_t& config,
                 Ventilation_Status_t& status, AlarmData& alarm_data);
void check_bck_state(Keyboard_data_t& keyboard_data, unsigned long time);
void show_calibration_cycle(byte calib_cycle);
void wait_for_flux_disconnected();

#endif
