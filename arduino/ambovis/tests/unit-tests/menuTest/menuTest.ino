#include <AUnit.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "../../../alarms.h"
#include "../../../MechanicalVentilation.h"
#include "../../../menu.h"

Keyboard_data_t keyboard_data;
Menu_state_t menu_state;
Ventilation_Config_t config;
Ventilation_Status_t status;
AlarmData alarm_data;

test(check_encoder_change_menu_position_test) {
    menu_state.menu_number = PARAMETERS_MENU;
    menu_state.menu_position = 1;
    menu_state.is_initial_menu = false;

    keyboard_data.selection = 2; // next position
    keyboard_data.pressed = 0;
    keyboard_data.last_button_pressed = 0;
    unsigned long current_time = 300;

    check_encoder(keyboard_data, menu_state, config, alarm_data, current_time);

    assertEqual(menu_state.menu_number, PARAMETERS_MENU);
    assertEqual(menu_state.old_menu_position, 1);
    assertEqual(menu_state.menu_position, 2);
    assertEqual(menu_state.edited_value, keyboard_data.selection);
    assertEqual(menu_state.show_changed_options, true); // changed position
}

test(check_encoder_select_menu_position_test) {
    config.respiratory_rate = 25;
    menu_state.menu_number = PARAMETERS_MENU;
    menu_state.menu_position = 1; // BPM
    menu_state.is_initial_menu = false;

    keyboard_data.pressed = ENTER_PRESSED;
    keyboard_data.last_button_pressed = 0;
    unsigned long current_time = 300;

    check_encoder(keyboard_data, menu_state, config, alarm_data, current_time);

    assertEqual(menu_state.menu_number, PARAMETERS_MENU);
    assertEqual(*menu_state.value_to_edit, config.respiratory_rate);
    assertEqual(keyboard_data.selection, config.respiratory_rate);
    assertEqual(menu_state.is_item_selected, true);
}

test(check_encoder_select_menu_position_main_menu_test) {
    menu_state.menu_number = MAIN_MENU;
    menu_state.menu_position = 1; // ALARMS
    menu_state.edited_value = 1;
    menu_state.is_initial_menu = false;
    menu_state.is_item_selected = false;

    keyboard_data.pressed = ENTER_PRESSED;
    keyboard_data.last_button_pressed = 0;
    unsigned long current_time = 300;

    check_encoder(keyboard_data, menu_state, config, alarm_data, current_time);

    assertEqual(menu_state.menu_number, ALARMS_MENU);
    assertEqual(menu_state.menu_position, 0);
    assertEqual(menu_state.clear_display, true);
}

test(check_encoder_change_selected_value_test) {
    config.respiratory_rate = 25;
    menu_state.menu_number = PARAMETERS_MENU;
    menu_state.menu_position = 1;
    menu_state.is_item_selected = true;
    menu_state.is_initial_menu = false;

    keyboard_data.pressed = 0;
    keyboard_data.selection = 24;
    keyboard_data.last_button_pressed = 0;
    unsigned long current_time = 300;

    check_encoder(keyboard_data, menu_state, config, alarm_data, current_time);

    assertEqual(menu_state.edited_value, keyboard_data.selection);
}

test(check_encoder_update_selected_value_test) {
    config.respiratory_rate = 25;
    menu_state.menu_number = PARAMETERS_MENU;
    menu_state.menu_position = 1;
    menu_state.is_item_selected = true;
    menu_state.is_initial_menu = false;

    keyboard_data.pressed = ENTER_PRESSED;
    keyboard_data.selection = 24;
    keyboard_data.last_button_pressed = 0;
    unsigned long current_time = 300;

    check_encoder(keyboard_data, menu_state, config, alarm_data, current_time);

    assertEqual(config.respiratory_rate, 24);
    assertEqual(keyboard_data.selection, menu_state.menu_position);
    assertEqual(menu_state.is_item_selected, false);
    assertEqual(menu_state.update_options, true);
}

test(check_encoder_press_back_without_updating_selected_value_test) {
    config.respiratory_rate = 25;
    menu_state.menu_number = PARAMETERS_MENU;
    menu_state.menu_position = 1;
    menu_state.is_item_selected = true;
    menu_state.is_initial_menu = false;

    keyboard_data.pressed = BACK_PRESSED;
    keyboard_data.selection = 24;
    keyboard_data.last_button_pressed = 0;
    unsigned long current_time = 300;

    check_encoder(keyboard_data, menu_state, config, alarm_data, current_time);

    assertEqual(config.respiratory_rate, 25);
    assertEqual(keyboard_data.selection, menu_state.menu_position);
    assertEqual(menu_state.is_item_selected, false);
}

test(check_encoder_force_parameters_after_time_test) {
    menu_state.menu_number = MAIN_MENU;
    keyboard_data.last_button_pressed = 100;
    unsigned long current_time = 300000;
    check_encoder(keyboard_data, menu_state, config, alarm_data, current_time);

    assertEqual(menu_state.menu_number, PARAMETERS_MENU);
}

test(check_back_state_keyboard_interactions_test) {
    keyboard_data.selection = 1;
    keyboard_data.last_bck_state = 1; // HIGH
    keyboard_data.last_button_pressed = 0;
    unsigned long current_time = 300; // greater than 150ms
    digitalReadValue(PIN_MENU_BCK, LOW);

    check_bck_state(keyboard_data, current_time);

    assertEqual(keyboard_data.pressed, BACK_PRESSED);
    assertEqual((long)keyboard_data.start_pressed, (long)current_time);
    assertEqual(keyboard_data.change_sleep, false);
    assertEqual((long)keyboard_data.last_button_pressed, (long)current_time);
}

test(wait_for_flux_disconnected_loop_test) {
    digitalReadValue(PIN_MENU_EN, LOW); // Enter pressed
    wait_for_flux_disconnected();
    passTestNow();
}

test(check_buttons_down_pressed_test) {
    digitalReadValue(PIN_MENU_DN, LOW);
    keyboard_data.selection = 1;
    keyboard_data.last_button_pressed = 0;
    unsigned long current_time = 300;

    check_buttons(keyboard_data, current_time);

    assertEqual(keyboard_data.selection, 2);
}

void setup() {
    delay(1000); // wait for stability on some boards to prevent garbage Serial
    Serial.begin(115200); // ESP8266 default of 74880 not supported on Linux
    while(!Serial); // for the Arduino Leonardo/Micro only

    Serial.println(F("Menu tests:"));
    Serial.println(F("----"));
}

void loop() {
    aunit::TestRunner::run();
}

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
bool put_to_sleep;
bool show_changed_options;
bool sleep_mode;
unsigned long time2;
bool update_options;
bool wake_up;
EpoxyEepromAvr EEPROM;