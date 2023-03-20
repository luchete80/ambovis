#include "menu.h"

byte backDigit[8] = {
        0b00100, 0b01000, 0b11111,
        0b01001, 0b00101, 0b00001,
        0b00001, 0b11111
};

void init_display() {
    lcd.begin(20, 4);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.createChar(0, backDigit);
}

void writeLine(int line, String message = "", int offsetLeft = 0) {
    lcd.setCursor(0, line);
    lcd.print("");
    lcd.setCursor(offsetLeft, line);
    lcd.print(message);
}

void lcd_clearxy(int x, int y,int pos=1) {
    for (int i=0; i<pos; i++) {
        lcd.setCursor(x+i, y);
        lcd.print(" ");
    }
}

void lcd_update_value(int x, int y, int pos, String msg) {
    lcd_clearxy(x, y, pos);
    lcd.setCursor(x, y);
    lcd.print(msg);
}

void lcd_selxy(int x, int y, bool isItemSelected) {
    lcd.setCursor(x, y);
    if (!isItemSelected) {
        lcd.print(">");
    } else {
        lcd.write(byte(0));
    }
}

void check_bck_state(Keyboard_data_t& keyboard_data, unsigned long time) {
    keyboard_data.bck_state=digitalRead(PIN_MENU_BCK);

    if (keyboard_data.bck_state != keyboard_data.last_bck_state) {
        if (keyboard_data.bck_state == LOW) {
            keyboard_data.start_pressed = time;
            keyboard_data.change_sleep = false;
            if (time - keyboard_data.last_button_pressed > 150) {
                keyboard_data.pressed = BACK_PRESSED;
                keyboard_data.last_button_pressed = time;
            }
        } else {
            keyboard_data.end_pressed = time;
            keyboard_data.hold_time = keyboard_data.end_pressed - keyboard_data.start_pressed;
        }
    } else {
        if (keyboard_data.bck_state == LOW) {
            keyboard_data.hold_time = time - keyboard_data.start_pressed;
        } // button state not changed. It runs in a loop.
        if (keyboard_data.hold_time > 2000 && !keyboard_data.change_sleep) {
            if (!sleep_mode) {
                sleep_mode=true;
                put_to_sleep=true;
            } else {
                sleep_mode=false;
                wake_up=true;
            }
            keyboard_data.change_sleep=true;
            Serial.print("Sleep Mode");Serial.println(sleep_mode);
        }
    }
    keyboard_data.last_bck_state = keyboard_data.bck_state;
}

void check_buttons(Keyboard_data_t& keyboard_data, unsigned long time) {
    if (digitalRead(PIN_MENU_DN) == LOW) {
        if (time - keyboard_data.last_button_pressed > 150) {
            keyboard_data.selection+=1;
            keyboard_data.last_button_pressed = time;
        }
    }

    if (digitalRead(PIN_MENU_UP) == LOW) {
        if (time - keyboard_data.last_button_pressed > 150) {
            keyboard_data.selection-=1;
            keyboard_data.last_button_pressed = time;
        }
    }

    keyboard_data.pressed = 0;
    if (digitalRead(PIN_MENU_EN) == LOW) {
        if (millis() - keyboard_data.last_button_pressed > 120) {
            keyboard_data.pressed = ENTER_PRESSED;
            keyboard_data.last_button_pressed = millis();
        }
    }

    if (keyboard_data.pressed == 0) {
        check_bck_state(keyboard_data, time);
    }
}

int8_t* get_value_to_edit(Menu_state_t& menu_state) {
    if (menu_state.menu_number == PARAMETERS_MENU) {
        if (menu_state.is_initial_menu) {
            if (menu_state.menu_position == 0) { // BPM
                return &options.respiratoryRate;
            }
            if (menu_state.menu_position == 1) { // IE
                return &options.percInspEsp;
            }
        } else {
            if (menu_state.menu_position == 0) { // PERC_V_OPT
                return &options.percVolume;
            }
            if (menu_state.menu_position == 1) { // BPM_OPT
                return &options.respiratoryRate;
            }
            if (menu_state.menu_position == 2) { // IE_OPT
                return &options.percInspEsp;
            }
        }
    }
    if (menu_state.menu_number == ALARMS_MENU) {
        if (menu_state.menu_position == 0) { // PIP_ALARM_OPT
            return &alarm_max_pressure;
        }
        if (menu_state.menu_position == 1) { // PEEP_ALARM_OPT
            return &alarm_peep_pressure;
        }
        if (menu_state.menu_position == 2) { // VT_ALARM_OPT
            return &alarm_vt;
        }
    }
    if (menu_state.menu_number == SETTINGS_MENU) {
        if (menu_state.menu_position == 0) { // FIL_OPT
            return &filter;
        }
        if (menu_state.menu_position == 1) { // AUTO_OPT
            return &autopid;
        }
    }
}

int8_t validate_boundaries(int8_t min, int8_t max, int8_t value) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

int8_t validate_parameter(Menu_state_t& menu_state, int8_t selection) {
    if (menu_state.menu_number == PARAMETERS_MENU) {
        if (menu_state.is_initial_menu) {
            if (menu_state.menu_position == 0) { // BPM
                return validate_boundaries(DEFAULT_MIN_RPM, DEFAULT_MAX_RPM, selection);
            }
            if (menu_state.menu_position == 1) { // IE
                return validate_boundaries(1, 3, selection);
            }
            if (menu_state.menu_position == 2) { // READY
                return selection;
            }
        } else {
            if (menu_state.menu_position == 0) { // PERC_V_OPT
                return validate_boundaries(40, 100, selection);
            }
            if (menu_state.menu_position == 1) { // BPM_OPT
                return validate_boundaries(DEFAULT_MIN_RPM, DEFAULT_MAX_RPM, selection);
            }
            if (menu_state.menu_position == 2) { // IE_OPT
                return validate_boundaries(1, 3, selection);
            }
        }
    }
    if (menu_state.menu_number == ALARMS_MENU) {
        if (menu_state.menu_position == 0) { // PIP_ALARM_OPT
            return validate_boundaries(20, 50, selection);
        }
        if (menu_state.menu_position == 1) { // PEEP_ALARM_OPT
            return validate_boundaries(5, 30, selection);
        }
        if (menu_state.menu_position == 2) { // VT_ALARM_OPT
            return validate_boundaries(10, 50, selection);
        }
    }
    if (menu_state.menu_number == SETTINGS_MENU) {
        if (menu_state.menu_position == 0) { // FIL_OPT
            return validate_boundaries(0, 1, selection);
        }
        if (menu_state.menu_position == 1) { // AUTO_OPT
            return validate_boundaries(0, 1, selection);
        }
    }
    return -1;
}

int8_t validate_menu_position(int8_t& menu_number, int8_t selection) {
    if (menu_number == MAIN_MENU) {
        return validate_boundaries(0, 2, selection);
    }
    if (menu_number == PARAMETERS_MENU) {
        return validate_boundaries(0, 2, selection);
    }
    if (menu_number == ALARMS_MENU) {
        return validate_boundaries(0, 2, selection);
    }
    if (menu_number == SETTINGS_MENU) {
        return validate_boundaries(0, 1, selection);
    }
    return -1;
}

bool forceParametersMenuAfterWait(Keyboard_data_t& keyboard_data, Menu_state_t& menu_state, unsigned long time) {
    if (menu_state.menu_number != PARAMETERS_MENU) {
        if (time - keyboard_data.last_button_pressed > 10000) {
            menu_state.menu_number = PARAMETERS_MENU;
            keyboard_data.edited_value = keyboard_data.selection = 0;
            keyboard_data.is_item_selected = false;
            menu_state.old_menu_position = menu_state.menu_position = 0;
            menu_state.show_changed_options = true;
            menu_state.clear_display = true;
            return true;
        }
    }
    return false;
}

bool isEditing(int8_t m, int8_t p, bool is_item_selected, Menu_state_t& menu_state) {
    return is_item_selected && menu_state.menu_number == m && menu_state.menu_position == p;
}

String value_to_display(int8_t menu, int8_t position, Keyboard_data_t& keyboard, Menu_state_t& menu_state) {
    bool itemSel = keyboard.is_item_selected;
    String selection = String(keyboard.edited_value);
    if (menu == PARAMETERS_MENU) {
        if (menu_state.is_initial_menu) {
            switch (position) {
                case 0: return isEditing(PARAMETERS_MENU, 0, itemSel, menu_state) ? selection : String(options.respiratoryRate);
                case 1: return isEditing(PARAMETERS_MENU, 1, itemSel, menu_state) ? selection : String(options.percInspEsp);
            }
        } else {
            switch (position) {
                case 0: return isEditing(PARAMETERS_MENU, 0, itemSel, menu_state) ? selection : String(options.percVolume);
                case 1: return isEditing(PARAMETERS_MENU, 1, itemSel, menu_state) ? selection : String(options.respiratoryRate);
                case 2: return isEditing(PARAMETERS_MENU, 2, itemSel, menu_state) ? selection : String(options.percInspEsp);
            }
        }
    } else if (menu == ALARMS_MENU) {
        switch (position) {
            case 0: return isEditing(ALARMS_MENU, 0, itemSel, menu_state) ? selection : String(alarm_max_pressure);
            case 1: return isEditing(ALARMS_MENU, 1, itemSel, menu_state) ? selection : String(alarm_peep_pressure);
            case 2: return isEditing(ALARMS_MENU, 2, itemSel, menu_state) ? selection : String(alarm_vt);
        }
    } else if (menu == SETTINGS_MENU) {
        switch (position) {
            case 0: return isEditing(SETTINGS_MENU, 0, itemSel, menu_state) ? selection : String(filter);
            case 1: return isEditing(SETTINGS_MENU, 1, itemSel, menu_state) ? selection : String(autopid);
        }
    }
    return String("ERR");
}

void update_edited_value(int8_t& menu_number, int8_t& menu_position, int8_t& old_menu_position, bool& is_initial_menu, int8_t& edited_value) {
    if (menu_number == PARAMETERS_MENU) {
        if (is_initial_menu) {
            switch(menu_position) {
                case 0:lcd_update_value(5, 2, 2, String(edited_value)); break;
                case 1:lcd_update_value(13, 2, 2, String(edited_value)); break;
            }
        } else {
            switch(menu_position) {
                case 0:lcd_update_value(12, 0, 4, String(edited_value) + "%"); break;
                case 1:lcd_update_value(5, 1, 3, String(edited_value)); break;
                case 2:lcd_update_value(13, 1, 1, String(edited_value)); break;
            }
        }
    } else if (menu_number == ALARMS_MENU) {
        switch(menu_position) {
            case 0:lcd_update_value(7, 0, 3, String(edited_value)); break;
            case 1:lcd_update_value(8, 1, 3, String(edited_value)); break;
            case 2:lcd_update_value(6, 2, 3, String(edited_value)); break;
        }
    } else if (menu_number == SETTINGS_MENU) {
        switch(menu_position) {
            case 0:lcd_update_value(8, 0, 3, String(edited_value) == "1" ? "ON" : "OFF"); break;
            case 1:lcd_update_value(9, 1, 3, String(edited_value) == "1" ? "ON" : "OFF"); break;
        }
    }
}

void show_cursor(int8_t& menu_number, int8_t& menu_position, bool& is_item_selected, bool& is_initial_menu) {
    if (menu_number == MAIN_MENU) {
        switch(menu_position) {
            case 0: lcd_selxy(0, 0, false); break; //PARAMETERS
            case 1: lcd_selxy(0, 1, false); break; //ALARMS
            case 2: lcd_selxy(0, 2, false); break; //SETTINGS
        }
    } else if (menu_number == PARAMETERS_MENU) {
        if (is_initial_menu) {
            switch(menu_position) {
                case 0: lcd_selxy(0, 2, is_item_selected); break;
                case 1: lcd_selxy(7, 2, is_item_selected); break; //IE:1:
                case 2: lcd_selxy(0, 3, is_item_selected); break; //READY
            }
        } else {
            switch(menu_position) {
                case 0: lcd_selxy(9, 0, is_item_selected); break; //V:
                case 1: lcd_selxy(0, 1, is_item_selected); break; //BPM:
                case 2: lcd_selxy(7, 1, is_item_selected); break; //IE:1:
            }
        }
    } else if (menu_number == ALARMS_MENU) {
        switch(menu_position) {
            case 0: lcd_selxy(0, 0, is_item_selected); break; //PIPAL:
            case 1: lcd_selxy(0, 1, is_item_selected); break; //PEEPAL:
            case 2: lcd_selxy(0, 2, is_item_selected); break; //VTAL:
        }
    } else if (menu_number == SETTINGS_MENU) {
        switch(menu_position) {
            case 0: lcd_selxy(0, 0, is_item_selected); break; //FILTER:
            case 1: lcd_selxy(0, 1, is_item_selected); break; //AUTOPID:
        }
    }
}

void move_cursor(int8_t& menu_number, int8_t& menu_position, int8_t& old_menu_position, bool& is_item_selected, bool& is_initial_menu) {
    if (menu_number == MAIN_MENU) {
        switch(old_menu_position) {
            case 0: lcd_clearxy(0, 0); break;
            case 1: lcd_clearxy(0, 1); break;
            case 2: lcd_clearxy(0, 2); break;
        }
    } else if (menu_number == PARAMETERS_MENU) {
        if (is_initial_menu) {
            switch(old_menu_position) {
                case 0: lcd_clearxy(0, 2); break; //BPM
                case 1: lcd_clearxy(7, 2); break; //IE
                case 2: lcd_clearxy(0, 3); break; //READY
            }
        } else {
            switch(old_menu_position) {
                case 0: lcd_clearxy(9, 0); break; //V:
                case 1: lcd_clearxy(0, 1); break; //BPM:
                case 2: lcd_clearxy(7, 1); break; //IE:
            }
        }
    } else if (menu_number == ALARMS_MENU) {
        switch(old_menu_position) {
            case 0: lcd_clearxy(0, 0); break;//PIP
            case 1: lcd_clearxy(0, 1); break;//PEEP
            case 2: lcd_clearxy(0, 2); break; //VT
        }
    } else if (menu_number == SETTINGS_MENU) {
        switch(old_menu_position) {
            case 0: lcd_clearxy(0, 0); break;//FILTER
            case 1: lcd_clearxy(0, 1); break;//AUTOPID
        }
    }
    show_cursor(menu_number, menu_position, is_item_selected, is_initial_menu);
}

void check_encoder(Keyboard_data_t& keyboard_data, Menu_state_t& menu_state, unsigned long time) {
    keyboard_data.old_selection = keyboard_data.edited_value;
    check_buttons(keyboard_data, time);

    if (forceParametersMenuAfterWait(keyboard_data, menu_state, time)) {
        return;
    }

    if (time - keyboard_data.last_button_pressed > 2000) {
        return;
    }

    if (keyboard_data.is_item_selected) {
        if (keyboard_data.pressed == ENTER_PRESSED) {
            *keyboard_data.value_to_edit = validate_parameter(menu_state, keyboard_data.selection);
            keyboard_data.edited_value = 0;
            menu_state.update_options = true;
            keyboard_data.selection = menu_state.menu_position;
            keyboard_data.is_item_selected = false;
            menu_state.show_changed_options = true;
            show_cursor(menu_state.menu_number, menu_state.menu_position,
                        keyboard_data.is_item_selected, menu_state.is_initial_menu);
        } else if (keyboard_data.pressed == BACK_PRESSED) {
            keyboard_data.is_item_selected = false;
            menu_state.show_changed_options = true;
            keyboard_data.selection = menu_state.menu_position;
            show_cursor(menu_state.menu_number, menu_state.menu_position,
                        keyboard_data.is_item_selected, menu_state.is_initial_menu);
        } else {
            keyboard_data.selection = validate_parameter(menu_state, keyboard_data.selection);
            keyboard_data.edited_value = keyboard_data.selection;
            menu_state.show_changed_options = keyboard_data.old_selection != keyboard_data.edited_value;
            if (menu_state.show_changed_options) {
                update_edited_value(menu_state.menu_number, menu_state.menu_position, menu_state.old_menu_position,
                                    menu_state.is_initial_menu, keyboard_data.edited_value);
            }
        }
    } else {
        if (keyboard_data.pressed == ENTER_PRESSED) {
            if (menu_state.menu_number == MAIN_MENU) {
                menu_state.menu_number = MENU_NUMBER_LIST[keyboard_data.edited_value];
                keyboard_data.selection = keyboard_data.edited_value = 0;
                menu_state.show_changed_options = true;
                menu_state.menu_position = 0;
                menu_state.clear_display = true;
            } else {
                if (menu_state.is_initial_menu && menu_state.menu_position == 2) {
                    menu_state.initial_menu_completed = true;
                } else {
                    keyboard_data.value_to_edit = get_value_to_edit(menu_state);
                    keyboard_data.selection = keyboard_data.edited_value = *keyboard_data.value_to_edit;
                    keyboard_data.is_item_selected = true;
                    show_cursor(menu_state.menu_number, menu_state.menu_position,
                                keyboard_data.is_item_selected, menu_state.is_initial_menu);
                }
            }
        } else if (keyboard_data.pressed == BACK_PRESSED) {
            if (menu_state.menu_number != MAIN_MENU) {
                menu_state.clear_display = true;
                menu_state.show_changed_options = true;
                if (!menu_state.is_initial_menu) {
                    menu_state.menu_number = MAIN_MENU;
                    keyboard_data.selection = 0;
                    menu_state.old_menu_position = menu_state.menu_position = 0;
                }
            }
        } else {
            keyboard_data.selection = validate_menu_position(menu_state.menu_number, keyboard_data.selection);
            keyboard_data.edited_value = keyboard_data.selection;
            menu_state.old_menu_position = menu_state.menu_position;
            menu_state.menu_position = keyboard_data.edited_value;
            menu_state.show_changed_options = menu_state.old_menu_position != menu_state.menu_position;
            if (menu_state.show_changed_options) {
                move_cursor(menu_state.menu_number, menu_state.menu_position, menu_state.old_menu_position,
                            keyboard_data.is_item_selected, menu_state.is_initial_menu);
            }
        }
    }
}

void display_lcd(Keyboard_data_t& keyboard_data, Menu_state_t& menu_state) {
    if (menu_state.clear_display) {
        menu_state.clear_display = false;
        lcd.clear();
    }
    char temp_str[5];
    if (menu_state.menu_number == MAIN_MENU) {
        writeLine(0, "PARAMETROS", 1);
        writeLine(1, "ALARMAS", 1);
        writeLine(2, "SETTINGS", 1);
    } else if (menu_state.menu_number == PARAMETERS_MENU) {
        if (menu_state.is_initial_menu) {
            writeLine(0, "Ingrese params", 1);
            writeLine(1, "MOD:MAN", 1);
            writeLine(2, "BPM:" + value_to_display(PARAMETERS_MENU, 0, keyboard_data, menu_state), 1);
            writeLine(2, "IE:1:" + value_to_display(PARAMETERS_MENU, 1, keyboard_data, menu_state), 8);
            writeLine(3, "READY", 1);
        } else {
            writeLine(0, "MOD:MAN", 1);
            writeLine(0, "V:" + value_to_display(PARAMETERS_MENU, 0, keyboard_data, menu_state)+"%", 10);
            writeLine(1, "BPM:" + value_to_display(PARAMETERS_MENU, 1, keyboard_data, menu_state), 1);
            writeLine(1, "IE:1:" + value_to_display(PARAMETERS_MENU, 2, keyboard_data, menu_state), 8);

            dtostrf(last_pressure_max, 2, 0, temp_str);
            writeLine(2, "PIP:" + String(temp_str), 1);

            dtostrf(last_pressure_min, 2, 0, temp_str);
            writeLine(2, "PEEP:" + String(temp_str), 9);

            dtostrf((_mllastInsVol + _mllastExsVol)/2.*options.respiratoryRate*0.001, 2, 1, temp_str);
            writeLine(3, "VM:" + String(temp_str), 1);
        }
    } else if (menu_state.menu_number == ALARMS_MENU) {
        writeLine(0, "PIPAL:" + value_to_display(ALARMS_MENU, 0, keyboard_data, menu_state), 1);
        dtostrf(Cdyn*1.01972, 2, 1, temp_str);
        writeLine(0, "CD:" + String(temp_str), 10);
    
        writeLine(1, "PEEPAL:" + value_to_display(ALARMS_MENU, 1, keyboard_data, menu_state), 1);
        writeLine(2, "VTAL:" + value_to_display(ALARMS_MENU, 2, keyboard_data, menu_state), 1);
    
        writeLine(3, "CYCLE:" + String(last_cycle), 1);

    } else if (menu_state.menu_number == SETTINGS_MENU) {
        writeLine(0, "FILTER:" , 1);
        String filterStr = value_to_display(SETTINGS_MENU, 0, keyboard_data, menu_state);
        if (filterStr == "1") writeLine(0, "ON", 8); else writeLine(0, "OFF", 8);

        writeLine(1, "AUTOPID: ", 1);
        String autopidStr = value_to_display(SETTINGS_MENU, 1, keyboard_data, menu_state);
        if (autopidStr == "1") writeLine(1, "ON", 9); else writeLine(1, "OFF", 9);
    }
    show_cursor(menu_state.menu_number, menu_state.menu_position,
                keyboard_data.is_item_selected, menu_state.is_initial_menu);
}

void initialize_menu(Keyboard_data_t& keyboard_data, Menu_state_t& menu_state) {
    menu_state.menu_number = PARAMETERS_MENU;
    menu_state.menu_position = 0;
    menu_state.old_menu_position = 0;
    menu_state.is_initial_menu = true;
    menu_state.initial_menu_completed = false;

    lcd.clear();
    keyboard_data.last_button_pressed = 0;
    display_lcd(keyboard_data, menu_state);

    unsigned long last_update_display = millis();
    while (!menu_state.initial_menu_completed) {
        check_encoder(keyboard_data, menu_state, millis());
        if (menu_state.show_changed_options && (millis() - last_update_display) > 60) {
            display_lcd(keyboard_data, menu_state);
            last_update_display = millis();
        }
    }
    keyboard_data.selection = 0;
    menu_state.is_initial_menu = false;
}