#include "menu.h"

static bool clear_all_display;

byte back[8] = {
  0b00100,
  0b01000,
  0b11111,
  0b01001,
  0b00101,
  0b00001,
  0b00001,
  0b11111
};

void init_display() {
    lcd.begin(20, 4);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.createChar(0,back);
}

void writeLine(int line, String message = "", int offsetLeft = 0) {
    lcd.setCursor(0, line);
    lcd.print("");
    lcd.setCursor(offsetLeft, line);
    lcd.print(message);
}

void lcd_clearxy(int x, int y,int pos=1) {
    for (int i=0;i<pos;i++) {
        lcd.setCursor(x+i, y);
        lcd.print(" ");
    }
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
                keyboard_data.pressed = 2;
                keyboard_data.is_item_selected = false;
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
        if (time - keyboard_data.last_button_pressed > 50) {
            keyboard_data.pressed = 1;
            keyboard_data.is_item_selected = true;
            keyboard_data.last_button_pressed = time;
        }
    }

    check_bck_state(keyboard_data, time);
}

void save_value(Menu_state_t& menu_state, byte selection) {
    if (menu_state.menu_number == PARAMETERS_MENU) {
        if (menu_state.is_initial_menu) {
            if (menu_state.menu_position == 0) { // BPM
                options.respiratoryRate = selection;
            }
            if (menu_state.menu_position == 1) { // IE
                options.percInspEsp = selection;
            }
        } else {
            if (menu_state.menu_position == 0) { // PERC_V_OPT
                options.percVolume = selection;
            }
            if (menu_state.menu_position == 1) { // PIP_OPT
                options.peakInspiratoryPressure = selection;
            }
            if (menu_state.menu_position == 2) { // BPM_OPT
                options.respiratoryRate = selection;
            }
            if (menu_state.menu_position == 3) { // IE_OPT
                options.percInspEsp = selection;
            }
        }
    }
    if (menu_state.menu_number == ALARMS_MENU) {
        if (menu_state.menu_position == 0) { // PIP_ALARM_OPT
            alarm_max_pressure = selection;
        }
        if (menu_state.menu_position == 1) { // PEEP_ALARM_OPT
            alarm_peep_pressure = selection;
        }
        if (menu_state.menu_position == 2) { // VT_ALARM_OPT
            alarm_vt = selection;
        }
    }
    if (menu_state.menu_number == SETTINGS_MENU) {
        if (menu_state.menu_position == 0) { // FIL_OPT
            filter = selection == 1;
        }
        if (menu_state.menu_position == 1) { // AUTO_OPT
            autopid = selection == 1;
        }
    }
}

byte validate_boundaries(byte min, byte max, byte value) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

byte validate_parameter(Menu_state_t& menu_state, byte selection) {
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
            if (menu_state.menu_position == 1) { // PIP_OPT
                return validate_boundaries(15, 30, selection);
            }
            if (menu_state.menu_position == 2) { // BPM_OPT
                return validate_boundaries(DEFAULT_MIN_RPM, DEFAULT_MAX_RPM, selection);
            }
            if (menu_state.menu_position == 3) { // IE_OPT
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
    return  -1;
}

byte validate_selection(byte& menu_number, byte selection) {
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
  
void check_encoder(Keyboard_data_t& keyboard_data, Menu_state_t& menu_state, unsigned long time) {
    check_buttons(keyboard_data, time);

    if (menu_state.menu_number == MAIN_MENU) {
        if (keyboard_data.pressed == 1) {
            menu_state.old_menu_position = menu_state.menu_number;
            menu_state.menu_number = keyboard_data.selection;
            menu_state.menu_position = 0;
            menu_state.show_changed_options = true;
            clear_all_display = true;
        } else if (keyboard_data.pressed == 2) {

        } else {
            menu_state.old_menu_position = menu_state.menu_position;
            menu_state.menu_position = validate_selection(menu_state.menu_number, keyboard_data.selection);
            menu_state.show_changed_options = menu_state.old_menu_position != menu_state.menu_position;
        }
    } else {
        if (keyboard_data.is_item_selected) {
            if (keyboard_data.pressed == 1) {
                save_value(menu_state, keyboard_data.selection);
                keyboard_data.is_item_selected = false;
                menu_state.update_options = true;
                menu_state.show_changed_options = true;
            } else if (keyboard_data.pressed == 2) {
                keyboard_data.is_item_selected = false;
                menu_state.show_changed_options = true;
            } else {
                keyboard_data.old_selection = keyboard_data.selection;
                keyboard_data.selection = validate_parameter(menu_state, keyboard_data.selection);
                menu_state.show_changed_options = keyboard_data.old_selection != keyboard_data.selection;
            }
        } else {
            if (keyboard_data.pressed == 1) {
                keyboard_data.selection = 0;
                keyboard_data.is_item_selected = true;
                menu_state.show_changed_options = true;

                if (menu_state.is_initial_menu && menu_state.menu_position == 2) {
                    menu_state.initial_menu_completed = true;
                }

            } else if (keyboard_data.pressed == 2) {
                keyboard_data.is_item_selected = false;
                if (!menu_state.is_initial_menu) {
                    menu_state.menu_number = MAIN_MENU;
                    menu_state.menu_position = 0;
                    menu_state.show_changed_options = true;
                }
            } else {
                menu_state.old_menu_position = menu_state.menu_position;
                menu_state.menu_position = validate_selection(menu_state.menu_number, keyboard_data.selection);
                menu_state.show_changed_options = menu_state.old_menu_position != menu_state.menu_position;
            }
        }
    }
}

void clear_n_sel(byte& menu_number, byte& menu_position, byte& old_menu_position, bool& is_initial_menu) {
    if (menu_number == MAIN_MENU) {
        switch(old_menu_position) {
            case 0: lcd_clearxy(0, 0); break;
            case 1: lcd_clearxy(0, 1); break;
            case 2: lcd_clearxy(0, 2); break;
        }
    } else if (menu_number == PARAMETERS_MENU) {
        if (is_initial_menu) {
            switch(old_menu_position) {
                case 0: lcd_clearxy(0, 1); break; //BPM
                case 1: lcd_clearxy(7, 1); break; //IE
                case 2: lcd_clearxy(0, 2); break; //READY
            }
            switch(menu_position) {
                case 0: lcd_clearxy(6, 1, 3); break; //BPM:
                case 1: lcd_clearxy(15, 1, 2); break; //IE:1:
            }
        } else {
            switch(old_menu_position) {
                case 0: lcd_clearxy(7, 0); break; //BPM
                case 1: lcd_clearxy(0, 1); break; //IE
                case 2: lcd_clearxy(0, 2); break; //READY
            }
            switch(menu_position) {
                case 0: lcd_clearxy(3, 0, 3); break; //V:
                case 1: lcd_clearxy(5, 1, 3); break; //BPM:
                case 2: lcd_clearxy(7, 2, 1); break; //IE:1:
            }
        }
    } else if (menu_number == ALARMS_MENU) {
        switch(old_menu_position) {
            case 0: lcd_clearxy(0, 0); break;//PIP
            case 1: lcd_clearxy(0, 1); break;//PEEP
            case 2: lcd_clearxy(0, 2); break; //VT
        }
        switch(menu_position) {
            case 0: lcd_clearxy(7, 0, 3); break; //PIPAL:
            case 1: lcd_clearxy(7, 1, 3); break; //PEEPAL:
            case 2: lcd_clearxy(6, 2, 3); break; //VTAL:
        }
    } else if (menu_number == SETTINGS_MENU) {
        switch(old_menu_position) {
            case 0: lcd_clearxy(0, 0); break;//FILTER
            case 1: lcd_clearxy(0, 1); break;//AUTOPID
        }
        switch(menu_position) {
            case 0: lcd_clearxy(8, 0, 3); break; //FILTER:
            case 1: lcd_clearxy(9, 1, 3); break; //AUTOPID:
        }
    }
}

void display_lcd(Keyboard_data_t& keyboard_data, Menu_state_t& menu_state) {
    if (clear_all_display) {
        clear_all_display = false;
        lcd.clear();
    }
    clear_n_sel(menu_state.menu_number, menu_state.menu_position, menu_state.old_menu_position, menu_state.is_initial_menu);
    char temp_str[5];
    if (menu_state.menu_number == MAIN_MENU) {
        switch (menu_state.menu_position) {
            case 0: lcd_selxy(0, 0, keyboard_data.is_item_selected); break;
            case 1: lcd_selxy(1, 0, keyboard_data.is_item_selected); break;
            case 2: lcd_selxy(2, 0, keyboard_data.is_item_selected); break;
        }
        writeLine(0, "PARAMETERS", 1);
        writeLine(1, "ALARMS");
        writeLine(2, "SETTINGS", 1);
    } else if (menu_state.menu_number == PARAMETERS_MENU) {
        switch (menu_state.menu_position) {
            case 0: lcd_selxy(0, 9, keyboard_data.is_item_selected); break;
            case 1: lcd_selxy(1, 0, keyboard_data.is_item_selected); break;
            case 2: lcd_selxy(1, 7, keyboard_data.is_item_selected); break;
        }
        writeLine(0, "MOD:MAN", 1);
        if (menu_state.is_initial_menu) {
            writeLine(1, "BPM:" + String(options.respiratoryRate), 1);
            writeLine(1, "IE:1:" + String(options.percInspEsp), 8);
            writeLine(2, "READY", 1);
        } else {
            writeLine(0, "V:" + String(options.percVolume)+"%", 10);
            writeLine(1, "BPM:" + String(options.respiratoryRate), 1);
            writeLine(1, "IE:1:" + String(options.percInspEsp), 8);

            dtostrf(last_pressure_max, 2, 0, temp_str);
            writeLine(3, "PIP:" + String(temp_str), 0);

            dtostrf(last_pressure_min, 2, 0, temp_str);
            writeLine(3, "PEEP: " + String(temp_str), 6);

            dtostrf((_mllastInsVol + _mllastExsVol)/2.*options.respiratoryRate*0.001, 2, 1, temp_str);
            writeLine(3, "VM:" + String(temp_str), 16);
        }
    } else if (menu_state.menu_number == ALARMS_MENU) {
        switch (menu_state.menu_position) {
            case 0: lcd_selxy(0, 0, keyboard_data.is_item_selected); break;
            case 1: lcd_selxy(1, 0, keyboard_data.is_item_selected); break;
            case 2: lcd_selxy(2, 0, keyboard_data.is_item_selected); break;
        }
        
        writeLine(0, "PIPAL:" + String(alarm_max_pressure), 1);
        dtostrf(Cdyn*1.01972, 2, 1, temp_str);
        writeLine(0, "CD:" + String(temp_str), 10);
    
        writeLine(1, "PEEPAL:" + String(alarm_peep_pressure), 1);
        writeLine(2, "VTAL:" + String(alarm_vt), 1);
    
        writeLine(3, "CYCLE:" + String(last_cycle), 1);

    } else if (menu_state.menu_number == SETTINGS_MENU) {
        switch (menu_state.menu_position) {
            case 0: lcd_selxy(0, 0, keyboard_data.is_item_selected); break;
            case 1: lcd_selxy(1, 0, keyboard_data.is_item_selected); break;
        }
        writeLine(0, "FILTER:" , 1);
        if (filter) writeLine(0, "ON", 9); else writeLine(0, "OFF", 9);

        writeLine(1, "AUTOPID: ", 1);
        if (autopid) writeLine(1, "ON", 9); else writeLine(1, "OFF", 9);
    }
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
        time=millis();
        check_encoder(keyboard_data, menu_state, time);
        if (menu_state.show_changed_options && (millis() - last_update_display) > 50) {
            display_lcd(keyboard_data, menu_state);
            last_update_display = millis();
        }
    }
    keyboard_data.is_item_selected = false;
    menu_state.is_initial_menu = false;
}