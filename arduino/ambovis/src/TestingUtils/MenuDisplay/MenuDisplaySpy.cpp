//
// Created by Mirian Quinteros on 11/08/2021.
//

#include "MenuDisplaySpy.h"
#include "../../../menu.h"
#include "../../../MechVentilation.h"

MenuDisplaySpy::MenuDisplaySpy() {
}

void MenuDisplaySpy::_init_display() {
    //No comments
    init_display();
}

void MenuDisplaySpy::_check_encoder() {
    check_encoder();
    MenuDisplayToOutput::Params params = collectValues();
    params.functionName = "check_encoder";
    this->menuDisplayToOutput.print(params);
}
void MenuDisplaySpy::_display_lcd () {
    display_lcd();
    MenuDisplayToOutput::Params params = collectValues();
    params.functionName = "display_lcd";
    this->menuDisplayToOutput.print(params);
}

void MenuDisplaySpy::_check_bck_state() {
    //No comments
    check_bck_state();
}

void MenuDisplaySpy::_writeLine(int line, String message = "", int offsetLeft = 0) {
    //No comments
    writeLine(line, message, offsetLeft);
}

void MenuDisplaySpy::_lcd_clearxy(int x, int y,int pos=1) {
    //No comments
    lcd_clearxy(x, y, pos);
}

void MenuDisplaySpy::_lcd_selxy(int x, int y) {
    //No comments
    lcd_selxy(x, y);
}

void MenuDisplaySpy::_check_updn_button(int pin, byte *var, bool incr_decr) {
    check_updn_button(pin, var, incr_decr);
    //No comments
}

MenuDisplayToOutput::Params MenuDisplaySpy::collectValues() {
    MenuDisplayToOutput::Params params;
    params.functionName = "undefined";
    params.last_pressure_min = last_pressure_min;
    params.last_pressure_max = last_pressure_max;
    params.timeoutEsp = _timeoutEsp;
    params.timeoutIns = _timeoutIns;
    params.p_acc = p_acc;
    params.pf_max = pf_max;
    params.pf_min = pf_min;
    params.max_accel = max_accel;
    params.min_accel = min_accel;
    params.max_cd = max_cd;
    params.min_cd = min_cd;
    params.max_speed = max_speed;
    params.min_speed = min_speed;
    params.max_pidd = max_pidd;
    params.min_pidd = min_pidd;
    params.max_pidi = max_pidi;
    params.min_pidi = min_pidi;
    params.min_pidk = min_pidk;
    params.max_pidk = max_pidk;
    return params;
}