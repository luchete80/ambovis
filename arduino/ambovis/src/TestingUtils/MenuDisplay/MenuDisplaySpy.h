//
// Created by Mirian Quinteros on 11/08/2021.
//

#ifndef AMBOVIS_MENUDISPLAYSPY_H
#define AMBOVIS_MENUDISPLAYSPY_H
#include "MenuDisplayToOutput.h"
#ifdef LCD_I2C
#include "src/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#else
#include <LiquidCrystal.h>
#endif

class MenuDisplaySpy {
public:
    MenuDisplaySpy();
    //same functions as menu.h
    void _writeLine(int line, String message = "", int offsetLeft = 0);
    void _lcd_clearxy(int x, int y,int pos=1);
    void _lcd_selxy(int x, int y);
    void _check_encoder();
    void _display_lcd ();
    void _init_display();
    void _check_updn_button(int pin, byte *var, bool incr_decr);
    void _check_bck_state();

private:
    MenuDisplayToOutput menuDisplayToOutput;
    MenuDisplayToOutput::Params collectValues();
};

#endif //AMBOVIS_MENUDISPLAYSPY_H
