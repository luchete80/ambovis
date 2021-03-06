#ifndef _MENU_H_
#define _MENU_H_

#include "defaults.h"

#ifdef LCD_I2C
#include "src/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#else
#include <LiquidCrystal.h>
#endif

#define MAIN_MENU         0
#define ALARM_MENU        1
#define SETTINGS_MENU     2
#define PID_SETT_MENU_1   3
#define PID_SETT_MENU_2   4
#define SCREEN_MENU       5

#ifdef LCD_I2C
extern LiquidCrystal_I2C lcd;
//LiquidCrystal_I2C lcd(0x3F, 20, 4);
#else
//LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
extern LiquidCrystal lcd;
#endif

extern bool switching_menus;
class Menu{
  
  byte cant_opciones_mod;
  byte opciones_mod[3];
  
  public:

  Menu(){};
  ~Menu(){};
  
  
  };
  
  
class Menu_inic:public Menu{
  byte _mod,_bpm,_i_e;  
  unsigned long last_update_display;
  bool fin=false;
  public:
    Menu_inic(byte *mode, byte *bpm, byte *i_e);
    void clear_n_sel( int menu );
    void Menu_inic::check_encoder ( );
    void Menu_inic::display_lcd ( );
    void check_bck_state();
};

extern byte max_sel,min_sel; //According to current selection
extern unsigned long lastButtonPress;

extern int bck_state ;     // current state of the button
extern int last_bck_state ; // previous state of the button
extern int startPressed ;    // the moment the button was pressed
extern int endPressed ;      // the moment the button was released
extern int holdTime ;        // how long the button was hold
extern int idleTime ;        // how long the button was idle
//bool bck_pressed;

extern int curr_sel, old_curr_sel;
extern byte encoderPos; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
extern byte oldEncPos; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
extern byte old_menu_pos,old_menu_num;
extern bool show_changed_options; //Only for display
extern bool update_options;
extern char tempstr[5],tempstr2[5];
extern byte menu_number;
extern byte p_trim;


void writeLine(int line, String message = "", int offsetLeft = 0);
void lcd_clearxy(int x, int y,int pos=1);
void lcd_selxy(int x, int y);
void check_encoder();
void display_lcd ();
void init_display();

void check_updn_button(int pin, byte *var, bool incr_decr);
void check_bck_state();

extern bool isitem_sel;

#endif
