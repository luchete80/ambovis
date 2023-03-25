#ifndef _MENU_H_
#define _MENU_H_

#include <arduino.h>
#include <LiquidCrystal.h>
#include "MechanicalVentilation.h"

extern LiquidCrystal lcd;
extern byte max_sel,min_sel; //According to current selection
extern unsigned long lastButtonPress;

extern int bck_state ;     // current state of the button
extern int last_bck_state ; // previous state of the button
extern int startPressed ;    // the moment the button was pressed
extern int endPressed ;      // the moment the button was released
extern int holdTime ;        // how long the button was hold
extern int idleTime ;        // how long the button was idle

extern int curr_sel, old_curr_sel;
extern byte encoderPos; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
extern byte oldEncPos; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
extern bool show_changed_options; //Only for display
extern bool update_options;
extern char tempstr[5];
extern byte menu_number;
extern byte p_trim;
// These were imported from old mechVentilation
extern int max_accel,min_accel,max_speed,min_speed,max_cd,min_cd,max_pidk,min_pidk;
extern int max_pidi,min_pidi;
extern int max_pidd,min_pidd;
extern float f_acc;
extern byte f_acc_b;
extern byte p_acc;
extern byte pfmin,pfmax;
extern float peep_fac;
extern float pf_min,pf_max;
extern float dpip;
extern byte dpip_b;
extern bool autopid;
extern bool filter;
extern float flow_f;

extern byte alarm_max_pressure;
extern byte alarm_peep_pressure;
extern int alarm_vt;

void writeLine(int line, String message = "", int offsetLeft = 0);
void lcd_clearxy(int x, int y,int pos=1);
void lcd_selxy(int x, int y);
void check_encoder(VentilationStatus status, VentilationConfig& config);
void display_lcd (VentilationStatus& status, VentilationConfig& config);
void init_display();

void check_updn_button(int pin, byte *var, bool incr_decr);
void check_bck_state();

extern bool isitem_sel;

class Menu_inic {
  byte _mod,_bpm,_i_e;  
  unsigned long last_update_display;
  bool fin=false;
  bool switching_menus;
  int m_curr_sel;
  public:
    Menu_inic(VentilationConfig& vent_config);
    void clear_n_sel( int menu );
    void Menu_inic::check_encoder (VentilationConfig& vent_config);
    void Menu_inic::display_lcd (VentilationConfig& config);
    void check_bck_state();
};

#endif
