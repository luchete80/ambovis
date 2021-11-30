#include <Arduino.h>
#include "pinout.h"
#include "menu.h"
#include "MechVentilation.h"  //options

//This is the ASCI representation of the back symbol for the display
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

void updateState(MenuState& menuState);
void updateCounter(MenuState& menuState);
void lcd_clearxy(int x, int y, int pos = 1);
void lcd_selxy(int x, int y, bool item_sel);
void check_updn_button(int pin, byte *var, bool incr_decr, unsigned long& lastButtonPress);

void init_display() {
    #if TESTING_MODE_DISABLED
    #ifdef LCD_I2C
    lcd.begin(20, 4);  //I2C
    #else
    lcd.begin(20, 4); //NO I2C
    #endif
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.createChar(0,back);
    #endif //TESTING_MODE_DISABLED
}

void writeLine(int line, String message, int offsetLeft) {
    #if TESTING_MODE_DISABLED
    lcd.setCursor(0, line);
    lcd.print("");
    lcd.setCursor(offsetLeft, line);
    lcd.print(message);
    #endif//TESTING_MODE_DISABLED
}

void check_bck_state(MenuState& menuState, SystemState& systemState) {
    menuState.bck_state = digitalRead(PIN_MENU_BCK);
    if (menuState.bck_state != menuState.last_bck_state) {
        updateState(menuState); // button state changed. It runs only once.
        if ( menuState.bck_state == LOW ) { //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6)
            if (time - menuState.lastButtonPress > 150) {
              menuState.pressed = 2;
              menuState.isitem_sel=false;
              menuState.lastButtonPress = time;
            }// if time > last button press
        }
    } else {
       updateCounter(menuState); // button state not changed. It runs in a loop.
       if ( menuState.holdTime > 2000 && !menuState.change_sleep ) {
           Serial.println("Activando Sleep Mode");
           if ( !systemState.sleep_mode ){
               systemState.sleep_mode = true;
               systemState.put_to_sleep = true;
           } else {
               systemState.sleep_mode = false;
               systemState.wake_up = true;
           }
            menuState.change_sleep=true;
            Serial.print("Sleep Mode");Serial.println(systemState.sleep_mode);
        }
    }
    menuState.last_bck_state = menuState.bck_state;
}
  
void check_encoder(MenuState& menuState, SystemState& systemState, SensorParams& sensorParams) {
    byte encoderPos;
    check_updn_button(PIN_MENU_DN, &encoderPos, true, menuState.lastButtonPress);   //Increment
    check_updn_button(PIN_MENU_UP, &encoderPos, false, menuState.lastButtonPress);  //Decrement
    menuState.pressed=0;  //0 nothing , 1 enter, 2 bck
    menuState.encoderPos = encoderPos;

    if (digitalRead(PIN_MENU_EN) == LOW) { //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6) v
        if (time - menuState.lastButtonPress > 50) {
            menuState.pressed = 1;
            menuState.isitem_sel=true;
            menuState.lastButtonPress = time;
        }// if time > last button press
    }

    check_bck_state(menuState, systemState);

    byte max_sel, min_sel;
    if (menuState.pressed > 0) { //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6)
        if (!menuState.isitem_sel) {
            menuState.curr_sel = menuState.oldEncPos = menuState.encoderPos = menuState.old_curr_sel;
        }
                  
        if (menuState.isitem_sel) {
            switch (menuState.curr_sel) {
            case 1:
                if ( menuState.menu_number == 0 ) {
                    min_sel=1;max_sel=2;
                    menuState.encoderPos = menuState.oldEncPos = systemState.vent_mode;
                } else if ( menuState.menu_number == 1 ) {
                    min_sel=20;max_sel=50;
                    menuState.encoderPos = menuState.oldEncPos = alarm_max_pressure;
                } else if ( menuState.menu_number == 2 ) {
                    menuState.encoderPos = min_cd;
                    min_sel=0;max_sel=max_cd;
                } else if ( menuState.menu_number == 3 ) {
                    menuState.encoderPos = dpip_b;
                    min_sel=10;max_sel=40;
                }
                break;
            case 2: 
                if ( menuState.menu_number == 0 ) {
                    menuState.encoderPos = menuState.oldEncPos = options.respiratoryRate;
                    min_sel=DEFAULT_MIN_RPM;max_sel=DEFAULT_MAX_RPM;
                } else if ( menuState.menu_number == 1 ) {
                    min_sel=5;max_sel=30;
                    menuState.encoderPos = menuState.oldEncPos = alarm_peep_pressure;
                } else if ( menuState.menu_number == 2 ){
                    menuState.encoderPos=min_speed/10;
                    min_sel=10;max_sel=100;
                } else if ( menuState.menu_number == 3 ){
                    menuState.encoderPos=pfmin=50.*pf_min;
                    min_sel=0;max_sel=99;
                }
                break;
            case 3:
                if ( menuState.menu_number == 0 ) {
                    menuState.encoderPos = menuState.oldEncPos = options.percInspEsp;
                    min_sel=1;max_sel=3;
                } else if ( menuState.menu_number == 1 ) {
                    menuState.encoderPos = byte(0.1*float(alarm_vt));
                    min_sel=10;max_sel=50;//vt
                } else if ( menuState.menu_number == 2 ) {
                    menuState.encoderPos = min_accel/10;
                    min_sel=10;max_sel=100; 
                } else if ( menuState.menu_number == 3 ){
                    menuState.encoderPos = pfmax=50.*pf_max;
                    min_sel=0;max_sel=99;
                }
                break;
            case 4: 
                if ( menuState.menu_number == 0 ) {
                    if ( systemState.vent_mode==VENTMODE_PCL ) {
                        menuState.encoderPos = menuState.oldEncPos = options.peakInspiratoryPressure;
                        min_sel=15;max_sel=30;
                        Serial.print("pip: ");Serial.println(options.peakInspiratoryPressure);
                        Serial.print("encoderpos: ");Serial.println(encoderPos);
                    } else {//Manual
                        menuState.encoderPos = menuState.oldEncPos = options.percVolume;
                        min_sel=40;max_sel=100;
                    } 
                } else if ( menuState.menu_number == 1 ) {//menu 0
                    menuState.encoderPos = menuState.oldEncPos = menuState.p_trim;
                    min_sel=0;max_sel=200;   
                } else if ( menuState.menu_number == 2 ) {
                    menuState.encoderPos = max_cd;
                    min_sel=10;max_sel=80;
                } else if ( menuState.menu_number == 3 ) {
                    menuState.encoderPos = p_acc;
                    min_sel=10;max_sel=40;
                }
                break;
            case 5:
                if ( menuState.menu_number == 1 ) {
                    min_sel=0;max_sel=1;
                } else if ( menuState.menu_number == 2 ){
                    menuState.encoderPos = max_speed/10;
                    min_sel=10;max_sel=100;
                } else if ( menuState.menu_number == 3 ) {
                    menuState.encoderPos = f_acc_b;
                    min_sel=10;max_sel=40;
                }
                break;
            case 6:
                if ( menuState.menu_number == 1 ) {
                    menuState.encoderPos = filter ? 1 : 0;
                    min_sel=0;max_sel=1;
                } else if ( menuState.menu_number == 2 ){
                    menuState.encoderPos = max_accel/10;
                    min_sel=10;max_sel=100;
                }
                break;
            case 7:
                if ( menuState.menu_number == 2 ) {
                    menuState.encoderPos = min_pidk/10;
                    min_sel=10;max_sel=99;
                }
                break;
            case 8:
                if ( menuState.menu_number == 2 ){
                    menuState.encoderPos = byte(min_pidi/2);
                    min_sel=2;max_sel=200;
                }
                break;
            case 9:
                if ( menuState.menu_number == 2 ){
                    menuState.encoderPos = byte(min_pidd/2);
                    min_sel=2;max_sel=200;
                }
                break;
            case 10:
                if ( menuState.menu_number == 2 ){
                    menuState.encoderPos = max_pidk/10;
                    min_sel=2;max_sel=200;
                }
                break;
            case 11:
                if ( menuState.menu_number == 2 ){
                    menuState.encoderPos = byte(max_pidi/2);
                    min_sel=2;max_sel=200;
                }
                break;
            case 12:
                if ( menuState.menu_number == 2 ){
                    menuState.encoderPos = byte(max_pidd/2);
                    min_sel=2;max_sel=200;
                }
                break;
            }
    
        }//if switch select
        menuState.show_changed_options = true;
        menuState.update_options = true;
    }//If selection
  
    if (menuState.oldEncPos != menuState.encoderPos) {
        menuState.show_changed_options = true;

        if (!menuState.isitem_sel) { //Selecting position
            menuState.curr_sel = menuState.encoderPos;
            menuState.encoderPos = menuState.oldEncPos = menuState.curr_sel;

            if ( menuState.menu_number == 0 ) {
                if ( menuState.encoderPos > 4 ) {
                    menuState.encoderPos=1;
                    menuState.menu_number+=1;
                } else if ( menuState.encoderPos < 1 ) {
                    menuState.encoderPos=5;
                    menuState.menu_number=3;
                }
            } else if ( menuState.menu_number == 1 ) {
                if ( menuState.encoderPos > 6 ) {
                    menuState.encoderPos = 1;
                    menuState.menu_number = 2;
                } else if ( menuState.encoderPos < 1 ) {
                    menuState.encoderPos=4;
                    menuState.menu_number=0;
                }
            } else if ( menuState.menu_number == 2 ) {
                if ( menuState.curr_sel > 12 ) {
                    menuState.encoderPos=1;
                    menuState.menu_number=3;
                } else if ( menuState.encoderPos < 1 ) {
                    menuState.encoderPos=6;
                    menuState.menu_number=1;
                }
            } else if ( menuState.menu_number == 3 ) {
                if ( menuState.curr_sel > 5 ) {
                    menuState.encoderPos=1;
                    menuState.menu_number=0;
                } else if ( menuState.encoderPos < 1) {
                    menuState.encoderPos=12;
                    menuState.menu_number=2;
                }
            }
            menuState.clear_all_display=true;
            display_lcd(menuState, systemState.vent_mode, sensorParams);
        } else {//inside a particular selection curr_sel != 0
            if ( menuState.encoderPos > max_sel ) {
                menuState.encoderPos = menuState.oldEncPos = max_sel;
            } else if ( menuState.encoderPos < min_sel ) {
                menuState.encoderPos = menuState.oldEncPos = min_sel;
            } else {
                menuState.oldEncPos = menuState.encoderPos;
                switch (menuState.curr_sel) {
                case 1:
                    if ( menuState.menu_number == 0 ) {
                        systemState.vent_mode = menuState.encoderPos;
                    } else if ( menuState.menu_number == 1 ) {
                        alarm_max_pressure = menuState.encoderPos;
                    } else if ( menuState.menu_number == 2 ) {
                        min_cd = int(menuState.encoderPos);
                    } else if ( menuState.menu_number == 3 ) {
                        dpip_b = menuState.encoderPos;
                        dpip = float(menuState.encoderPos)/10.;
                    }
                    break;
                case 2:
                    if ( menuState.menu_number == 0 ) {
                        options.respiratoryRate = menuState.encoderPos;
                    } else if ( menuState.menu_number == 1 ) {
                        alarm_peep_pressure = menuState.encoderPos;
                    } else if ( menuState.menu_number == 2 ) {
                        min_speed = int((float)menuState.encoderPos*10.);
                    } else if ( menuState.menu_number == 3 ) {
                        Serial.print("encoderPos: ");Serial.println(menuState.encoderPos);
                        pfmin = menuState.encoderPos;
                        pf_min = (float)menuState.encoderPos/50.;
                        peep_fac = -(pf_max - pf_min)/15.* sensorParams.last_pressure_min + pf_max;
                    }
                    break;
                case 3:
                    if ( menuState.menu_number == 0 ) {
                        options.percInspEsp = menuState.encoderPos;
                    } else if ( menuState.menu_number == 1 ) {
                        alarm_vt = int(10.*(float)menuState.encoderPos);
                    } else if ( menuState.menu_number == 2 ) {
                        min_accel = int((float)menuState.encoderPos*10.);
                    } else if ( menuState.menu_number == 3 ) {
                        pfmax = menuState.encoderPos;
                        pf_max = (float)menuState.encoderPos/50.;
                        peep_fac = -(pf_max-pf_min)/15.* sensorParams.last_pressure_min + pf_max;
                    }
                    break;
                case 4:
                    if ( menuState.menu_number == 0 ) {
                        if ( systemState.vent_mode == VENTMODE_PCL ) {
                            options.peakInspiratoryPressure = menuState.encoderPos;
                            Serial.print("pip: ");Serial.println(options.peakInspiratoryPressure);
                            Serial.print("encoderpos: ");Serial.println(menuState.encoderPos);
                        } else { //manual
                            options.percVolume = menuState.encoderPos;
                        }
                    } else if ( menuState.menu_number == 1 ) {
                        menuState.p_trim = menuState.encoderPos;
                    } else if ( menuState.menu_number == 2 ) {
                        max_cd  = int(menuState.encoderPos);
                    } else if ( menuState.menu_number == 3 ) {
                        p_acc = menuState.encoderPos;
                    }
                    break;
                case 5:
                    if ( menuState.menu_number == 0 ) {
                        //options.peakInspiratoryPressure = encoderPos;
                    } else if ( menuState.menu_number == 1 ) {
                        autopid = menuState.encoderPos;
                    } else if ( menuState.menu_number == 2 ) {
                        max_speed = int((float)menuState.encoderPos*10.);
                    } else if ( menuState.menu_number == 3 ) {
                        f_acc_b = menuState.encoderPos;
                        f_acc = (float)f_acc_b/10.;
                    }
                    break;
                case 6:
                    if ( menuState.menu_number == 0 ) {
                        options.peakEspiratoryPressure = menuState.encoderPos;
                    } else if ( menuState.menu_number == 1 ) { //There is not 6 in menu 1
                        filter = menuState.encoderPos == 1;
                    } else if ( menuState.menu_number == 2 ) { //There is not 6 in menu 1
                        max_accel  = int((float)menuState.encoderPos*10.);
                    }
                    break;
                case 7:
                    if ( menuState.menu_number == 2 ) {
                        min_pidk = menuState.encoderPos*10;
                    }
                    break;
                case 8:
                    if ( menuState.menu_number == 2 ) {
                        min_pidi = menuState.encoderPos*10;
                    }
                    break;
                case 9:
                    if ( menuState.menu_number == 2 ) {
                        min_pidd = menuState.encoderPos*2;
                    }
                    break;
                case 10:
                    if ( menuState.menu_number == 2 ){
                        max_pidk = menuState.encoderPos*2;
                    }
                    break;
                case 11:
                    if ( menuState.menu_number == 2 ) {
                        max_pidi = int(menuState.encoderPos)*2;
                        Serial.print("Max pid i:");Serial.println(max_pidi);
                        Serial.print("Encoder pos:");Serial.println(menuState.encoderPos);
                    }
                    break;
                case 12:
                    if ( menuState.menu_number == 2 ) {
                        max_pidd = menuState.encoderPos*2;
                    }
                    break;
                }//switch
                menuState.show_changed_options = true;
                menuState.update_options = true;
            }//Valid range

            menuState.old_curr_sel = menuState.curr_sel;

        }//oldEncPos != encoderPos and valid between range
    }
}

void clear_n_sel(int menu, int curr_selection, bool item_sel, byte ventilationMode) {
    if (menu == 0) {
        lcd_clearxy(0,0);
        lcd_clearxy(0,1);
        lcd_clearxy(9,0);
        lcd_clearxy(0,2);
        lcd_clearxy(8,1);
        switch(curr_selection) {
            case 1:
                lcd_selxy(0, 0, item_sel);
                break;
            case 2:
                lcd_selxy(0, 1, item_sel);
                break;
            case 3:
                lcd_selxy(0, 2, item_sel);
                break;
            case 4:
                if ( ventilationMode==VENTMODE_VCL || ventilationMode==VENTMODE_PCL) {
                    lcd_selxy(8, 1, item_sel);//pcl
                } else {
                    lcd_selxy(9, 0, item_sel);
                }
                break;
        }
    } else if ( menu == 1 ) {
        lcd_clearxy(0,0);
        lcd_clearxy(0,1);
        lcd_clearxy(12,2);
        lcd_clearxy(0,2);
        lcd_clearxy(0,3);
        switch(curr_selection){
            case 1:
                lcd_selxy(0, 0, item_sel);
                break;//PIP
            case 2:
                lcd_selxy(0, 1, item_sel);
                break;//PEEP
            case 3:
                lcd_selxy(10, 1, item_sel);
                break;
            case 4:
                lcd_selxy(0, 2, item_sel);
                break;
            case 5:
                lcd_selxy(0, 3, item_sel);
                break;
            case 6:
                lcd_selxy(12, 2, item_sel);
                break;
        }
    } else if ( menu == 2 ) {
        lcd_clearxy(0,0);
        lcd_clearxy(6,0);
        lcd_clearxy(12,0);
        lcd_clearxy(0,1);
        lcd_clearxy(6,1);
        lcd_clearxy(12,1);
        lcd_clearxy(0,2);
        lcd_clearxy(0,3);
        switch(curr_selection) {
            case 1:
                lcd_selxy(0, 0, item_sel);
                break;//PIP
            case 2:
                lcd_selxy(6, 0, item_sel);
                break;//PEEP
            case 3:
                lcd_selxy(12, 0, item_sel);
                break;
            case 4:
                lcd_selxy(0, 1, item_sel);
                break;//PIP
            case 5:
                lcd_selxy(6, 1, item_sel);
                break;
            case 6:
                lcd_selxy(12, 1, item_sel);
                break;
            case 7:
                lcd_selxy(0, 2, item_sel);
                break;
            case 8:
                lcd_selxy(6, 2, item_sel);
                break;
            case 9:
                lcd_selxy(12, 2, item_sel);
                break;
            case 10:
                lcd_selxy(0, 3, item_sel);
                break;
            case 11:
                lcd_selxy(6,3, item_sel);
                break;
            case 12:
                lcd_selxy(12, 3, item_sel);
                break;
        }
    } else if ( menu == 3 ) {
        lcd_clearxy(0,0);
        lcd_clearxy(6,0);
        lcd_clearxy(12,0);
        lcd_clearxy(0,1);
        lcd_clearxy(6,1);
        lcd_clearxy(12,1);
        lcd_clearxy(0,2);
        lcd_clearxy(0,3);
        switch(curr_selection) {
            case 1:
                lcd_selxy(0, 0, item_sel);
                break;//PIP
            case 2:
                lcd_selxy(0, 1, item_sel);
                break;//PEEP
            case 3:
                lcd_selxy(7, 1, item_sel);
                break;
            case 4:
                lcd_selxy(0, 2, item_sel);
                break;//PIP
            case 5:
                lcd_selxy(7, 2, item_sel);
                break;
        }
    }//menu number 
}

void display_lcd(MenuState& menuState, byte ventilationMode, SensorParams& sensorParams) {
    if ( menuState.clear_all_display ) {
        #if TESTING_MODE_DISABLED
        lcd.clear();
        #endif //TESTING_MODE_DISABLED
    }
    clear_n_sel(menuState.menu_number, menuState.curr_sel, menuState.isitem_sel, ventilationMode);
    char tempstr[5];
    if ( menuState.menu_number == 0 ) {
        lcd_clearxy(12,0,4);
        lcd_clearxy(5,1,3);
        lcd_clearxy(14,1,2);
        lcd_clearxy(5,2,2);
        lcd_clearxy(13,2,2);
  
        switch ( ventilationMode ) {
        case VENTMODE_VCL:
            writeLine(0, "MOD:VCV", 1); writeLine(0, "V:" + String(options.tidalVolume), 10);
            writeLine(1, "PIP: -", 9);
            break;
        case VENTMODE_PCL:
            writeLine(0, "MOD:PCV", 1);
            writeLine(1, "PIP:" + String(options.peakInspiratoryPressure), 9);
            writeLine(0, "V: -", 10);
            break;
        case VENTMODE_MAN:
            writeLine(0, "MOD:VCV", 1);
            writeLine(0, "V:" + String(options.percVolume)+"%", 10);
            writeLine(1, "PIP: -", 9);
            break;
        }

        writeLine(1, "BPM:" + String(options.respiratoryRate), 1);
        writeLine(2, "IE:1:", 1);
  
        dtostrf((_mllastInsVol+_mllastExsVol)/2, 4, 0, tempstr);
        writeLine(0, String(tempstr), 16);
  
        writeLine(2, String(options.percInspEsp), 6);
  
        #ifdef DEBUG_UPDATE
        Serial.print("Presion mostrada: ");Serial.println(sensorParams.pressure_max);
        #endif
        dtostrf(sensorParams.last_pressure_max, 2, 0, tempstr);
        writeLine(1, String(tempstr), 16);
    
        #ifdef DEBUG_UPDATE
        Serial.print("Max press conv: ");Serial.println(tempstr);
        Serial.print("Min Max press");  Serial.print(sensorParams.pressure_min);
        Serial.print(" ");Serial.println(sensorParams.pressure_max);
        #endif
      
        writeLine(2, "PEEP: ", 11);
        dtostrf(sensorParams.last_pressure_min, 2, 0, tempstr);
        writeLine(2, String(tempstr), 16);
    
        dtostrf((_mllastInsVol + _mllastExsVol)/2.*options.respiratoryRate*0.001, 2, 1, tempstr);
        writeLine(3, "VM:" + String(tempstr), 0);
    
        dtostrf(_timeoutIns*0.001, 1, 1, tempstr);
        writeLine(3, "I:" + String(tempstr), 9);
        dtostrf(_timeoutEsp*0.001, 1, 1, tempstr);
        writeLine(3, "E:" + String(tempstr), 15);

    } else if ( menuState.menu_number == 1 ) {//OTHER SETTINGS
        lcd_clearxy(12,0,8);
        lcd_clearxy(8,1,2);
        lcd_clearxy(16,1,3);
        lcd_clearxy(15,2,3);
        
        writeLine(0, "PIPAL:" + String(alarm_max_pressure), 1);
    
        dtostrf(Cdyn*1.01972, 2, 1, tempstr);
        writeLine(0, "CD:" + String(tempstr), 10);
    
        writeLine(1, "PEEPAL:" + String(alarm_peep_pressure), 1);
        writeLine(1, "VTAL:" + String(alarm_vt), 11);
    
        dtostrf((float(menuState.p_trim-100)), 2, 0, tempstr);
        writeLine(2, "TRIM:" + String(tempstr) + "e-3", 1);

        writeLine(2, "F:" , 13);
        if (filter) {
            writeLine(2, "ON", 15);
        } else {
            writeLine(2, "OFF", 15);
        }
        writeLine(3, "AUTO: ", 1);
        if (autopid) {
            writeLine(3, "ON", 6);
        } else {
            writeLine(3, "OFF", 6);
        }

        writeLine(3, "C:", 10);
        writeLine(3, String(last_cycle), 12);

    } else if ( menuState.menu_number == 2 ) {//PID

        for (int i=0 ; i<3 ; i++) {
           lcd_clearxy(3,i,3);
           lcd_clearxy(9,i,3);
           lcd_clearxy(15,i,3);
        }
        
        writeLine(0, "c:" + String(min_cd), 1);
        writeLine(1, "C:" + String(max_cd), 1);
    
        writeLine(0, "v:" + String(min_speed), 7);
        writeLine(1, "V:" + String(max_speed), 7);

        writeLine(0, "a:" + String(min_accel), 13);
        writeLine(1, "A:" + String(max_accel), 13);

        writeLine(2, "p:" + String(min_pidk), 1);
        writeLine(3, "P:" + String(max_pidk), 1);
    
        writeLine(2, "i:" + String(min_pidi), 7);
        writeLine(3, "I:" + String(max_pidi), 7);

        writeLine(2, "d:" + String(min_pidd), 13);
        writeLine(3, "D:" + String(max_pidd), 13);

    } else if ( menuState.menu_number == 3 ) {//PID Config 2
        lcd_clearxy(3,0,2);
        lcd_clearxy(9,0,3);
        lcd_clearxy(15,0,3);
        lcd_clearxy(3,1,2);
        lcd_clearxy(9,1,3);
        lcd_clearxy(15,1,3);
        lcd_clearxy(3,2,3);
        lcd_clearxy(3,3,3);
        
        writeLine(0, "dp:" + String(dpip), 1);
        
        dtostrf(pf_min, 1, 2, tempstr);writeLine(1, "f:"   + String(tempstr), 1);
        dtostrf(pf_max, 1, 2, tempstr);writeLine(1, "F:"   + String(tempstr), 8);

        writeLine(2, "pa:" + String(p_acc), 1);
        dtostrf(f_acc, 1, 2, tempstr);writeLine(2, "fa:"   + String(tempstr), 8);
    
    }//menu_number

    menuState.clear_all_display = false;
}

// private functions

void updateState(MenuState& menuState) {
    // the button has been just pressed
    if (menuState.bck_state == LOW) {
        menuState.startPressed = time;
        //        menuState.idleTime = menuState.startPressed - menuState.endPressed;
        menuState.change_sleep=false;
        // the button has been just released
    } else {
        menuState.endPressed = time;
        menuState.holdTime = menuState.endPressed - menuState.startPressed;
    }
}


void updateCounter(MenuState& menuState) {
    // the button is still pressed
    if (menuState.bck_state == LOW) {
        menuState.holdTime = time - menuState.startPressed;
        // the button is still released
    } else {
//        menuState.idleTime = time - menuState.endPressed;
    }
}

void lcd_clearxy(int x, int y,int pos) {
    #if TESTING_MODE_DISABLED
    for (int i=0;i<pos;i++) {
        lcd.setCursor(x+i, y);
        lcd.print(" ");
    }
    #endif //TESTING_MODE_DISABLED
}
void lcd_selxy(int x, int y, bool item_sel) {
    #if TESTING_MODE_DISABLED
    lcd.setCursor(x, y);
    if (!item_sel) {
        lcd.print(">");
    } else {
        lcd.write(byte(0));
    }
    #endif //TESTING_MODE_DISABLED
}

void check_updn_button(int pin, byte *var, bool incr_decr, unsigned long& lastButtonPress) {
    if (digitalRead(pin)==LOW) { //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6)
        if (time - lastButtonPress > 150) {
            if (incr_decr) {
                *var=*var+1;
            } else {
                *var=*var-1;
            }
            lastButtonPress = time;
        }// if time > last button press
    }
}