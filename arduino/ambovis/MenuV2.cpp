#include "MenuV2.h"

void init_display(Menu& menu) {
    menu.lcd.begin(20, 4);
    menu.lcd.clear();
    menu.lcd.setCursor(0, 0);
    menu.lcd.createChar(0, back);
}

void writeLine(Menu& menu, int line, String message = "", int offsetLeft = 0) {
    menu.lcd.setCursor(0, line);
    menu.lcd.print("");
    menu.lcd.setCursor(offsetLeft, line);
    menu.lcd.print(message);
}

void clearDigits(Menu& menu, int x, int y, int pos=1) {
    for (int i=0; i<pos; i++) {
        menu.lcd.setCursor(x+i, y);
        menu.lcd.print(" ");
    }
}

void selectDigit(Menu& menu, int x, int y) {
    menu.lcd.setCursor(x, y);
    if (!menu.itemSelected) {
        menu.lcd.print(">");
    } else {
        // print the custom back digit
        menu.lcd.write(byte(0));
    }
}
//void moveCursorInMenu(Menu& menu, VariableParameters& parameters) {
//    switch (menu.menuNumber) {
//        case MAIN_MENU:
//            int currentCursor = (menu.currentSelection + menu.menuKeyboard.value) % 4;
//            switch (currentCursor) {
//                case 0:
//                    menu.currentSelection = GO_TO_PARAMETER_MENU;
//                    break;
//                case 1:
//                    menu.currentSelection = GO_TO_ALARM_MENU;
//                    break;
//                case 2:
//                    menu.currentSelection = GO_TO_SETTINGS_MENU;
//                    break;
//                case 3:
//                    menu.currentSelection = GO_TO_PID_SETTINGS_MENU;
//                    break;
//            }
//            break;
//        case PARAMETER_MENU:
//            int currentCursor = (menu.currentSelection + menu.menuKeyboard.value) % 4;
//            switch (currentCursor) {
//                case 0:
//                    menu.currentSelection = MODE_OPT;
//                    break;
//                case 1:
//                    menu.currentSelection = parameters.vent_mode == VENTMODE_MAN ? PERC_V_OPT : PIP_OPT;
//                    break;
//                case 2:
//                    menu.currentSelection = BPM_OPT;
//                    break;
//                case 3:
//                    menu.currentSelection = IE_OPT;
//                    break;
//            }
//            break;
//        case ALARM_MENU:
//            int currentCursor = (menu.currentSelection + menu.menuKeyboard.value) % 5;
//            switch (currentCursor) {
//                case 0:
//                    menu.currentSelection = PIP_ALARM_OPT;
//                    break;
//                case 1:
//                    menu.currentSelection = PEEP_ALARM_OPT;
//                    break;
//                case 2:
//                    menu.currentSelection = VT_ALARM_OPT;
//                    break;
//                case 3:
//                    menu.currentSelection = VM_ALARM_OPT;
//                    break;
//                case 4:
//                    menu.currentSelection = AMBU_ALARM_OPT;
//                    break;
//            }
//            break;
//        case SETTINGS_MENU:
//            break;
//        case PID_SETTINGS_MENU:
//            break;
//    }
//}
//
//void doMainMenuAction(Menu& menu) {
//    switch (menu.currentSelection) {
//        case GO_TO_PARAMETER_MENU:
//            menu.menuNumber = PARAMETER_MENU;
//            menu.updateDisplay = true;
//            break;
//        case GO_TO_ALARM_MENU:
//            menu.menuNumber = ALARM_MENU;
//            menu.updateDisplay = true;
//            break;
//        case GO_TO_SETTINGS_MENU:
//            menu.menuNumber = SETTINGS_MENU;
//            menu.updateDisplay = true;
//            break;
//        case GO_TO_PID_SETTINGS_MENU:
//            menu.menuNumber = PID_SETTINGS_MENU;
//            menu.updateDisplay = true;
//            break;
//    }
//}
//
//int applyValidRange(int value, int min, int max) {
//    if (value < min) {
//        return min;
//    } else if (value > max) {
//        return max;
//    } else {
//        return value;
//    }
//}
//
//void doParameterMenuAction(Menu& menu, VariableParameters& parameters) {
//    switch (menu.currentSelection) {
//        case MODE_OPT:
//            int currentMode = parameters.vent_mode;
//            int newSelectedMode = menu.menuKeyboard.value;
//            newSelectedMode = applyValidRange(newSelectedMode, 1, 2);
//            parameters.vent_mode = newSelectedMode;
//            break;
//        case PERC_V_OPT:
//            int currentPercVol = parameters.percVolume;
//            int newPercVol = menu.menuKeyboard.value;
//            newPercVol = applyValidRange(newPercVol, 40, 100); //TODO extract default
//            parameters.percVolume = newPercVol;
//            break;
//        case BPM_OPT:
//            int currentBpm = parameters.respiratoryRate;
//            int newBpm = menu.menuKeyboard.value;
//            newBpm = applyValidRange(newBpm, DEFAULT_MIN_RPM, DEFAULT_MAX_RPM);
//            parameters.respiratoryRate = newBpm;
//            break;
//        case IE_OPT:
//            int currentIE = parameters.percInspEsp;
//            int newIE = menu.menuKeyboard.value;
//            newIE = applyValidRange(newIE, 1, 3); //TODO extract default
//            parameters.percInspEsp = newIE;
//            break;
//        case PIP_OPT:
//            int currentPip = parameters.peakInspiratoryPressure;
//            int newPip = menu.menuKeyboard.value;
//            newPip = applyValidRange(newPip, 15, 30); //TODO extract default
//            parameters.peakInspiratoryPressure = newPip;
//            break;
//    }
//}
//
//void updateSelection(Menu& menu, VariableParameters& parameters) {
//    switch (menu.menuNumber) {
//        case MAIN_MENU:
//            doMainMenuAction(menu);
//            break;
//        case PARAMETER_MENU:
//            doParameterMenuAction(menu, parameters);
//            break;
//        case ALARM_MENU:
//            break;
//        case SETTINGS_MENU:
//            break;
//        case PID_SETTINGS_MENU:
//            break;
//    }
//}
//
//void checkEncoder(Menu& menu, SystemState& systemState, VariableParameters& parameters) {
//    checkUPButtonPressed(menu.menuKeyboard, time);
//    checkDOWNButtonPressed(menu.menuKeyboard, time);
//    checkOKButtonPressed(menu.menuKeyboard, time);
//    checkBackButtonPressed(menu.menuKeyboard, time);
//
//    if (menu.menuKeyboard.itemSelected) {
//        updateSelection(menu, parameters);
//    } else {
//        moveCursorInMenu(menu, parameters);
//    }
//}


//    if (pressed > 0) { //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6)
//        if (!isitem_sel) {
//            curr_sel=oldEncPos=encoderPos=old_curr_sel;
//        }
//
//        if (isitem_sel) {
//            switch (curr_sel) {
//            case 1:
//                if ( menu_number == 0 ) {
//                    min_sel=1;max_sel=2;
//                    encoderPos=oldEncPos=systemState.vent_mode;
//                } else if ( menu_number == 1 ) {
//                    min_sel=20;max_sel=50;
//                    encoderPos=oldEncPos=alarm_max_pressure;
//                } else if ( menu_number == 2 ) {
//                    encoderPos = STEPPER_ACCEL_MAX/200;
//                    min_sel=0;max_sel=8000;
//                } else if ( menu_number == 3 ) {
//                    encoderPos=dpip_b;
//                    min_sel=10;max_sel=40;
//                }
//                break;
//            case 2:
//                if ( menu_number == 0 ) {
//                    encoderPos=oldEncPos=options.respiratoryRate;
//                    min_sel=DEFAULT_MIN_RPM;max_sel=DEFAULT_MAX_RPM;
//                } else if ( menu_number == 1 ) {
//                    min_sel=5;max_sel=30;
//                    encoderPos=oldEncPos=alarm_peep_pressure;
//                } else if ( menu_number == 2 ){
//                    encoderPos=STEPPER_SPEED_MAX/200;
//                    min_sel=10;max_sel=8000;
//                } else if ( menu_number == 3 ){
//                    encoderPos=pfmin=50.*pf_min;
//                    min_sel=0;max_sel=99;
//                }
//                break;
//            case 3:
//                if ( menu_number == 0 ) {
//                    encoderPos=oldEncPos=options.percInspEsp;
//                    min_sel=1;max_sel=3;
//                } else if ( menu_number == 1 ) {
//                    encoderPos=byte(0.1*float(alarm_vt));
//                    min_sel=10;max_sel=50;//vt
//                } else if ( menu_number == 2 ) {
//                    encoderPos=min_accel/10;
//                    min_sel=10;max_sel=100;
//                } else if ( menu_number == 3 ){
//                    encoderPos=pfmax=50.*pf_max;
//                    min_sel=0;max_sel=99;
//                }
//                break;
//            case 4:
//                if ( menu_number == 0 ) {
//                    if ( systemState.vent_mode==VENTMODE_PCL) {
//                        encoderPos=oldEncPos=options.peakInspiratoryPressure;
//                        min_sel=15;max_sel=30;
//                        Serial.print("pip: ");Serial.println(options.peakInspiratoryPressure);
//                        Serial.print("encoderpos: ");Serial.println(encoderPos);
//                    } else {//Manual
//                        encoderPos=oldEncPos=options.percVolume;
//                        min_sel=40;max_sel=100;
//                    }
//                } else if ( menu_number == 1 ) {//menu 0
//                    encoderPos=oldEncPos=p_trim;
//                    min_sel=0;max_sel=200;
//                } else if ( menu_number == 2 ) {
//                    encoderPos=max_cd;
//                    min_sel=10;max_sel=80;
//                } else if ( menu_number == 3 ) {
//                    encoderPos=p_acc;
//                    min_sel=10;max_sel=40;
//                }
//                break;
//            case 5:
//                if ( menu_number == 0 ) {
////                  encoderPos=oldEncPos=options.peakInspiratoryPressure;
////                  min_sel=15;max_sel=30;
//                } else if ( menu_number == 1 ) {//menu 0
//                    min_sel=0;max_sel=1;
//                } else if ( menu_number == 2 ){
//                    encoderPos=max_speed/10;
//                    min_sel=10;max_sel=100;
//                } else if ( menu_number == 3 ) {
//                    encoderPos=f_acc_b;
//                    min_sel=10;max_sel=40;
//                }
//                break;
//            case 6:
//                if ( menu_number == 1 ){
//                    if (filter) {
//                        encoderPos=1;
//                    } else {
//                        encoderPos=0;
//                    }
//                    min_sel=0;max_sel=1;
//                } else if ( menu_number == 2 ){
//                    encoderPos=max_accel/10;
//                    min_sel=10;max_sel=100;
//                }
//                break;
//            case 7:
//                if ( menu_number == 2 ){
//                    encoderPos=min_pidk/10;
//                    min_sel=10;max_sel=99;
//                }
//                break;
//            case 8:
//                if ( menu_number == 2 ){
//                    encoderPos=byte(min_pidi/2);
//                    min_sel=2;max_sel=200;
//                }
//                break;
//            case 9:
//                if ( menu_number == 2 ){
//                    encoderPos=byte(min_pidd/2);
//                    min_sel=2;max_sel=200;
//                }
//                break;
//            case 10:
//                if ( menu_number == 2 ){
//                    encoderPos=max_pidk/10;
//                    min_sel=2;max_sel=200;
//                }
//                break;
//            case 11:
//                if ( menu_number == 2 ){
//                    encoderPos=byte(max_pidi/2);
//                    min_sel=2;max_sel=200;
//                }
//                break;
//            case 12:
//                if ( menu_number == 2 ) {
//                    encoderPos=byte(max_pidd/2);
//                    min_sel=2;max_sel=200;
//                }
//                break;
//            }
//        }//if switch select
//        show_changed_options = true;
//        update_options = true;
//    }//If selection
//
//    if (oldEncPos != encoderPos) {
//        show_changed_options = true;
//
//        if (!isitem_sel) { //Selecting position
//            curr_sel=encoderPos;
//            encoderPos=oldEncPos=curr_sel;
//
//            if ( menu_number == 0 ) {
//                if (encoderPos > 4) {
//                    encoderPos=1;
//                    menu_number+=1;
//                } else if ( encoderPos < 1) {
//                    encoderPos=5;
//                    menu_number=3;
//                }
//            } else if (menu_number == 1) {
//                if (encoderPos > 6) {
//                    encoderPos=1;
//                    menu_number=2;
//                } else if ( encoderPos < 1) {
//                    encoderPos=4;
//                    menu_number=0;
//                }
//            } else if (menu_number == 2) {
//                if (curr_sel > 2) {
//                    encoderPos=1;
//                    menu_number=3;
//                } else if ( encoderPos < 1) {
//                    encoderPos=6;
//                    menu_number=1;
//                }
//            } else if (menu_number == 3) {
//                if (curr_sel > 5) {
//                    encoderPos=1;
//                    menu_number=0;
//                } else if ( encoderPos < 1) {
//                    encoderPos=2;
//                    menu_number=2;
//                }
//            }
//            clear_all_display=true;
//            display_lcd(systemState);
//        } else {//inside a particular selection
//
//            //if (curr_sel != 0) {
//            if ( encoderPos > max_sel ) {
//                encoderPos=oldEncPos=max_sel;
//            } else if ( encoderPos < min_sel ) {
//                encoderPos=oldEncPos=min_sel;
//            } else {
//                oldEncPos = encoderPos;
//                switch (curr_sel) {
//                case 1:
//                    if ( menu_number == 0 )     systemState.vent_mode           = encoderPos;
//                    else if (menu_number == 1)  alarm_max_pressure  = encoderPos;
//                    else if (menu_number == 2)  {STEPPER_ACCEL_MAX  = int((float)encoderPos*200.);}
//                    else if (menu_number == 3)  {dpip_b = encoderPos; dpip  = float(encoderPos)/10.;}
//                    break;
//                case 2:
//                    if ( menu_number == 0 )       options.respiratoryRate = encoderPos;
//                    else  if (menu_number == 1)   alarm_peep_pressure     = encoderPos;
//                    else  if (menu_number == 2)   STEPPER_SPEED_MAX  = int((float)encoderPos*200.);
//                    else if ( menu_number == 3 ){
//                        Serial.print("encoderPos: ");Serial.println(encoderPos);
//                        pfmin=encoderPos;
//                        pf_min=(float)encoderPos/50.;
//                        peep_fac = -(pf_max-pf_min)/15.*last_pressure_min + pf_max;
//                    }
//                    break;
//                case 3:
//                    if ( menu_number == 0 ) options.percInspEsp=encoderPos;
//                    else    if (menu_number == 1) alarm_vt=int(10.*(float)encoderPos);
//                    else    if (menu_number == 2) min_accel  = int((float)encoderPos*10.);
//                    if ( menu_number == 3 ){
//                        pfmax=encoderPos;
//                        pf_max=(float)encoderPos/50.;
//                        peep_fac = -(pf_max-pf_min)/15.*last_pressure_min + pf_max;
//                    }
//                    break;
//                case 4:
//                    if ( menu_number == 0 ) {
//                        if (systemState.vent_mode==VENTMODE_PCL){
//                            options.peakInspiratoryPressure = encoderPos;
//                            Serial.print("pip: ");Serial.println(options.peakInspiratoryPressure);
//                            Serial.print("encoderpos: ");Serial.println(encoderPos);
//                        } else { //manual
//                            options.percVolume = encoderPos;
//                        }
//                    } else if (menu_number == 1) {
//                        p_trim=encoderPos;
//                    } else if (menu_number == 2) {
//                        max_cd  = int(encoderPos);
//                    } else if (menu_number == 3) {
//                        p_acc=encoderPos;
//                    }
//                    break;
//                case 5:
//                    if ( menu_number == 0 ) {
//                        //options.peakInspiratoryPressure = encoderPos;
//                    } else if (menu_number == 1) {
//                        autopid=encoderPos;
//                    } else if (menu_number == 2) {
//                        max_speed  = int((float)encoderPos*10.);
//                    } else if (menu_number == 3) {f_acc_b=encoderPos;f_acc=(float)f_acc_b/10.;}
//                    break;
//                case 6:
//                    if ( menu_number == 0 )
//                        options.peakEspiratoryPressure = encoderPos;
//                    else if ( menu_number == 1 )  //There is not 6 in menu 1
//                        if (encoderPos==1) filter  = true;
//                        else                filter=false;
//                    else if ( menu_number == 2 )  //There is not 6 in menu 1
//                        max_accel  = int((float)encoderPos*10.);
//                    break;
//                case 7:
//                    if ( menu_number == 2 ){
//                        min_pidk=encoderPos*10;
//                    }
//                    break;
//                case 8:
//                    if ( menu_number == 2 ){
//                        min_pidi=encoderPos*10;
//                    }
//                    break;
//                case 9:
//                    if ( menu_number == 2 ){
//                        min_pidd=encoderPos*2;
//                    }
//                    break;
//                case 10:
//                    if ( menu_number == 2 ){
//                        max_pidk=encoderPos*2;
//                    }
//                    break;
//                case 11:
//                    if ( menu_number == 2 ){
//                        max_pidi=int(encoderPos)*2;
//                        Serial.print("Max pid i:");Serial.println(max_pidi);
//                        Serial.print("Encoder pos:");Serial.println(encoderPos);
//                    }
//                    break;
//                case 12:
//                    if ( menu_number == 2 ){
//                        max_pidd=encoderPos*2;
//                    }
//                    break;
//                }//switch
//                show_changed_options = true;
//                update_options=true;
//            }//Valid range
//
//            old_curr_sel = curr_sel;
//            if (menu_number==2)
//                change_pid_params=true;
//        }//oldEncPos != encoderPos and valid between range
//    }
//}
//
//void clear_n_sel(int menu, SystemState& systemState){
//    if (menu==0) {
//        lcd_clearxy(0,0);
//        lcd_clearxy(0,1);lcd_clearxy(9,0);
//        lcd_clearxy(0,2);lcd_clearxy(8,1);
//         switch(curr_sel) {
//             case 1:
//                 lcd_selxy(0,0);break;
//             case 2:
//                lcd_selxy(0,1);break;
//             case 3:
//                lcd_selxy(0,2);break;
//             case 4:
//                if ( systemState.vent_mode==VENTMODE_VCL || systemState.vent_mode==VENTMODE_PCL) lcd_selxy(8,1);//pcl
//                else lcd_selxy(9,0);
//             }
//    } else if (menu==1) {
//        lcd_clearxy(0,0);
//        lcd_clearxy(0,1);lcd_clearxy(12,2);
//        lcd_clearxy(0,2);lcd_clearxy(0,3);
//        switch(curr_sel) {
//            case 1:
//                lcd_selxy(0,0);break;//PIP
//            case 2:
//                lcd_selxy(0,1);break;//PEEP
//            case 3:
//                lcd_selxy(10,1);break;
//            case 4:
//                lcd_selxy(0,2);break;
//            case 5:
//                lcd_selxy(0,3);break;
//            case 6:
//                lcd_selxy(12,2);break;
//        }
//    } else if (menu==2) {
//        lcd_clearxy(0,0);lcd_clearxy(9,0);
//        lcd_clearxy(0,1);lcd_clearxy(6,1);
//        lcd_clearxy(0,2);
//        lcd_clearxy(0,3);
//        switch(curr_sel) {
//            case 1:
//                lcd_selxy(0,0);break;//PIP
//            case 2:
//                lcd_selxy(9,0);break;//PEEP
//            case 3:
//                lcd_selxy(12,0);break;
//            case 4:
//                lcd_selxy(0,1);break;//PIP
//            case 5:
//                lcd_selxy(6,1);break;
//            case 6:
//                lcd_selxy(12,1);break;
//            case 7:
//                lcd_selxy(0,2);break;
//            case 8:
//                lcd_selxy(6,2);break;
//            case 9:
//                lcd_selxy(12,2);break;
//            case 10:
//                lcd_selxy(0,3);break;
//            case 11:
//                lcd_selxy(6,3);break;
//            case 12:
//                lcd_selxy(12,3);break;
//        }
//    }//menu number
//    else if (menu==3) {
//        lcd_clearxy(0,0);
//        lcd_clearxy(6,0);
//        lcd_clearxy(12,0);
//        lcd_clearxy(0,1);
//        lcd_clearxy(6,1);
//        lcd_clearxy(12,1);
//        lcd_clearxy(0,2);
//        lcd_clearxy(0,3);
//        switch(curr_sel) {
//            case 1:
//                lcd_selxy(0,0);break;//PIP
//            case 2:
//                lcd_selxy(0,1);break;//PEEP
//            case 3:
//                lcd_selxy(7,1);break;
//            case 4:
//                lcd_selxy(0,2);break;//PIP
//            case 5:
//                lcd_selxy(7,2);break;
//        }
//    }//menu number
//}
//

void displayMainMenu(Menu& menu, SystemState& systemState) {
    menu.lcd.clear();
    writeLine(menu, 0, "Seleccione un menu", 1);
    writeLine(menu, 1, "Parametros Principales", 1);
    writeLine(menu, 2, "Alarmas", 1);
    writeLine(menu, 3, "Ajustes", 1);
    writeLine(menu, 4, "Ajustes PID 1", 1);
}

void displaySensorValues(int line, Menu& menu, SystemState& systemState) {
    writeLine(menu, line, "PIP:-", 1);
    //    dtostrf(last_pressure_min, 2, 0, tempstr);
    writeLine(menu, line, "PEEP:12.1", 12);
    //    dtostrf((_mllastInsVol + _mllastExsVol)/2.*options.respiratoryRate*0.001, 2, 1, tempstr);
    writeLine(menu, line, "VM:11.5", 20);
}

void displayMainParameters(Menu& menu, SystemState& systemState) {
    menu.lcd.clear();
    writeLine(menu, 0, "Parametros Principales", 1);
    if (systemState.vent_mode == VENTMODE_PCL) {
        writeLine(menu, 1, "MOD: PCV", 1);
        writeLine(menu, 1, "PIP:", 4);
    } else (systemState.vent_mode == VENTMODE_MAN ) {
        writeLine(menu, 1, "MOD: VCV", 1);
        writeLine(menu, 1, "%V:", 4);
        writeLine(menu, 1, "(Vt):", 8);
    }
    writeLine(menu, 2, "BPM:", 1);
    writeLine(menu, 2, "IE:", 4);
    writeLine(menu, 2, "ti:", 8);
    displaySensorValues(3, menu, systemState);
}


void displayAlarmSettings(Menu& menu, SystemState& systemState) {
    menu.lcd.clear();
    writeLine(menu, 0, "Alarmas", 1);
    writeLine(menu, 1, "PIP:", 1);
    writeLine(menu, 1, "PEEP:", 6);
    writeLine(menu, 2, "VT:", 1);
    writeLine(menu, 2, "VM:", 6);
    writeLine(menu, 3, "AMBU:", 1);
    writeLine(menu, 3, "CIC:", 6);
}

void display(Menu& menu, SystemState& systemState, bool shouldClearDisplay) {
    if (shouldClearDisplay) {
        menu.lcd.clear();
    }
//    clear_n_sel(menu_number, systemState);
    if ( menu.menuNumber == MAIN_MENU ) { //All the titles
        displayMainMenu(menu, systemState);
    } else if ( menu.menuNumber == PARAMETER_MENU ) { //Main Parameters
        displayMainParameters(menu, systemState);
    } else if ( menu.menuNumber == ALARM_MENU ) { //Alarms
        displayAlarmSettings(menu, systemState);
    }
//    else if (menu_number == 2 ) {//PID I
//        for (int i=0;i<3;i++){
//            lcd_clearxy(3,i,3);
//            lcd_clearxy(9,i,3);
//            lcd_clearxy(15,i,3);
//        }
//
//        writeLine(0, "a:" + String(STEPPER_ACCEL_MAX), 1);
//        writeLine(0, "s:" + String(STEPPER_SPEED_MAX), 10);
//        writeLine(1, "fs:" + String(STEPPER_SPEED_MAX), 1);
//
//    } else if (menu_number == 3 ){//PID Config 2
//        lcd_clearxy(3,0,2); lcd_clearxy(9,0,3);lcd_clearxy(15,0,3);
//        lcd_clearxy(3,1,2); lcd_clearxy(9,1,3);lcd_clearxy(15,1,3);
//        lcd_clearxy(3,2,3);
//        lcd_clearxy(3,3,3);
//
//        writeLine(0, "dp:" + String(dpip), 1);
//
//        dtostrf(pf_min, 1, 2, tempstr);writeLine(1, "f:"   + String(tempstr), 1);
//        dtostrf(pf_max, 1, 2, tempstr);writeLine(1, "F:"   + String(tempstr), 8);
//
//        writeLine(2, "pa:" + String(p_acc), 1);
//        dtostrf(f_acc, 1, 2, tempstr);writeLine(2, "fa:"   + String(tempstr), 8);
//    }//menu_number
//    clear_all_display=false;
}
