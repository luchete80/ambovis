#include <arduino.h>
#include "pinout.h"
#include "menu.h"
#include "MechVentilation.h"  //options

static bool clear_all_display;

static byte max_pidk_byte,min_pidk_byte;

bool change_sleep;
int pressed=0;  //0 nothing , 1 enter, 2 bck

byte opciones_mod[3];
byte cant_opciones_mod;
byte seleccion_mod[3];

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


///////////////////

Menu::Menu()
{}


void updateState() {
  // the button has been just pressed
  if (bck_state == LOW) {
      startPressed = time;
      idleTime = startPressed - endPressed;
      change_sleep=false;
//      if (idleTime >= 500 && idleTime < 1000) {
//          Serial.println("Button was idle for half a second");
//      }
//
//      if (idleTime >= 1000) {
//          Serial.println("Button was idle for one second or more"); 
//      }

  // the button has been just released
  } else {
      endPressed = time;
      holdTime = endPressed - startPressed;

//      if (holdTime >= 10 && holdTime < 1000) {
//          Serial.println("Button was hold for half a second"); 
//      }
//      if (holdTime >= 500 && holdTime < 1000) {
//          Serial.println("Button was hold for half a second"); 
//      }
//
//      if (holdTime >= 1000) {
//          Serial.println("Button was hold for one second or more"); 
//      }

  }
}

void updateCounter() {
  // the button is still pressed
  if (bck_state == LOW) {
      holdTime = time - startPressed;

//      if (holdTime >= 1000) {
//          Serial.println("Button is hold for more than a second"); 
//      }

  // the button is still released
  } else {
      idleTime = time - endPressed;

//      if (idleTime >= 1000) {
//          Serial.println("Button is released for more than a second");  
//      }
  }
}

void init_display() {
  #ifdef LCD_I2C
  lcd.begin(20, 4);  //I2C
#else
  lcd.begin(20, 4); //NO I2C
#endif
  //lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
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
void lcd_selxy(int x, int y) {
  lcd.setCursor(x, y);
  if (!isitem_sel)
      lcd.print(">");
  else 
      lcd.write(byte(0));
}

void check_updn_button(int pin, byte *var, bool incr_decr) {
    if (digitalRead(pin)==LOW) { //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6) 
      if (time - lastButtonPress > 150) {
          if (incr_decr)
              *var=*var+1;
          else
              *var=*var-1;
          lastButtonPress = time;
      }// if time > last button press
    }
}
void check_bck_state(){
      bck_state=digitalRead(PIN_MENU_BCK);         
//    Serial.print("holdTime:");Serial.println(holdTime);
//    Serial.print("change_sleep:");Serial.println(change_sleep);
//    Serial.print("bck_state:");Serial.println(bck_state);
//    Serial.print("sleep_mode:");Serial.println(sleep_mode);
        
    if (bck_state != last_bck_state) { 
       updateState(); // button state changed. It runs only once.
        if (bck_state == LOW ) { //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6) 
            if (time - lastButtonPress > 150) {
              pressed = 2;
              isitem_sel=false; 
              lastButtonPress = time;
            }// if time > last button press
        } else {  //Button released
          }
    } else {
       updateCounter(); // button state not changed. It runs in a loop.
       if (holdTime > 2000 && !change_sleep){
        Serial.println("Activando Sleep Mode");
        if (!sleep_mode){
            
            sleep_mode=true;
            put_to_sleep=true;
        } else {
           sleep_mode=false;
           wake_up=true;   
        }
        change_sleep=true;
        Serial.print("Sleep Mode");Serial.println(sleep_mode);
        }
    }
    last_bck_state = bck_state;
  
  }
  
void check_encoder ( ) {
  check_updn_button(PIN_MENU_DN,&encoderPos,true);   //Increment
  check_updn_button(PIN_MENU_UP,&encoderPos,false);  //Decrement
  pressed=0;  //0 nothing , 1 enter, 2 bck

    if (digitalRead(PIN_MENU_EN) == LOW)  //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6) v
    if (time - lastButtonPress > 150) {
      pressed = 1;
      isitem_sel=true; 
      lastButtonPress = time;
    }// if time > last button press

// ORIGINAL BCK WITHOUT SLEEP MODE
//    if (digitalRead(PIN_MENU_BCK) == LOW )  //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6) 
//        if (time - lastButtonPress > 150) {
//          pressed = 2;
//          isitem_sel=false; 
//          lastButtonPress = time;
//        }// if time > last button press

    check_bck_state();

    if (pressed > 0) { //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6) 
      if (!isitem_sel) {
        curr_sel=oldEncPos=encoderPos=old_curr_sel;
      }
                  
      if (isitem_sel) {
          switch (curr_sel){
            case 1: 
             if ( menu_number == 0 ) {
                    min_sel=1;max_sel=2;
                    encoderPos=oldEncPos=vent_mode;
                } else if ( menu_number == 1 ) {
                    min_sel=20;max_sel=50;
                    encoderPos=oldEncPos=alarm_max_pressure;            
             } else if ( menu_number == 2 ) {
                encoderPos=min_cd;
                min_sel=0;max_sel=max_cd;
             } else if ( menu_number == 3 ) {
                encoderPos=dpip_b;
                min_sel=10;max_sel=40;
             }
            break;
            case 2: 
                if ( menu_number == 0 ) {
                    encoderPos=oldEncPos=options.respiratoryRate;
                    min_sel=DEFAULT_MIN_RPM;max_sel=DEFAULT_MAX_RPM;
                    } else if ( menu_number == 1 ) {
                        min_sel=5;max_sel=30;
                        encoderPos=oldEncPos=alarm_peep_pressure;                          
                    } else if ( menu_number == 2 ){
                        encoderPos=min_speed/10;     
                        min_sel=10;max_sel=100;             
                    } else if ( menu_number == 3 ){
                        encoderPos=pfmin=50.*pf_min;
                        min_sel=0;max_sel=99;
                    }     
            break;
            case 3:
              if ( menu_number == 0 ) {
                  encoderPos=oldEncPos=options.percInspEsp;
                  min_sel=1;max_sel=3;   
              } else if ( menu_number == 1 ) {
                    encoderPos=byte(0.1*float(alarm_vt));
                    min_sel=10;max_sel=50;//vt
              } else if ( menu_number == 2 ) {
                    encoderPos=min_accel/10; 
                    min_sel=10;max_sel=100; 
              } else if ( menu_number == 3 ){
                        encoderPos=pfmax=50.*pf_max;
                        min_sel=0;max_sel=99;
                    }  
            break;
            case 4: 
                if ( menu_number == 0 ) {
                    if ( vent_mode==VENTMODE_PCL){
 //                     encoderPos=oldEncPos=options.tidalVolume;
 //                     min_sel=200;max_sel=800;
                        encoderPos=oldEncPos=options.peakInspiratoryPressure;
                        min_sel=15;max_sel=30;
                        Serial.print("pip: ");Serial.println(options.peakInspiratoryPressure);
                        Serial.print("encoderpos: ");Serial.println(encoderPos);
                    } else {//Manual
                      encoderPos=oldEncPos=options.percVolume;
                      min_sel=40;max_sel=100;            
                    } 
                } else if ( menu_number == 1 ) {//menu 0
                    encoderPos=oldEncPos=p_trim;
                    min_sel=0;max_sel=200;   
                } else if ( menu_number == 2 ) {
                    encoderPos=max_cd;
                    min_sel=10;max_sel=80;
                } else if ( menu_number == 3 ) {
                    encoderPos=p_acc;
                    min_sel=10;max_sel=40;
                }
                break;
                  case 5: 
                if ( menu_number == 0 ) {
//                    encoderPos=oldEncPos=options.peakInspiratoryPressure;
//                    min_sel=15;max_sel=30;
                } else if ( menu_number == 1 ) {//menu 0
                      min_sel=0;max_sel=1; 
                } else if ( menu_number == 2 ){
                    encoderPos=max_speed/10;
                    min_sel=10;max_sel=100;
                } else if ( menu_number == 3 ) {
                    encoderPos=f_acc_b;
                    min_sel=10;max_sel=40;
                }
                break;
                case 6:
                    if ( menu_number == 1 ){
                        if (filter) encoderPos=1;
                        else        encoderPos=0;
                        min_sel=0;max_sel=1;
                    } else if ( menu_number == 2 ){
                        encoderPos=max_accel/10;
                        min_sel=10;max_sel=100;
                    }
                    break;
                case 7:
                    if ( menu_number == 2 ){
                        encoderPos=min_pidk/10;
                        min_sel=10;max_sel=99;
                    }
                    break;
                case 8:
                    if ( menu_number == 2 ){
                        encoderPos=byte(min_pidi/2);
                        min_sel=2;max_sel=200;
                    }                
                break;
                case 9:
                    if ( menu_number == 2 ){
                        encoderPos=byte(min_pidd/2);
                        min_sel=2;max_sel=200;
                    }
                    break;
                case 10:
                    if ( menu_number == 2 ){
                        encoderPos=max_pidk/10;
                        min_sel=2;max_sel=200;
                    }                
                break;
                case 11:
                    if ( menu_number == 2 ){
                        encoderPos=byte(max_pidi/2);
                        min_sel=2;max_sel=200;
                    }
                    break;
                case 12:
                    if ( menu_number == 2 ){
                        encoderPos=byte(max_pidd/2);
                        min_sel=2;max_sel=200;
                    }                
                break;
          }
    
        }//if switch select
          show_changed_options = true;
          update_options = true;
  }//If selection
  
  if (oldEncPos != encoderPos) {
    show_changed_options = true;

    if (!isitem_sel) { //Selecting position
          curr_sel=encoderPos;
          encoderPos=oldEncPos=curr_sel;

          if ( menu_number == 0 ) {
              if (encoderPos > 4) {
                  encoderPos=1;
                  menu_number+=1;
              } else if ( encoderPos < 1) {
                  encoderPos=5;
                  menu_number=3;
              }
          } else if (menu_number == 1) {
               if (encoderPos > 6) {
                  encoderPos=1;
                  menu_number=2;         
               } else if ( encoderPos < 1) {
                  encoderPos=4;
                  menu_number=0;
              }
          } else if (menu_number == 2) {
             if (curr_sel > 12) {
              encoderPos=1;
              menu_number=3; 
             } else if ( encoderPos < 1) {
                  encoderPos=6;
                  menu_number=1;
              }
          } else if (menu_number == 3) {
             if (curr_sel > 5) {
              encoderPos=1;
              menu_number=0; 
             } else if ( encoderPos < 1) {
                  encoderPos=12;
                  menu_number=2;
              }
          }
          clear_all_display=true;
          display_lcd();
    } else {//inside a particular selection
     
      //if (curr_sel != 0) {
        if ( encoderPos > max_sel ) {
           encoderPos=oldEncPos=max_sel; 
        } else if ( encoderPos < min_sel ) {
            encoderPos=oldEncPos=min_sel;
          } else {
        
        oldEncPos = encoderPos;
        cant_opciones_mod=1;
        seleccion_mod[0] = encoderPos;
        
            switch (curr_sel) {
              case 1:
                if ( menu_number == 0 )     {vent_mode           = encoderPos;opciones_mod[0]=MENU_OPT_MOD;}
                else if (menu_number == 1)  {alarm_max_pressure  = encoderPos;opciones_mod[0]=MENU_OPT_PIP_AL;}
                else if (menu_number == 2)  {min_cd  = int(encoderPos);}
                else if (menu_number == 3)  {dpip_b = encoderPos; dpip  = float(encoderPos)/10.;}
                break;
              case 2:
                if ( menu_number == 0 )       {options.respiratoryRate = encoderPos; opciones_mod[0]=MENU_OPT_BPM;}
                else  if (menu_number == 1)   {alarm_peep_pressure     = encoderPos; opciones_mod[0]=MENU_OPT_PEEP_AL;}
                else  if (menu_number == 2)   min_speed  = int((float)encoderPos*10.);
                else if ( menu_number == 3 ){
                    Serial.print("encoderPos: ");Serial.println(encoderPos);
                    pfmin=encoderPos;
                    pf_min=(float)encoderPos/50.;
                    peep_fac = -(pf_max-pf_min)/15.*last_pressure_min + pf_max;
                }
                break;
              case 3:
                if ( menu_number == 0 ) {options.percInspEsp = encoderPos;opciones_mod[0]=MENU_OPT_IE;}
                else    if (menu_number == 1) alarm_vt=int(10.*(float)encoderPos);
                else    if (menu_number == 2) min_accel  = int((float)encoderPos*10.);
                if ( menu_number == 3 ){
                    pfmax=encoderPos;
                    pf_max=(float)encoderPos/50.;
                    peep_fac = -(pf_max-pf_min)/15.*last_pressure_min + pf_max;
                }
                break;
              case 4:
                if ( menu_number == 0 ) {
                    if (vent_mode==VENTMODE_PCL){
                        options.peakInspiratoryPressure = encoderPos;
                        opciones_mod[0]=MENU_OPT_PIP;
                        Serial.print("pip: ");Serial.println(options.peakInspiratoryPressure);
                        Serial.print("encoderpos: ");Serial.println(encoderPos);
                      } else { //manual
                      options.percVolume = encoderPos;
                      opciones_mod[0]=MENU_OPT_VOL;
                    }
                } else if (menu_number == 1) {
                    p_trim=encoderPos;
                } else if (menu_number == 2) {max_cd  = int(encoderPos);
                } else if (menu_number == 3) {p_acc=encoderPos;}
                    
                break;
              case 5:
                if ( menu_number == 0 ) {
                    //options.peakInspiratoryPressure = encoderPos;
                } else if (menu_number == 1) {
                    autopid=encoderPos;
                } else if (menu_number == 2) {
                    max_speed  = int((float)encoderPos*10.);
                } else if (menu_number == 3) {f_acc_b=encoderPos;f_acc=(float)f_acc_b/10.;}
                break;
              case 6:
                if ( menu_number == 0 )
                  options.peakEspiratoryPressure = encoderPos;
                else if ( menu_number == 1 )  //There is not 6 in menu 1
                    if (encoderPos==1) filter  = true;
                    else                filter=false;
                else if ( menu_number == 2 )  //There is not 6 in menu 1
                    max_accel  = int((float)encoderPos*10.);
                break;
            
            case 7:
                if ( menu_number == 2 ){
                    min_pidk=encoderPos*10;
                }
                break;
            case 8:
                if ( menu_number == 2 ){
                    min_pidi=encoderPos*10;
                }
                break;
            case 9:
                if ( menu_number == 2 ){
                    min_pidd=encoderPos*2;
                }
                break;
            case 10:
                if ( menu_number == 2 ){
                    max_pidk=encoderPos*2;
                }
                break;
            case 11:
                if ( menu_number == 2 ){
                    max_pidi=int(encoderPos)*2;
                    Serial.print("Max pid i:");Serial.println(max_pidi);
                    Serial.print("Encoder pos:");Serial.println(encoderPos);
                }
                break;
            case 12:
                if ( menu_number == 2 ){
                    max_pidd=encoderPos*2;
                }
                break;

            }//switch
            show_changed_options = true;
            update_options=true;
          }//Valid range

    old_curr_sel = curr_sel;
    if (menu_number==2)
      change_pid_params=true;
    }//oldEncPos != encoderPos and valid between range
  }
}

void clear_n_sel(int menu){
    if (menu==0) {  
        lcd_clearxy(0,0);
        lcd_clearxy(0,1);lcd_clearxy(9,0);
        lcd_clearxy(0,2);lcd_clearxy(8,1);
         switch(curr_sel){
              case 1: 
                lcd_selxy(0,0);break;
              case 2: 
                lcd_selxy(0,1);break;
              case 3:
                lcd_selxy(0,2);break;
              case 4: 
                if ( vent_mode==VENTMODE_VCL || vent_mode==VENTMODE_PCL)  lcd_selxy(8,1);//pcl
                else                                                      lcd_selxy(9,0);
//              case 5: 
//                lcd_selxy(8,1);break;
            }
     } else if (menu==1){  
      lcd_clearxy(0,0);
      lcd_clearxy(0,1);lcd_clearxy(12,2);
      lcd_clearxy(0,2);lcd_clearxy(0,3);
      switch(curr_sel){
          case 1: 
            lcd_selxy(0,0);break;//PIP
          case 2: 
            lcd_selxy(0,1);break;//PEEP
          case 3:
            lcd_selxy(10,1);break;
          case 4: 
            lcd_selxy(0,2);break;
          case 5: 
            lcd_selxy(0,3);break;
          case 6: 
            lcd_selxy(12,2);break;
      }
    } else if (menu==2) {  
      lcd_clearxy(0,0);lcd_clearxy(6,0);lcd_clearxy(12,0);
      lcd_clearxy(0,1);lcd_clearxy(6,1);lcd_clearxy(12,1);
      lcd_clearxy(0,2);
      lcd_clearxy(0,3);
      switch(curr_sel){
          case 1: 
            lcd_selxy(0,0);break;//PIP
          case 2: 
            lcd_selxy(6,0);break;//PEEP
          case 3:
            lcd_selxy(12,0);break;
          case 4: 
            lcd_selxy(0,1);break;//PIP
          case 5: 
            lcd_selxy(6,1);break;
          case 6: 
            lcd_selxy(12,1);break;  
          case 7: 
            lcd_selxy(0,2);break;
          case 8: 
            lcd_selxy(6,2);break;  
          case 9: 
            lcd_selxy(12,2);break;              
          case 10: 
            lcd_selxy(0,3);break;  
          case 11: 
            lcd_selxy(6,3);break;              
          case 12: 
            lcd_selxy(12,3);break;  
      }
    }//menu number 
    else if (menu==3) {  
      lcd_clearxy(0,0);lcd_clearxy(6,0);lcd_clearxy(12,0);
      lcd_clearxy(0,1);lcd_clearxy(6,1);lcd_clearxy(12,1);
      lcd_clearxy(0,2);
      lcd_clearxy(0,3);
      switch(curr_sel){
          case 1: 
            lcd_selxy(0,0);break;//PIP
          case 2: 
            lcd_selxy(0,1);break;//PEEP
          case 3:
            lcd_selxy(7,1);break;
          case 4: 
            lcd_selxy(0,2);break;//PIP
          case 5: 
            lcd_selxy(7,2);break;
      }
    }//menu number 
}

void display_lcd ( ) {
    if (clear_all_display)
        lcd.clear();        
  clear_n_sel(menu_number);
  if (menu_number==0) {  
    lcd_clearxy(12,0,4);
    lcd_clearxy(5,1,3); lcd_clearxy(14,1,2);
    lcd_clearxy(5,2,2); lcd_clearxy(13,2,2);
  
    switch (vent_mode){
      case VENTMODE_PCL:
        writeLine(0, "MOD:PCV", 1); 
        writeLine(1, "PIP:" + String(options.peakInspiratoryPressure), 9);
        writeLine(0, "V: -", 10);
      break;    
      case VENTMODE_VCL:
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
  
//    #ifdef DEBUG_UPDATE
//      Serial.print("Presion mostrada: ");Serial.println(pressure_max);
//    #endif
    dtostrf(last_pressure_max, 2, 0, tempstr);
    writeLine(1, String(tempstr), 16);  
    
    #ifdef DEBUG_UPDATE
      Serial.print("Max press conv: ");Serial.println(tempstr);
      Serial.print("Min Max press");  Serial.print(pressure_min);Serial.print(" ");Serial.println(pressure_max);
    #endif
      
    writeLine(2, "PEEP: ", 11);
    dtostrf(last_pressure_min, 2, 0, tempstr);
    writeLine(2, String(tempstr), 16);  
    
    dtostrf((_mllastInsVol + _mllastExsVol)/2.*options.respiratoryRate*0.001, 2, 1, tempstr);
    writeLine(3, "VM:" + String(tempstr), 0);
    
    dtostrf(_timeoutIns*0.001, 1, 1, tempstr);
    writeLine(3, "I:" + String(tempstr), 9); 
    //dtostrf(_timeoutEsp*0.001, 1, 1, tempstr);
    //writeLine(3, "E:" + String(tempstr), 15); 
    
    //dtostrf(ventilation->getCycleTime()*0.001, 1, 1, tempstr);
    writeLine(3, "i:" + String(tempstr), 15); 
   
      
  } else if (menu_number ==1 ) {//OTHER SETTINGS
                        lcd_clearxy(12,0,8);
    lcd_clearxy(8,1,2); lcd_clearxy(16,1,3);
                        lcd_clearxy(15,2,3);
        
    writeLine(0, "PIPAL:" + String(alarm_max_pressure), 1); 
    
    dtostrf(Cdyn*1.01972, 2, 1, tempstr);
    writeLine(0, "CD:" + String(tempstr), 10); 
    
    writeLine(1, "PEEPAL:" + String(alarm_peep_pressure), 1); 
    writeLine(1, "VTAL:" + String(alarm_vt), 11);
    
    dtostrf((float(p_trim-100)), 2, 0, tempstr);
    writeLine(2, "TRIM:" + String(tempstr) + "e-3", 1); 

    writeLine(2, "F:" , 13);
    if (filter)     writeLine(2, "ON", 15);
    else            writeLine(2, "OFF", 15);    
         
    writeLine(3, "AUTO: ", 1);
    if (autopid)    writeLine(3, "ON", 6);
    else            writeLine(3, "OFF", 6);    

    writeLine(3, "C:", 10);
    writeLine(3, String(last_cycle), 12);
  } else if (menu_number ==2 ){//PID

    for (int i=0;i<3;i++){
        lcd_clearxy(3,i,3); lcd_clearxy(9,i,3);lcd_clearxy(15,i,3);
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
    
    
  } else if (menu_number ==3 ){//PID Config 2
    lcd_clearxy(3,0,2); lcd_clearxy(9,0,3);lcd_clearxy(15,0,3);
    lcd_clearxy(3,1,2); lcd_clearxy(9,1,3);lcd_clearxy(15,1,3);
    lcd_clearxy(3,2,3); 
    lcd_clearxy(3,3,3);
        
    writeLine(0, "dp:" + String(dpip), 1); 
        
    dtostrf(pf_min, 1, 2, tempstr);writeLine(1, "f:"   + String(tempstr), 1); 
    dtostrf(pf_max, 1, 2, tempstr);writeLine(1, "F:"   + String(tempstr), 8); 

    writeLine(2, "pa:" + String(p_acc), 1); 
    dtostrf(f_acc, 1, 2, tempstr);writeLine(2, "fa:"   + String(tempstr), 8); 
    
  }//menu_number
  
  clear_all_display=false;

}
//////////////////////////////////////
/////// MENU INICIAL /////////////////
//////////////////////////////////////

Menu_inic::Menu_inic(byte *mode, byte *bpm, byte *i_e){
		
		
		bool fin=false;
		menu_number=0;
		
		while (!fin){
			
			
			
			
		}


}


void Menu_inic::check_encoder ( ) {
  check_updn_button(PIN_MENU_DN,&encoderPos,true);   //Increment
  check_updn_button(PIN_MENU_UP,&encoderPos,false);  //Decrement
  pressed=0;  //0 nothing , 1 enter, 2 bck

    if (digitalRead(PIN_MENU_EN) == LOW)  //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6) v
    if (time - lastButtonPress > 150) {
      pressed = 1;
      isitem_sel=true; 
      lastButtonPress = time;
    }// if time > last button press

    check_bck_state();

    if (pressed > 0) { //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6) 
      if (!isitem_sel) {
        curr_sel=oldEncPos=encoderPos=old_curr_sel;
      }
                  
      if (isitem_sel) {
          switch (curr_sel){
            case 1: 
             if ( menu_number == 0 ) {
                    min_sel=1;max_sel=2;
                    encoderPos=oldEncPos=vent_mode;
                } 
            break;
            case 2: 
                if ( menu_number == 0 ) {
                    encoderPos=oldEncPos=options.respiratoryRate;
                    min_sel=DEFAULT_MIN_RPM;max_sel=DEFAULT_MAX_RPM;
                    } 
            break;
            case 3:
              if ( menu_number == 0 ) {
                  encoderPos=oldEncPos=options.percInspEsp;
                  min_sel=1;max_sel=3;   
              } else if ( menu_number == 1 ) {
                    encoderPos=byte(0.1*float(alarm_vt));
                    min_sel=10;max_sel=50;//vt
              } else if ( menu_number == 2 ) {
                    encoderPos=min_accel/10; 
                    min_sel=10;max_sel=100; 
              } else if ( menu_number == 3 ){
                        encoderPos=pfmax=50.*pf_max;
                        min_sel=0;max_sel=99;
                    }  
            break;
            case 4: 
                if ( menu_number == 0 ) {
                    if ( vent_mode==VENTMODE_PCL){
 //                     encoderPos=oldEncPos=options.tidalVolume;
 //                     min_sel=200;max_sel=800;
                        encoderPos=oldEncPos=options.peakInspiratoryPressure;
                        min_sel=15;max_sel=30;
                        Serial.print("pip: ");Serial.println(options.peakInspiratoryPressure);
                        Serial.print("encoderpos: ");Serial.println(encoderPos);
                    } else {//Manual
                      encoderPos=oldEncPos=options.percVolume;
                      min_sel=40;max_sel=100;            
                    } 
                }
                break;
			}
    
        }//if switch select
          show_changed_options = true;
          update_options = true;
  }//If selection
  
  if (oldEncPos != encoderPos) {
    show_changed_options = true;

    if (!isitem_sel) { //Selecting position
          curr_sel=encoderPos;
          encoderPos=oldEncPos=curr_sel;

          if ( menu_number == 0 ) {
              if (encoderPos > 4) {
                  encoderPos=1;
                  menu_number+=1;
              } else if ( encoderPos < 1) {
                  encoderPos=5;
                  menu_number=3;
              }
          } 
          clear_all_display=true;
          display_lcd();
    } else {//inside a particular selection
     
      //if (curr_sel != 0) {
        if ( encoderPos > max_sel ) {
           encoderPos=oldEncPos=max_sel; 
        } else if ( encoderPos < min_sel ) {
            encoderPos=oldEncPos=min_sel;
          } else {
        
        oldEncPos = encoderPos;
        
            switch (curr_sel) {
              case 1:
                if ( menu_number == 0 )     {vent_mode           = encoderPos;}
                break;
              case 2:
                if ( menu_number == 0 )       {}
                break;
              case 3:
                if ( menu_number == 0 ) {}

                break;
              case 4:
                if ( menu_number == 0 ) {

                }  
                break;

            }//switch
            show_changed_options = true;
            update_options=true;
          }//Valid range

    old_curr_sel = curr_sel;

    }//oldEncPos != encoderPos and valid between range
  }
}

void Menu_inic::clear_n_sel(int menu){
    if (menu==0) {  
        lcd_clearxy(0,0);
        lcd_clearxy(0,1);lcd_clearxy(9,0);
        lcd_clearxy(0,2);lcd_clearxy(8,1);
         switch(curr_sel){
              case 1: 
                lcd_selxy(0,1);break;
              case 2: 
                lcd_selxy(0,2);break;
              case 3:
                lcd_selxy(0,3);break;
              case 4: 
                lcd_selxy(9,3);break;//pcl
            }
     } 
}


void Menu_inic::display_lcd ( ) {
	
  if (clear_all_display)
        lcd.clear();        
  clear_n_sel(menu_number);
  if (menu_number==0) {  
    lcd_clearxy(12,0,4);
    lcd_clearxy(5,1,3); lcd_clearxy(14,1,2);
    lcd_clearxy(5,2,2); lcd_clearxy(13,2,2);
  
     
    writeLine(0, "INGRESE PARAMS", 4);
	writeLine(1, "BPM: " + String(_mod), 1);
    writeLine(2, "BPM: " + String(_bpm), 1);
    writeLine(3, "IE:1:"+ String(_i_e), 1);
    writeLine(3, "FIN:", 10);
   
      
  } 
  
  clear_all_display=false;

}
