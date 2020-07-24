#include <arduino.h>
#include "pinout.h"
#include "menu.h"
#include "MechVentilation.h"  //options

static bool clear_all_display;

static byte max_pidk_byte,min_pidk_byte;

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

void check_encoder ( ) {
  
  byte btnState = digitalRead(PIN_ENC_SW);
  if (btnState == LOW) { //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6) 
    if (millis() - lastButtonPress > 150) {
      Serial.println("Boton presionado");
      isitem_sel=!isitem_sel; 
      if (!isitem_sel) {
          curr_sel=oldEncPos=encoderPos=old_curr_sel;
          Serial.print("curr sel: ");Serial.println(curr_sel);
          Serial.print("old curr sel: ");Serial.println(old_curr_sel);
      }
      lastButtonPress = millis();
    }// if time > last button press

      if (vent_mode==VENTMODE_PCL && curr_sel==4 && menu_number == 0) curr_sel++; //Not selecting pip in VCL 
            
      if (isitem_sel) {
          switch (curr_sel){
            case 1: 
             if ( menu_number == 0 ) {
                    min_sel=1;max_sel=2;
                    encoderPos=oldEncPos=vent_mode;
                } else if ( menu_number == 1 ) {
                    min_sel=20;max_sel=50;
                    encoderPos=oldEncPos=alarm_max_pressure;            
             } else {
                encoderPos=min_cd;
                min_sel=0;max_sel=max_cd;
             }
            break;
            case 2: 
                if ( menu_number == 0 ) {
                    encoderPos=oldEncPos=options.respiratoryRate;
                    min_sel=DEFAULT_MIN_RPM;max_sel=DEFAULT_MAX_RPM;
                    } else if ( menu_number == 1 ) {
                        min_sel=5;max_sel=30;
                        encoderPos=oldEncPos=alarm_peep_pressure;                          
                    }
                    else{
                        encoderPos=min_speed/10;     
                        min_sel=10;max_sel=100;             
                    }
            break;
            case 3:
              if ( menu_number == 0 ) {
                  encoderPos=oldEncPos=options.percInspEsp;
                  min_sel=2;max_sel=3;   
              } else if ( menu_number == 1 ) {
                    encoderPos=byte(0.1*float(alarm_vt));
                    min_sel=10;max_sel=50;//vt
              }   else {
                    encoderPos=min_accel/10; 
                    min_sel=10;max_sel=100; }
            break;
            case 4: 
                if ( menu_number == 0 ) {
                    if ( vent_mode==VENTMODE_VCL || vent_mode==VENTMODE_PCL){
                      encoderPos=oldEncPos=options.tidalVolume;
                      min_sel=200;max_sel=800;
                    } else {//Manual
                      encoderPos=oldEncPos=options.percVolume;
                      min_sel=40;max_sel=100;            
                    } 
                } else if ( menu_number == 1 ) {//menu 0
                    encoderPos=oldEncPos=p_trim;
                    min_sel=0;max_sel=200;   
                } else {
                    encoderPos=max_cd;
                    min_sel=10;max_sel=80;
                }
                break;
                  case 5: 
                if ( menu_number == 0 ) {
                    encoderPos=oldEncPos=options.peakInspiratoryPressure;
                    min_sel=15;max_sel=30;
                } else if ( menu_number == 1 ) {//menu 0
                      min_sel=0;max_sel=1; 
                } else {
                    encoderPos=max_speed/10;
                    min_sel=10;max_sel=100;
                }
                break;
                case 6:
                    if ( menu_number == 1 ){
                        encoderPos=filter;
                        min_sel=0;max_sel=1;
                    } else if ( menu_number == 2 ){
                        encoderPos=max_accel/10;
                        min_sel=10;max_sel=100;
                    }
                    break;
                case 7:
                    if ( menu_number == 2 ){
                        encoderPos=min_pidk/10;
                        min_sel=10;max_sel=100;
                    }
                    break;
                case 8:
                    if ( menu_number == 2 ){
                        encoderPos=max_pidk/10;
                        min_sel=10;max_sel=100;
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
          Serial.print("curr sel: ");Serial.println(curr_sel);
          Serial.print("Encoder pos: ");Serial.println(encoderPos);
          if ( menu_number == 0 ) {
              if (encoderPos > 5) {
                  encoderPos=1;
                  menu_number+=1;
              } else if ( encoderPos < 1) {
                  encoderPos=8;
                  menu_number=2;
              }
          } else if (menu_number == 1) {
               if (encoderPos > 6) {
                  encoderPos=1;
                  menu_number=2;         
               } else if ( encoderPos < 1) {
                  encoderPos=5;
                  menu_number=0;
              }
          }
            else if (menu_number == 2) {
             if (curr_sel > 8) {
              encoderPos=1;
              menu_number=0; 
             } else if ( encoderPos < 1) {
                  encoderPos=5;
                  menu_number=1;
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
        //switch (menu_number) {
            switch (curr_sel) {
              case 1:
                if ( menu_number == 0 )     vent_mode           = encoderPos;
                else if (menu_number == 1)  alarm_max_pressure  = encoderPos;
                else                        {min_cd  = int(encoderPos);Serial.print("Mincd: ");Serial.println(min_cd);}
                break;
              case 2:
                if ( menu_number == 0 )       options.respiratoryRate = encoderPos;
                else  if (menu_number == 1)   alarm_peep_pressure     = encoderPos;
                else                          min_speed  = int((float)encoderPos*10.);
                break;
              case 3:
                if ( menu_number == 0 ) options.percInspEsp=encoderPos;
                else    if (menu_number == 1) alarm_vt=int(10.*(float)encoderPos);
                else                          min_accel  = int((float)encoderPos*10.);
                //pressure_max = 0;
                break;
              case 4:
                if ( menu_number == 0 ) {
                    if ( vent_mode==VENTMODE_VCL || vent_mode==VENTMODE_PCL){
                      options.tidalVolume = encoderPos;
                      #ifdef DEBUG_UPDATE
                      Serial.print("tidal ");Serial.print(options.tidalVolume);Serial.print("encoder pos");Serial.print(encoderPos);
                      #endif
                      } else { //manual
                      options.percVolume =encoderPos;
                     // Serial.print("Encoder pos: ");Serial.println(encoderPos);
                     // Serial.print("Perc vol: ");Serial.println(options.percVolume);
                    }
                } else if (menu_number == 1) {
                    p_trim=encoderPos;
                } else {max_cd  = int(encoderPos);Serial.print("Maxcd: ");Serial.println(max_cd);}
                    
                break;
              case 5:
                if ( menu_number == 0 ) {
                    options.peakInspiratoryPressure = encoderPos;
                } else if (menu_number == 1) {
                    autopid=encoderPos;
                } else {
                    max_speed  = int((float)encoderPos*10.);
                }
                break;
              case 6:
                if ( menu_number == 0 )
                  options.peakEspiratoryPressure = encoderPos;
                else if ( menu_number == 1 )  //There is not 6 in menu 1
                    filter  = encoderPos;
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
                    max_pidk=encoderPos*10;
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
                lcd_selxy(9,0);break;
              case 5: 
                lcd_selxy(8,1);break;
            }
     } else if (menu==1){  
      lcd_clearxy(0,0);
      lcd_clearxy(0,1);lcd_clearxy(10,2);
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
            lcd_selxy(0,3);break;  
      }
    }//menu number 


}

void display_lcd ( ) {
    if (clear_all_display)
        lcd.clear();        
  clear_n_sel(menu_number);
  if (menu_number==0) {  
    lcd_clearxy(5,1,3); lcd_clearxy(12,0,4);
    lcd_clearxy(5,2,2); lcd_clearxy(14,1,2);
                        lcd_clearxy(13,2,2);
  
    switch (vent_mode){
      case VENTMODE_VCL:
        writeLine(0, "MOD:VCV", 1); writeLine(0, "V:" + String(options.tidalVolume), 10);    
        writeLine(1, "PIP : ", 10);
      break;
      case VENTMODE_PCL:
        writeLine(0, "MOD:PCV", 1); 
        writeLine(1, "PIP:" + String(options.peakInspiratoryPressure), 9);
        writeLine(0, "V: -", 10);
      break;    
      case VENTMODE_MAN:
        writeLine(0, "MOD:VCV", 1); 
        writeLine(0, "V:" + String(options.percVolume)+"%", 10);    
        writeLine(1, "PIP : ", 10);
      break;
    }
     
      
    writeLine(1, "BPM:" + String(options.respiratoryRate), 1);
    writeLine(2, "IE:1:", 1);
  
    dtostrf((_mllastInsVol+_mllastExsVol)/2, 4, 0, tempstr);
    writeLine(0, String(tempstr), 16);
  
    writeLine(2, String(options.percInspEsp), 6);
  
    #ifdef DEBUG_UPDATE
      Serial.print("Presion mostrada: ");Serial.println(pressure_max);
    #endif
    dtostrf(last_pressure_max, 2, 0, tempstr);
    writeLine(1, String(tempstr), 16);  
    
    #ifdef DEBUG_UPDATE
      Serial.print("Max press conv: ");Serial.println(tempstr);
      Serial.print("Min Max press");  Serial.print(pressure_min);Serial.print(" ");Serial.println(pressure_max);
    #endif
      
    writeLine(2, "PEEP: ", 11);
    dtostrf(last_pressure_min, 2, 0, tempstr);
    writeLine(2, String(tempstr), 16);  
    
    dtostrf((_mllastInsVol + _mllastExsVol)*30./_timeoutIns, 2, 1, tempstr);
    writeLine(3, "V:" + String(tempstr), 0); 
    
    dtostrf(_timeoutIns*0.001, 1, 1, tempstr);
    writeLine(3, "I:" + String(tempstr), 9); 
    dtostrf(_timeoutEsp*0.001, 1, 1, tempstr);
    writeLine(3, "E:" + String(tempstr), 15); 
      
  } else if (menu_number ==1 ) {//OTHER SETTINGS
                        lcd_clearxy(12,0,8);
    lcd_clearxy(8,1,2); lcd_clearxy(16,1,3);
                        lcd_clearxy(13,6,3);
        
    writeLine(0, "PIPAL:" + String(alarm_max_pressure), 1); 
    
    dtostrf(Cdyn*1.01972, 2, 1, tempstr);
    writeLine(0, "CD:" + String(tempstr), 10); 
    
    writeLine(1, "PEEPAL:" + String(alarm_peep_pressure), 1); 
    writeLine(1, "VTAL:" + String(alarm_vt), 11);
    
    dtostrf((float(p_trim-100)), 2, 0, tempstr);
    writeLine(2, "TRIM:" + String(tempstr) + "e-3", 1); 

    writeLine(2, " F:" , 12);
    if (filter)     writeLine(2, "ON", 15);
    else            writeLine(2, "OFF", 15);    
         
    writeLine(3, "AUTO: ", 1);
    if (autopid)    writeLine(3, "ON", 6);
    else            writeLine(3, "OFF", 6);    

    writeLine(3, "C:", 10);
    writeLine(3, String(last_cycle), 12);
  } else if (menu_number ==2 ){//PID
    lcd_clearxy(3,0,2); lcd_clearxy(9,0,3);lcd_clearxy(15,0,3);
    lcd_clearxy(3,1,2); lcd_clearxy(9,1,3);lcd_clearxy(15,1,3);
    lcd_clearxy(3,2,3); 
    lcd_clearxy(3,3,3);
        
    writeLine(0, "c:" + String(min_cd), 1); 
    writeLine(1, "C:" + String(max_cd), 1); 
    
    writeLine(0, "v:" + String(min_speed), 7); 
    writeLine(1, "V:" + String(max_speed), 7);

    writeLine(0, "a:" + String(min_accel), 13); 
    writeLine(1, "A:" + String(max_accel), 13);

    writeLine(2, "p:" + String(min_pidk), 1); 
    writeLine(2, "I:" + String(PID_KI), 7); 
    writeLine(2, "D:" + String(PID_KD), 13);
    writeLine(3, "P:" + String(max_pidk), 1); 

  }//menu_number
  
  clear_all_display=false;

}
