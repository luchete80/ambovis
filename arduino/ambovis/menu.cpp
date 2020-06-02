#include <arduino.h>
#include "pinout.h"
#include "menu.h"
#include "MechVentilation.h"  //options

static bool clear_all_display;

void init_display()
{
  #ifdef LCD_I2C
  lcd.begin();  //I2C
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
  lcd.print(">");
}
void check_encoder ( ) {
  byte btnState = digitalRead(PIN_ENC_SW);
  if (btnState == LOW) { //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6) 
    if (millis() - lastButtonPress > 200) {
      curr_sel++; //NOT +=1, is a byte

      //if ((vent_mode==VENTMODE_VCL || vent_mode==VENTMODE_MAN) && curr_sel==5) curr_sel++; //Not selecting pip in VCL
      if (vent_mode==VENTMODE_PCL && curr_sel==4) curr_sel++; //Not selecting pip in VCL 
            
      if ( menu_number == 0 ) {
        if (curr_sel > 5) {
          curr_sel=1;
          menu_number+=1;
          clear_all_display=true;
          display_lcd();
        } 
      } else if (menu_number == 1) {
         if (curr_sel > 3) {
          curr_sel=0;
          menu_number=0; 
          clear_all_display=true;
          display_lcd();         
         }
      }
      
      switch (curr_sel){
        case 1: 
         if ( menu_number == 0 ) {
            min_sel=0;max_sel=2;
            encoderPos=oldEncPos=vent_mode;
            } else if ( menu_number == 1 ) {
            min_sel=20;max_sel=50;
            encoderPos=oldEncPos=alarm_max_pressure;            
         } 
        break;
        case 2: 
            if ( menu_number == 0 ) {
                encoderPos=oldEncPos=options.respiratoryRate;
                min_sel=DEFAULT_MIN_RPM;max_sel=DEFAULT_MAX_RPM;
                } else if ( menu_number == 1 ) {
                    min_sel=5;max_sel=15;
                    encoderPos=oldEncPos=alarm_peep_pressure;                          
                }
        break;
        case 3:
          if ( menu_number == 0 ) {
              encoderPos=oldEncPos=options.percInspEsp;
              min_sel=1;max_sel=4;   
          } else if ( menu_number == 1 ) {
              encoderPos=oldEncPos=p_trim;
              min_sel=0;max_sel=200;  
          }     
        break;
        case 4: 
          if ( vent_mode==VENTMODE_VCL || vent_mode==VENTMODE_PCL){
            encoderPos=oldEncPos=options.tidalVolume;
            min_sel=200;max_sel=800;
          } else {//Manual
            encoderPos=oldEncPos=options.percVolume;
//            Serial.print("Encoder pos: ");Serial.println(encoderPos);
            min_sel=40;max_sel=100;            
          } break;
        case 5: 
          encoderPos=oldEncPos=options.peakInspiratoryPressure;
          min_sel=15;max_sel=25;
        break;
      }

      old_curr_sel = curr_sel;
      show_changed_options = true;
      update_options = true;
    }
    lastButtonPress = millis();

    #ifdef DEBUG_UPDATE
      Serial.print("Modo: ");Serial.println(vent_mode);
    #endif
  }


  if (oldEncPos != encoderPos) {
    show_changed_options = true;
    if (curr_sel != 0) {
      if ( encoderPos > max_sel ) {
         encoderPos=oldEncPos=max_sel; 
      } else if ( encoderPos < min_sel ) {
          encoderPos=oldEncPos=min_sel;
        } else {
       
        oldEncPos = encoderPos;
        switch (curr_sel) {
          case 1:
            if ( menu_number == 0 ) vent_mode           = encoderPos;
            else                    alarm_max_pressure  = encoderPos;
            break;
          case 2:
            if ( menu_number == 0 ) options.respiratoryRate = encoderPos;
            else                    alarm_peep_pressure     = encoderPos;
            break;
          case 3:
            if ( menu_number == 0 ) options.percInspEsp=encoderPos;
            else                    p_trim=encoderPos;
            //pressure_max = 0;
            break;
          case 4:
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
            break;
          case 5:
            options.peakInspiratoryPressure = encoderPos;
            break;
          case 6:
            options.peakEspiratoryPressure = encoderPos;
            break;
        }
        show_changed_options = true;
        update_options=true;
      }//Valid range
  
    }//oldEncPos != encoderPos and valid between range
  }
}


void display_lcd ( ) {
    if (clear_all_display)
        lcd.clear();
    
    lcd_clearxy(0,0);
    lcd_clearxy(0,1);lcd_clearxy(9,1);
    lcd_clearxy(0,2);lcd_clearxy(7,2);
    switch(curr_sel){
          case 1: 
            lcd_selxy(0,0);break;
          case 2: 
            lcd_selxy(0,1);break;
          case 3:
            lcd_selxy(0,2);break;
          case 4: 
            lcd_selxy(9,1);break;
          case 5: 
            lcd_selxy(7,2);break;
      }
      
  if (menu_number==0) {  
    lcd_clearxy(5,1,3); lcd_clearxy(12,1,4);
    lcd_clearxy(5,2,2); lcd_clearxy(13,2,2);
    lcd_clearxy(13,3,2);
  
    switch (vent_mode){
      case VENTMODE_VCL:
        writeLine(0, "MOD:VCV", 1); 
        writeLine(1, "V:" + String(options.tidalVolume), 10);    
        writeLine(2, "PIP : - ", 8);
      break;
      case VENTMODE_PCL:
        writeLine(0, "MOD:PCV", 1); 
        writeLine(2, "PIP :" + String(options.peakInspiratoryPressure), 8);
        writeLine(1, "V: - ", 10);
      break;    
      case VENTMODE_MAN:
        writeLine(0, "MOD:MAN", 1); 
        writeLine(2, "PIP : -", 8);
        writeLine(1, "V:" + String(options.percVolume)+"%", 10);    
      break;
    }
     
      
    writeLine(0, "SET | ME", 11);
    writeLine(1, "BPM:" + String(options.respiratoryRate), 1);
    writeLine(2, "IE:1:", 1);
  
    //dtostrf(ventilation->getInsVol(), 4, 0, tempstr);
    dtostrf(_mllastInsVol, 4, 0, tempstr);
    writeLine(1, String(tempstr), 16);
    //writeLine(1, "---", 16);
  
    writeLine(2, String(options.percInspEsp), 6);
  
    #ifdef DEBUG_UPDATE
      Serial.print("Presion mostrada: ");Serial.println(pressure_max);
    #endif
    dtostrf(last_pressure_max, 2, 1, tempstr);
    writeLine(2, String(tempstr), 16);  
    
    #ifdef DEBUG_UPDATE
      Serial.print("Max press conv: ");Serial.println(tempstr);
      Serial.print("Min Max press");  Serial.print(pressure_min);Serial.print(" ");Serial.println(pressure_max);
    #endif
      
    writeLine(3, "PEEP: -", 11);
    dtostrf(last_pressure_min, 2, 1, tempstr);
    writeLine(3, String(tempstr), 16);  
    
    writeLine(3, "C: -", 1);
    writeLine(3, String(last_cycle), 3);
  
  } else if (menu_number ==1 ){//OTHER SETTINGS
    writeLine(0, "PIP  AL:" + String(alarm_max_pressure), 1); 
    writeLine(1, "PEEP AL:" + String(alarm_peep_pressure), 1); 
        dtostrf((float(p_trim-100)), 2, 0, tempstr);
    writeLine(2, "TRIM:" + String(tempstr) + "e-3", 1); 
        
    #ifdef DEBUG_FLUX
    writeLine(3, "F: " + String(ciclo) + " " + String(ins_prom,0) + "ml " + String(err_sum/((float)ciclo-2.)*100) + "%", 0);
    #endif    
  }//menu_number
      
  clear_all_display=false;

}
