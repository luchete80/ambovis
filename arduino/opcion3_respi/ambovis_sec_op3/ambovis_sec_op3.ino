#include "pinout.h"

#include "menu.h"
#include "display.h"
#include "Serial.h"
#include "display.h"
#include <EEPROM.h>

bool init_verror;
byte Cdyn;
bool autopid;
bool filter;
bool sleep_mode;
bool put_to_sleep,wake_up;
unsigned long print_bat_time;
unsigned long _msecTimerCnt=0;
bool change_cycle;
unsigned long tft_draw_time;
//int vt;

byte _back[8] = {
  0b00100,
  0b01000,
  0b11111,
  0b01001,
  0b00101,
  0b00001,
  0b00001,
  0b11111
};


bool wait4read;

int last_vals[7][2];
int xgra[5][2];

Menu *menu;

// FOR ADS
#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads(0x48);
float Voltage = 0.0;

float _mlInsVol = 0;
float _mlExsVol = 0;
int _mllastInsVol, _mllastExsVol;
unsigned long mute_count;


int Compression_perc = 8; //80%

byte alarm_state = 0; //0: No alarm 1: peep 2: pip 3:both

#ifdef LCD_I2C
LiquidCrystal_I2C lcd(0x3F, 20, 4);
#else

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
#endif

//MUTE
boolean last_mute,curr_mute;
unsigned long time_mute;

boolean buzzmuted;
unsigned long timebuzz=0;
bool isbuzzeron=false;


Adafruit_ILI9341 tft=Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

byte vcorr_count;
byte p_trim = 100;
float pressure_p;   //EXTERN!!
float last_pressure_max, last_pressure_min, last_pressure_peep;
float pressure_max,pressure_min,pressure_peep;

byte vent_mode = VENTMODE_PCL; //0

float verrp;
float _flux,    flow_f;;
//#ifdef FILTER_FLUX
float _flux_fil[5];
float _mlInsVol2;
float _flux_sum;
byte flux_count;
//#endif

bool send_data = false;
char tempstr[5], tempstr2[5];
int curr_sel, old_curr_sel;
float p_dpt;

unsigned long lastReadSensor = 0;
unsigned long lastShowSensor = 0;
unsigned long lastSave = 0;
bool display_needs_update = false;
//
//State static lastState;
bool show_changed_options = false; //Only for display
bool update_options = false;

unsigned long time_update_display = 20; //ms
unsigned long last_update_display;

extern float _mlInsVol, _mlExsVol;

unsigned long last_vent_time;
unsigned long time;
byte cycle_pos;
int16_t adc0;

int max_accel,min_accel;
int max_speed, min_speed;
int min_pidk,max_pidk;
int min_pidi,max_pidi;
int min_pidd,max_pidd;
byte pfmin,pfmax;
float pf_min,pf_max;
float peep_fac;

//min_pidk=250;
//max_pidk=1000;
int min_cd,max_cd;
//max_cd=40;  //T MODIFY: READ FROM MEM
//min_cd=10;
  
unsigned long last_cycle;

unsigned int _timeoutIns,_timeoutEsp; //In ms

byte menu_number = 0;
//TODO: READ FROM EEPROM
byte alarm_max_pressure = 35;
byte alarm_peep_pressure = 5;
byte isalarmvt_on;
int alarm_vt = 200;

//MENU
unsigned long lastButtonPress;
float verror, verror_sum;

bool change_pid_params=false;

byte encoderPos = 1; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
byte oldEncPos = 1; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

byte max_sel, min_sel; //According to current selection

void check_buzzer_mute();
void autotrim_flux();
void check_sleep_mode();  //Batt charge only

bool isitem_sel;
byte old_menu_pos=0;
byte old_menu_num=0;
/* TODO: ARE THEY GONNA BE USED ?? */
float dpip,dpip_b;
float f_acc, f_acc_b, p_acc;

VentilationOptions_t options;

float p_dpt0;

int bck_state ;     // current state of the button
int last_bck_state ; // previous state of the button
int startPressed ;    // the moment the button was pressed
int endPressed ;      // the moment the button was released
int holdTime ;        // how long the button was hold
int idleTime ;        // how long the button was idle

byte valsreaded;
byte last_x;
 bool tft_cleaned;
unsigned long time_serial_read;
int cant_enviadas_menu;
void setup() {

    Serial1.begin(115200);
    Serial.begin(250000);
    init_display();
    isitem_sel=false;

    pinMode(PIN_BUZZER, OUTPUT); //Set buzzerPin as output
    pinMode(GREEN_LED,  OUTPUT); //Set buzzerPin as output
    pinMode(BCK_LED,    OUTPUT); //Set buzzerPin as output
    pinMode(YELLOW_LED, OUTPUT); //Set buzzerPin as output
    pinMode(RED_LED, OUTPUT); //Set buzzerPin as output

    digitalWrite(PIN_BUZZER,BUZZER_LOW); //LOW, INVERTED
 
  
    max_cd=40;  //T MODIFY: READ FROM MEM
    min_cd=10;
    min_speed = 250;  // x microsteps
    max_speed = 750;  // x Microsteps, originally 16000 (with 16 ms = 750)
    max_accel = 600;
    min_accel = 200;
    change_pid_params=true; //To calculate at first time
    
    // Parte motor
    pinMode(PIN_MUTE, INPUT_PULLUP);
    pinMode(PIN_POWEROFF, INPUT);
  
    pinMode(PIN_MENU_UP, INPUT_PULLUP);
    pinMode(PIN_MENU_DN, INPUT_PULLUP);
    pinMode(PIN_MENU_EN, INPUT_PULLUP);
    pinMode(PIN_MENU_BCK, INPUT_PULLUP);
    pinMode(PIN_BAT_LEV, INPUT);
    
    // TODO: Añadir aquí la configuarcion inicial desde puerto serie
    options.respiratoryRate = DEFAULT_RPM;
    options.percInspEsp = 2; //1:1 to 1:4, is denom
    //options.peakInspiratoryPressure = DEFAULT_PEAK_INSPIRATORY_PRESSURE;
    options.peakInspiratoryPressure = 20.;
    options.peakEspiratoryPressure = DEFAULT_PEAK_ESPIRATORY_PRESSURE;
    options.triggerThreshold = DEFAULT_TRIGGER_THRESHOLD;
    options.hasTrigger = false;
    options.tidalVolume = 300;
    options.percVolume = 100; //1 to 10
  
  
    delay(100);
  
    writeLine(1, "RespirAR FIUBA", 4);
    writeLine(2, "v1.1.1", 8);
    
    p_dpt0 = 0;
    ads.begin();
    verror = 0;
    float verrp = 0.;

  Serial.print("dp  error : "); Serial.println(-verror / (5.*0.09));
  p_dpt0 = 0.20;


  //sensors -> readPressure();
  lcd.createChar(0, _back);//Custom chars
  display_lcd();

  //ENCODER
  curr_sel = old_curr_sel = 1; //COMPRESSION

  lastReadSensor =   lastShowSensor = millis();
  last_update_display = millis();
  int eeAddress=0;

    tft.begin();
    tft.fillScreen(ILI9341_BLACK);


    digitalWrite(BCK_LED,LOW);
    buzzmuted=false;
    last_mute=HIGH;
    mute_count=0;

    //lcd.clear();

    pf_min=(float)pfmin/50.;
    pf_max=(float)pfmax/50.;
    peep_fac = -(pf_max-pf_min)/15.*last_pressure_min + pf_max;

    sleep_mode=false;
    put_to_sleep=false;
    wake_up=false;

    valsreaded=last_x=0;
    tft_cleaned=false;

    xgra[P_][1]=0;
    xgra[FLUX_][1]=0;

    time_serial_read=millis();
    cant_opciones_mod=0;
    wait4read=false;

    change_cycle=false;
    cant_enviadas_menu=0;
}


bool update_display = false;
byte pos;

byte recvchars;

void loop() {

//
//
//  if (!sleep_mode){
//    if (wake_up){
//      lcd.clear();
//      init_display();
//      Serial.println("wake up");
//      display_lcd();
//      tft.fillScreen(ILI9341_BLACK);
//      wake_up=false;
//      }
//
      time = millis();
      check_encoder();

            

      //if (time > time_serial_read + SERIAL_READ){
      if (!wait4read){
          recvchars=recvWithEndMarker();
          showNewData();
          Serial.print("chars: ");Serial.println(receivedChars);
          //parseData();
          cycle_pos=integerFromPC[TIME_];
          if (integerFromPC[TIME_] == 128){
              _mllastInsVol= integerFromPC[1];
              _mllastExsVol= integerFromPC[2];            
          } else {
              if (integerFromPC[P_]> 0 ){
                  if ( integerFromPC[P_] > pressure_max){
                      pressure_max = (float)integerFromPC[P_];
                  } else {
                      if ( integerFromPC[P_] < pressure_min){
                          pressure_min = (float)integerFromPC[P_];
                          Serial.print("pmin");Serial.println(pressure_min);
                      }
                    }
                  wait4read=true;
              }
          }
          //Serial.print("cyclepos: ");Serial.println(integerFromPC[P_]);
          time_serial_read=time;
      }
      //}

      //Si leo la ultima informacion al final del ciclo

      if ( time > lastShowSensor + TIME_SHOW ) {

          lastShowSensor=time; 
          tft_draw_time=millis();
          tft_draw();
          Serial.print("dra time: ");Serial.println(millis()-tft_draw_time);
          wait4read=false;

      }      
      //Serial.print("char length: ");Serial.println(recvchars);
//    
      
//      check_buzzer_mute();
//      //Serial.print("Carga: ");Serial.println(analogRead(PIN_BAT_LEV));
//


      if (cycle_pos > 100) {
          //#ifdef DEBUG_UPDATE 
          Serial.print("Sending by serial");
          //#endif
          if (cant_opciones_mod>0 && cant_enviadas_menu < 3 ){
              cant_opciones_mod=0;
              Serial1.print(opciones_mod[0]);Serial1.print(",");
              Serial1.println(seleccion_mod[0]);
              cant_enviadas_menu++;
          }//if hay opciones modificadas

          //Recibo data de flujo y tiempos
          
          last_pressure_max = pressure_max;
          pressure_max = 0;
          last_pressure_min=pressure_min;
          pressure_min = 100;
          Serial.println("FIN DE CICLO");
      }

      if (cycle_pos < 5 && !change_cycle){
          cant_enviadas_menu=0;
          //vt=(_mllastInsVol + _mllastInsVol)/2;
          //if (vt<alarm_vt)  isalarmvt_on=1;
          //else              isalarmvt_on=0;
          if ( last_pressure_max > alarm_max_pressure + 1 ) {
            if ( last_pressure_min < alarm_peep_pressure - 1) {
                if (!isalarmvt_on)  alarm_state = 3;
                else                alarm_state = 13;
              } else {
                if (!isalarmvt_on)  alarm_state = 2;
                else                alarm_state = 12;
              }
            } else {
              if ( last_pressure_min < alarm_peep_pressure - 1 ) {
                if (!isalarmvt_on) alarm_state = 1;
                else               alarm_state = 11;
              } else {
                if (!isalarmvt_on)  alarm_state = 0;
                else                alarm_state = 10;
              }
          }
          change_cycle=true;
      }

//        if ( ventilation -> getCycleNum () != last_cycle ) {
//        vt=(_mllastInsVol + _mllastInsVol)/2;
//        if (vt<alarm_vt)  isalarmvt_on=1;
//        else              isalarmvt_on=0;
//        if ( last_pressure_max > alarm_max_pressure + 1 ) {
//        if ( last_pressure_min < alarm_peep_pressure - 1) {
//            if (!isalarmvt_on)  alarm_state = 3;
//            else                alarm_state = 13;
//          } else {
//            if (!isalarmvt_on)  alarm_state = 2;
//            else                alarm_state = 12;
//          }
//        } else {
//          if ( last_pressure_min < alarm_peep_pressure - 1 ) {
//            if (!isalarmvt_on) alarm_state = 1;
//            else               alarm_state = 11;
//          } else {
//            if (!isalarmvt_on)  alarm_state = 0;
//            else                alarm_state = 10;
//          }
//        }

        display_lcd();
        update_display = true;
        last_update_display = time;
//    
//     
      if (display_needs_update) {
    
        display_lcd();
        display_needs_update = false;
      }
//    
//      if ( update_options ) {
//        update_options = false;
//        //show_changed_options=true;
//      }//
//
//    
//      //HERE changed_options flag is not updating until cycle hcanges
      if (show_changed_options && ((millis() - last_update_display) > time_update_display) ) {
        display_lcd();  //WITHOUT CLEAR!
        last_update_display = millis();
        show_changed_options = false;
      }
//    
        if (alarm_state > 0) {
    
              if (!buzzmuted) {
                  if (time > timebuzz + TIME_BUZZER) {
                      timebuzz=time;
                      isbuzzeron=!isbuzzeron;
                      if (isbuzzeron){
                          //digitalWrite(PIN_BUZZER,BUZZER_LOW);
                      }   
                      else {
                          //digitalWrite(PIN_BUZZER,!BUZZER_LOW);
                      }
                  }
              } else {  //buzz muted
                  //digitalWrite(PIN_BUZZER,!BUZZER_LOW);
              }
        } else {//state > 0
          //digitalWrite(PIN_BUZZER,!BUZZER_LOW);
          isbuzzeron=true;        //Inverted logic
        }

  //! sleep_mode
//  } else { 
//      if (put_to_sleep){
//          tft.fillScreen(ILI9341_BLACK);
//          digitalWrite(PIN_LCD_EN,HIGH);
//          put_to_sleep=false;  
//          print_bat_time=time;
//          print_bat();
//          //digitalWrite(PIN_BUZZER,!BUZZER_LOW); //Buzzer inverted
//          lcd.clear();
//      }
//      if (time > print_bat_time + 5000){
//        print_bat();
//        print_bat_time=time;
//      }
//      time = millis();
//      check_bck_state();
//  }

}//LOOP


int findClosest(float arr[], int n, float target) {
  int i = 0, j = n - 1, mid = 0;
  while ( j - i > 1 ) {
    mid = (i + j) / 2;
    if (target < arr[mid]) {
      j = mid;
    } else {       // If target is greater than mid
      i = mid;
    }
    //Serial.print("i,j: ");Serial.print(i);Serial.print(",");Serial.println(j);
  }
  return i;
}

boolean debounce(boolean last, int pin) {
    boolean current = digitalRead(pin);
    if (last != current) {
        delay(50);
        current = digitalRead(pin);
    }
    return current;
}

void check_buzzer_mute() {
    curr_mute = debounce ( last_mute, PIN_MUTE );         //Debounce for Up button
    if (last_mute== HIGH && curr_mute == LOW && !buzzmuted){
        mute_count=time;
        buzzmuted=true;
    }
    last_mute = curr_mute;
    if(buzzmuted) {
        if (time > mute_count  + TIME_MUTE)  //each count is every 500 ms
        buzzmuted=false;
    }
}


void autotrim_flux(){
  
  }

  
