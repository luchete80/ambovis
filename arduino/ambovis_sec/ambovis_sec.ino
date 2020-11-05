#include "pinout.h"

#include "menu.h"
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

// FOR ADS
#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads(0x48);
float Voltage = 0.0;
int vt;
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
extern byte stepper_time = 50;
unsigned long last_stepper_time;
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

void setup() {
  
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
//  for (int i = 0; i < 10; i++) {
//    adc0 = ads.readADC_SingleEnded(0);
//    Voltage = (adc0 * 0.1875) * 0.001; //VOLT!
//    Serial.print("Voltage dp: "); Serial.println(Voltage, 3);
//    verror += ( Voltage - 5.0 * 0.04 ); //IF VOUT IS: Vo=VS(0.09*P0.04) +/- ERR
//    //vo=vs(0.09 dp +0.04)+/-verr
//    p_dpt0 += 0.5 * (( Voltage /* 5.0/V_SUPPLY_HONEY */ - 0.1 * 4.8/* - corr_fs */) / (0.8 * 4.8) * DEFAULT_PSI_TO_CM_H20 * 2. - DEFAULT_PSI_TO_CM_H20);
//    verrp += (analogRead(A0) * 5. / 1024. - 5.*0.04);
//    Serial.print("Voltage p: "); Serial.println(analogRead(A0));
//    delay(10);
//  }
//
//  verror /= 10.; //
//  verrp /= 10.;
//  p_dpt0 /= 10.0;
//  Serial.print("dp (Flux) MPX Volt (mV) at p0: "); Serial.println(verror * 1000, 3);
//  Serial.print("pressure  MPX Volt (mV) at p0: "); Serial.println(verrp * 1000, 3);

  Serial.print("dp  error : "); Serial.println(-verror / (5.*0.09));
  p_dpt0 = 0.20;


  //sensors -> readPressure();
  lcd.createChar(0, _back);//Custom chars
  display_lcd();

  //ENCODER
  curr_sel = old_curr_sel = 1; //COMPRESSION

  lastReadSensor =   lastShowSensor = millis();
  last_update_display = millis();

#ifdef DEBUG_UPDATE

#endif

  //STEPPER
  last_stepper_time = millis();
  last_vent_time = millis();

  //Serial.print(",0,50");

  
  //Timer2.setPeriod(500000);
  //Timer2.attachInterrupt(timer2Isr);

#ifdef DEBUG_UPDATE
  Serial.print("Honey Volt at p0: "); Serial.println(analogRead(A0) / 1023.);
#endif
  int eeAddress=0;

 
  Serial.print("Maxcd: ");Serial.println(max_cd);
        
  Serial.print("LAST CYCLE: "); Serial.println(last_cycle);


    tft.begin();
    tft.fillScreen(ILI9341_BLACK);


    digitalWrite(BCK_LED,LOW);
    buzzmuted=false;
    last_mute=HIGH;
    mute_count=0;

    lcd.clear();

    pf_min=(float)pfmin/50.;
    pf_max=(float)pfmax/50.;
    peep_fac = -(pf_max-pf_min)/15.*last_pressure_min + pf_max;

    sleep_mode=false;
    put_to_sleep=false;
    wake_up=false;
}


bool update_display = false;
byte pos;

void loop() {



  if (!sleep_mode){
    if (wake_up){
      lcd.clear();
      init_display();
      display_lcd();
      tft.fillScreen(ILI9341_BLACK);
      wake_up=false;
      }

      check_encoder();
    
      time = millis();
      check_buzzer_mute();
      //Serial.print("Carga: ");Serial.println(analogRead(PIN_BAT_LEV));

      if ( time > lastShowSensor + TIME_SHOW ) {
    
          lastShowSensor=time; 
          //Serial.println(time);
//           Serial.print(int(cycle_pos));Serial.print(",");
//    //	     Serial.println(int(pressure_p));//Serial.print(",");
//    //     //Serial.println(analogRead(A0));
//    //	     #ifdef FILTER_FLUX
//           Serial.print(Voltage,5);Serial.print(",");
//           Serial.print(verror,3);Serial.print(",");
//           Serial.print(p_dpt,5);Serial.print(",");
//    //       Serial.print(_mlInsVol - _mlExsVol);Serial.print(",");
//           Serial.println(flow_f,2);
           //Serial.println(_flux,2);
           
    //       #else
    //       Serial.print(int(_flux));Serial.print(",");
    //       #endif      
    //      Serial.println(int(alarm_state));
          //Serial.print(",");
           //Serial.println(int(_mlInsVol-_mlExsVol));
    //      
          //Serial.print(",");Serial.println(int(alarm_state));     
    //      #ifdef FILTER_FLUX 
    //      Serial.print(Voltage*1000);Serial.print(",");Serial.print(p_dpt);Serial.print(",");Serial.println(_flux);/*Serial.print(",");/*Serial.print(",");Serial.println(_flux_sum/5.);*/
    //      #endif
          //Serial.print(int(_mlInsVol));Serial.print(",");Serial.println(int(_mlExsVol));
          tft_draw();
    
      }
    
    

    
        display_lcd();
        update_display = true;
        last_update_display = time;
    
     
      if (display_needs_update) {
    
        display_lcd();
        display_needs_update = false;
      }
    
      if ( update_options ) {
        update_options = false;
        //show_changed_options=true;
      }//

    
      //HERE changed_options flag is not updating until cycle hcanges
      if (show_changed_options && ((time - last_update_display) > time_update_display) ) {
        display_lcd();  //WITHOUT CLEAR!
        last_update_display = time;
        show_changed_options = false;
      }
    
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
  } else { 
      if (put_to_sleep){
          tft.fillScreen(ILI9341_BLACK);
          digitalWrite(PIN_LCD_EN,HIGH);
          put_to_sleep=false;  
          print_bat_time=time;
          print_bat();
          //digitalWrite(PIN_BUZZER,!BUZZER_LOW); //Buzzer inverted
          lcd.clear();
      }
      if (time > print_bat_time + 5000){
        print_bat();
        print_bat_time=time;
      }
      time = millis();
      check_bck_state();
  }

  //stepper -> processMovement();
}//LOOP

void update_error() {
  //UPDATING VERROR
  if (cycle_pos > 100) {
    if (vcorr_count < 20) {
      vcorr_count += 1.;
      verror_sum += ( Voltage - 0.2 ); //-5*0.04
      verror_sum +=p_dpt; //Si el error es de presion
      //verror+=Voltage;
      init_verror = true;
    }
    //Serial.print("Verror (mV) and count: ");Serial.print(verror_sum*1000);Serial.print(",  ");Serial.println(vcorr_count);
  }
  if (cycle_pos < 5 && init_verror) {
    verror = verror_sum / ((float)vcorr_count + 1.);
    //Serial.print("Verror (mV) and count: ");Serial.print(verror*1000);Serial.print(",  ");Serial.println(vcorr_count);
    //Serial.print("Verror (mV) and count: ");Serial.println(verror*1000);
    verror_sum = 0.;
    vcorr_count = 0;
    init_verror = false;
  }
}


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
