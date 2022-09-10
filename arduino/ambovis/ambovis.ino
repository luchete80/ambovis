#include "src/TimerOne/TimerOne.h"
#include "src/TimerThree/TimerThree.h"
#include "src/AutoPID/AutoPID.h"
#include "src/AccelStepper/AccelStepper.h"

#include "MenuV2.h"
#include "display.h"
#include "pinout.h"
#include "MechVentilation.h"
#include <EEPROM.h>

#define CALIB_CYCLES  5

#ifdef TEMP_TEST
#include <OneWire.h>
#include <DallasTemperature.h>
float temp;
#endif

byte Cdyn;
bool filter;
bool sleep_mode;
bool put_to_sleep, wake_up;
unsigned long print_bat_time;
bool motor_stopped;

bool drawing_cycle = 0;

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;

float Voltage = 0.0;
float _mlInsVol = 0;
float _mlExsVol = 0;
int _mllastInsVol, _mllastExsVol;
unsigned long mute_count;

void read_memory(); //Lee la EEPROM, usa variables externas, quiza deberian englobarse en un vector dinamico todos los offsets
void write_memory();

AccelStepper *stepper;

byte alarm_state = 0; //0: No alarm 1: peep 2: pip 3:both

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

//MUTE
bool last_mute, curr_mute;
bool buzzmuted;
unsigned long timebuzz = 0;
bool isbuzzeron = false;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

byte vcorr_count = 0;
float pressure_p;
float last_pressure_max, last_pressure_min;
float _flux;
float flow_f;
float _flux_fil[5];
float _flux_sum;
char tempstr[5];
float p_dpt;

unsigned long lastReadSensor = 0;
unsigned long lastShowSensor = 0;
unsigned long lastSave = 0;
bool display_needs_update = false;

#ifdef BAT_TEST
unsigned long lastShowBat = 0;
#endif
#ifdef TEMP_TEST
unsigned lastReadTemp = 0;
#endif
bool show_changed_options = false; //Only for display

unsigned long last_update_display;
unsigned long time;
byte cycle_pos;

unsigned long last_cycle;

unsigned int _timeoutIns, _timeoutEsp; //In ms

byte menu_number = 0;
//TODO: READ FROM EEPROM
//byte alarm_max_pressure = 35;
//byte alarm_peep_pressure = 5;
byte isalarmvt_on;
int alarm_vt = 200;

// Menu V2
KeyboardState keyboardState;
MenuState menuState;
MenuV2 menuV2;
VariableParameters varParams;
// END MENU V2

//MENU
float verror, verror_sum, verror_sum_outcycle, vzero = 0.;  //verror sum is intra cycle, verror_sum_outcycle is inter-cycle

//MAX FLUX IS IN INSPIRING POSITIVE (1st quad)
float dp[] = { -3.074176245, -2.547210457, -2.087678384, -1.60054669, -1.216013465, -0.795599072, -0.630753349, -0.504544509, -0.365700986, -0.260808033, -0.176879848, -0.109004974, -0.05874157, -0.051474571, -0.043771552, -0.037061691, -0.029794693, -0.023012161, -0.017561913, -0.01441288, -0.012111664, -0.009325981, -0.007024765, -0.004602432, -0.002664566, 0.00090924, 0.00030358, 0, -0.000242233, -0.000837976, 0.001999305, 0.003937171, 0.006117271, 0.008176254, 0.011688636, 0.014830113, 0.020045692, 0.023566372, 0.028644966, 0.03312327, 0.039664506, 0.047781395, 0.052868293, 0.096530072, 0.151339196, 0.216332764, 0.295221736, 0.377891785, 0.491216024, 0.606462279, 0.877207832, 1.207061607, 1.563385753, 2.030351958, 2.444452733};
byte po_flux[] = {0, 10, 20, 30, 40, 50, 55, 60, 65, 70, 75, 80, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 100, 100, 100, 100, 100, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 120, 125, 130, 135, 140, 145, 150, 160, 170, 180, 190, 200};

void check_buzzer_mute();
void check_sleep_mode();  //Batt charge only

AutoPID * pid;
MechVentilation * ventilation;

//int bck_state ;     // current state of the button
//int last_bck_state ; // previous state of the button
//int startPressed ;    // the moment the button was pressed
//int endPressed ;      // the moment the button was released
//int holdTime ;        // how long the button was hold
//int idleTime ;        // how long the button was idle

float vsupply_0 = 0.;
float vlevel = 0.;

#ifdef TEMP_TEST
OneWire           oneWire(PIN_TEMP);
DallasTemperature sensors(&oneWire);
#endif
  
void setup() {

  pinMode(PIN_STEPPER, OUTPUT);
  digitalWrite(PIN_STEPPER, LOW);
  
  Serial.begin(115200);
  
  analogReference(INTERNAL1V1); // use AREF for reference voltage

  read_memory();

  // Init Menu V2
  menuV2.lcd = &lcd;
  menuV2.keyboardState = keyboardState;
  menuV2.menuState = menuState;

  //init variables
  varParams.vent_mode = VENTMODE_MAN;
  varParams.alarm_max_pressure = 35;
  varParams.respiratoryRate = DEFAULT_RPM;
  varParams.alarm_peep_pressure = 5;
  varParams.percInspEsp = 2;
  varParams.alarm_vt = alarm_vt;
  varParams.peakInspiratoryPressure = 20;
  varParams.percVolume = 100;
  varParams.autopid = 0;
  varParams.filter = filter ? 1 : 0;

  // sensor values
  _mllastInsVol = 0;
  _mllastExsVol = 10;
  // end init menu v2

  initDisplay(menuV2);

  pinMode(TFT_SLEEP, OUTPUT); //Set buzzerPin as output
  digitalWrite(TFT_SLEEP, HIGH); //LOW, INVERTED

  pinMode(LCD_SLEEP, OUTPUT); //Set buzzerPin as output
  digitalWrite(LCD_SLEEP, HIGH); //LOW, INVERTED
  
  pinMode(PIN_BUZZER, OUTPUT); //Set buzzerPin as output
  pinMode(GREEN_LED,  OUTPUT); //Set buzzerPin as output
  pinMode(BCK_LED,    OUTPUT); //Set buzzerPin as output
  pinMode(YELLOW_LED, OUTPUT); //Set buzzerPin as output
  pinMode(RED_LED, OUTPUT); //Set buzzerPin as output

  digitalWrite(PIN_BUZZER, BUZZER_LOW); //LOW, INVERTED

  // PID
  pid = new AutoPID(PID_MIN, PID_MAX, PID_KP, PID_KI, PID_KD);
  // if pressure is more than PID_BANGBANG below or above setpoint,
  // output will be set to min or max respectively
  pid -> setBangBang(PID_BANGBANG);
  // set PID update interval
  pid -> setTimeStep(PID_TS);

  // Parte motor
  pinMode(PIN_MUTE, INPUT_PULLUP);
  pinMode(PIN_POWEROFF, INPUT);
  pinMode(PIN_EN, OUTPUT);

  pinMode(PIN_MENU_UP, INPUT_PULLUP);
  pinMode(PIN_MENU_DN, INPUT_PULLUP);
  pinMode(PIN_MENU_EN, INPUT_PULLUP);
  pinMode(PIN_MENU_BCK, INPUT_PULLUP);
  pinMode(PIN_BAT_LEV, INPUT);
  pinMode(PIN_MPX_LEV, INPUT);

  digitalWrite(PIN_EN, HIGH);

  delay(100);

  if (!digitalRead(PIN_POWEROFF)) {
    digitalWrite(YELLOW_LED, HIGH);
    Serial.println("Poweroff");
  }

  // Habilita el motor
  digitalWrite(PIN_EN, LOW);

  writeLine(menuV2, 1, "RespirAR FIUBA", 4);
  writeLine(menuV2, 2, "v2.1.0", 8);

  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_BLUE);
  tft.setTextSize(4); 
  tft.setCursor(10, 40);     tft.println("RespirAR");   
  tft.setCursor(10, 80);     tft.println("FIUBA");

  ads.begin();
  for (int i = 0; i < 100; i++) {
      vsupply_0 += float(analogRead(PIN_MPX_LEV))/1024.*1.1*VOLTAGE_CONV;
      delay(10);
  }
  vsupply_0 /= 100.;

  ////// ANTES DE CONFIGURAR LA VENTILACION Y CHEQUEAR EL FIN DE CARRERA INICIO LOS MENUES
  setupMenu(menuV2, varParams, millis());

  /////////////////// CALIBRACION /////////////////////////////////////
  bool fin = false;
  lcd.clear();
  writeLine(menuV2, 1, "Desconecte flujo", 0);
  writeLine(menuV2, 2, "y presione ok ", 0);

  delay (100); //Otherwise low enter button readed
  unsigned long lastButtonPress = millis();
  while (!fin) {
      if (digitalRead(PIN_MENU_EN) == LOW) {
          if (millis() - lastButtonPress > 50) {
              fin = true;
              lastButtonPress = millis();
          }// if time > last button press
      }
  }

  digitalWrite(PIN_STEPPER, HIGH);
  delay(1000);

  stepper = new AccelStepper(
    AccelStepper::DRIVER,
    PIN_STEPPER_STEP,
    PIN_STEPPER_DIRECTION);

  ventilation = new MechVentilation(
    stepper,
    pid,
    &varParams
  );

  tft.fillScreen(ILI9341_BLACK);

  // configura la ventilación
  ventilation->updateParameters();
  ventilation->start();
  ventilation->update();

  lcd.clear();
  writeLine(menuV2, 1, "Iniciando...", 0);

#ifdef ACCEL_STEPPER
  stepper->setSpeed(STEPPER_HOMING_SPEED);

  long initial_homing = -1;

  while (digitalRead(PIN_ENDSTOP)) {  // Make the Stepper move CCW until the switch is activated
    stepper->moveTo(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    stepper->run();  // Start moving the stepper
    delay(5);
  }
  stepper->setCurrentPosition(0);  // Set the current position as zero for now
  initial_homing = 1;

  while (!digitalRead(PIN_ENDSTOP)) { // Make the Stepper move CW until the switch is deactivated
    stepper->moveTo(initial_homing);
    stepper->run();
    initial_homing++;
    delay(5);
  }

  long position = stepper->currentPosition();
  stepper->setCurrentPosition(STEPPER_LOWEST_POSITION);
  position = stepper->currentPosition();
#endif

//  printMenu(menuV2, varParams); TODO : maybe this can be removed

  pinMode(PIN_ENC_SW, INPUT_PULLUP);

  lastReadSensor =  lastShowSensor = last_update_display = millis();

  #ifdef BAT_TEST
  lastShowBat = millis();
  #endif

  //STEPPER
  Timer3.initialize(TIME_STEPPER_ISR_MICROS);
  Timer3.attachInterrupt(timer3Isr);

  Timer1.initialize(TIME_BASE_MICROS);  //BEFORE WERE 20...
  Timer1.attachInterrupt(timer1Isr);

//  f_acc = (float)f_acc_b / 10.;
//  dpip = (float)dpip_b / 10.;

  ventilation->setCycleNum(last_cycle);

  digitalWrite(BCK_LED, LOW);
  buzzmuted = false;
  last_mute = HIGH;
  mute_count = 0;

  lcd.clear();

//  pf_min = (float)pfmin / 50.;
//  pf_max = (float)pfmax / 50.;
//  peep_fac = -(pf_max - pf_min) / 15.* varParams.last_pressure_min + pf_max;

  sleep_mode = false;
  put_to_sleep = false;
  wake_up = false;

  //Serial.println("Vcc & Out MPX: " + String(analogRead(PIN_MPX_LEV)) + String(", ") + String(Voltage));
    
  Serial.println("Exiting setup");
  //TODO: CALIBRATION RUN ALSO SHOULD BE HERE

  #ifdef TEMP_TEST
  sensors.begin();
  #endif
}

bool  calibration_run = true;
byte  calib_cycle = 0;
////////////////////////////////////////
////////////// MAIN LOOP ///////////////
////////////////////////////////////////
void loop() {

  //digitalWrite(LCD_SLEEP, HIGH); //LOW, INVERTED
  checkKeyboard(menuV2.keyboardState, time);

  if (menuV2.keyboardState.backHoldTime > 2000) {
      if (!sleep_mode) {
          sleep_mode=true;
          put_to_sleep=true;
      } else {
          sleep_mode=false;
          wake_up=true;
      }
      delay(1000);
      menuV2.keyboardState.backHoldTime = 0;
  }

  if (!sleep_mode) {
    if (wake_up) {
      digitalWrite(PIN_STEPPER, HIGH);
      digitalWrite(TFT_SLEEP, HIGH);
      digitalWrite(LCD_SLEEP, HIGH);
      lcd.clear();
      initDisplay(menuV2);
      printMenu(menuV2, varParams, millis());
      tft.begin();
      tft.fillScreen(ILI9341_BLACK);
      wake_up = false;
      ventilation->forceStart();
    }

    checkEncoder(menuV2, varParams, time);

    time = millis();
    check_buzzer_mute();

    if (time > lastSave + TIME_SAVE) {
      write_memory();

      lastSave = millis();
    }

    if ( time > lastShowSensor + TIME_SHOW ) {
      lastShowSensor = time;

      tft_draw(varParams);
    }


    float vs = 0.;
    if (time > lastReadSensor + TIME_SENSOR) {

      pressure_p = ( analogRead(PIN_PRESSURE)/ (1023.) - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20;//MPX5010

      vlevel = float(analogRead(PIN_MPX_LEV))/1024.*1.1*VOLTAGE_CONV;

      // Is like 1/vs
      vs = vlevel /** vfactor*/;
      
      int16_t adc0 = ads.readADC_SingleEnded(0);
      Voltage = (adc0 * 0.1875) * 0.001; //Volts

      p_dpt = ( (Voltage - vzero)/vs   - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20; //WITH TRIM
      
      byte pos = findClosest(dp, 55, p_dpt);
      //flux should be shifted up (byte storage issue)
      _flux = po_flux[pos] - 100 + ( float (po_flux[pos + 1] - 100) - float (po_flux[pos] - 100) ) * ( p_dpt - float(dp[pos]) ) / (float)( dp[pos + 1] - dp[pos]);
      _flux *= 16.6667;

      if (varParams.filter == 1) {
        for (int i = 0; i < 4; i++) {
          _flux_fil[i] = _flux_fil[i + 1];
        }
        _flux_fil[4] = _flux;
        _flux_sum = 0.;
        for (int i = 0; i < 5; i++)
          _flux_sum += _flux_fil[i];

        flow_f = _flux_sum / 5.;
      } else {
        flow_f = _flux;
      }

      if (_flux > 0) {
        _mlInsVol += flow_f * float((millis() - lastReadSensor)) * 0.001; //flux in l and time in msec, results in ml
      } else {
        _mlExsVol -= flow_f * float((millis() - lastReadSensor)) * 0.001; //flux in l and time in msec, results in ml
      }

      lastReadSensor = millis();

      //CHECK PIP AND PEEP (OUTSIDE ANY CYCLE!!)
      if (pressure_p > pressure_max) {
        pressure_max = pressure_p;
      }
      if (pressure_p < pressure_min) {
        pressure_min = pressure_p;
      }
      if (calibration_run) {
        vcorr_count ++;
        //According to datasheet
        //vout = vs(0.09*P + 0.04) +/ERR
        verror_sum += ( Voltage - 0.04 * vs); //-5*0.04
        //Serial.println("Calibration sum: "+ String(verror_sum));
        //Serial.println("readed: "+ String(Voltage - 0.04 * vs));
      }
    }//Read Sensor

    if ( ventilation -> getCycleNum () != last_cycle ) {
        int vt = (_mllastInsVol + _mllastInsVol) / 2;
      if (vt < varParams.alarm_vt)  isalarmvt_on = 1;
      else              isalarmvt_on = 0;
      if ( last_pressure_max > varParams.alarm_max_pressure + 1 ) {
          if ( last_pressure_min < varParams.alarm_peep_pressure - 1) {
          if (!isalarmvt_on)  alarm_state = 3;
          else                alarm_state = 13;
        } else {
          if (!isalarmvt_on)  alarm_state = 2;
          else                alarm_state = 12;
        }
      } else {
          if ( last_pressure_min < varParams.alarm_peep_pressure - 1 ) {
          if (!isalarmvt_on) alarm_state = 1;
          else               alarm_state = 11;
        } else {
          if (!isalarmvt_on)  alarm_state = 0;
          else                alarm_state = 10;
        }
      }

      last_cycle = ventilation->getCycleNum();

      if (!calibration_run) {
          printMenu(menuV2, varParams, millis());
          last_update_display = time;
      } else {
          lcd.clear();
          writeLine(menuV2, 1, "Calibracion flujo", 0);
          writeLine(menuV2, 2, "Ciclo: " + String(calib_cycle+1) + "/" + String(CALIB_CYCLES), 0);
      }

//#ifdef DEBUG_PID
//      if (varParams.vent_mode = VENTMODE_PCL) {
//        float err = (float)(pressure_max - varParams.peakInspiratoryPressure) / options.peakInspiratoryPressure;
//        errpid_prom += fabs(err);
//        errpid_prom_sig += err;
//        Serial.println("Error PID: "); Serial.print(err, 5);
//        ciclo_errpid++;
//
//        if (ciclo_errpid > 4) {
//          errpid_prom /= 5.; errpid_prom_sig /= 5.;
//          Serial.print(varParams.peakInspiratoryPressure); Serial.print(" "); Serial.print(errpid_prom, 5); Serial.print(" "); Serial.println(errpid_prom_sig, 5);
//          errpid_prom = 0.; errpid_prom_sig = 0.;
//          ciclo_errpid = 0;
//        }
//      }
//#endif
      if (!digitalRead(PIN_POWEROFF)) {
        digitalWrite(YELLOW_LED, HIGH);
      } else {
        digitalWrite(YELLOW_LED, LOW);
      }

      if (calibration_run) {
        verror = verror_sum / float(vcorr_count);
        Serial.println("Calibration iter, cycle, verror, sum: " + String(vcorr_count) + ", " +
                                                                  String(calib_cycle) + ", " +
                                                                  String(verror) + ", " +
                                                                  String(verror_sum_outcycle));
        vcorr_count = verror_sum = 0.;
        calib_cycle ++;
        verror_sum_outcycle += verror;
        if (calib_cycle >= CALIB_CYCLES ) {
          calibration_run = false;
          vzero = verror_sum_outcycle / float(CALIB_CYCLES);
          Serial.println("Calibration verror: " + String(vzero));
          lcd.clear();
          tft.fillScreen(ILI9341_BLACK);

      }
    } 

    #ifdef TEMP_TEST
    if (time > lastReadTemp + TIME_READ_TEMP){
      lastReadTemp = time;
      sensors.requestTemperatures();
      temp=sensors.getTempCByIndex(0);
      //Serial.println ("Temp: " + String(temp));
    }
    tft.fillRect(200,100,20,40, ILI9341_BLUE);    
    print_float(100,200,temp);
    #endif TEMP_TEST+

    }//change cycle

    if (!calibration_run) {
      if (display_needs_update) {
        printMenu(menuV2, varParams, millis());
        display_needs_update = false;
      }
    }
  
    if ( menuV2.menuState.updatedOptions ) {
        ventilation->updateParameters();
        menuV2.menuState.updatedOptions = false;
    }

    //HERE changed_options flag is not updating until cycle hcanges
    if (!calibration_run) {
        if (show_changed_options && ((millis() - last_update_display) > TIME_UPDATE_DISPLAY) ) {
        printMenu(menuV2, varParams, millis());
        last_update_display = millis();
        show_changed_options = false;
      }
    }

    //        if (alarm_state > 0) {
    //
    //              if (!buzzmuted) {
    //                  if (millis() > timebuzz + TIME_BUZZER) {
    //                      timebuzz=millis();
    //                      isbuzzeron=!isbuzzeron;
    //                      if (isbuzzeron){
    //                          digitalWrite(PIN_BUZZER,BUZZER_LOW);
    //                      }
    //                      else {
    //                          digitalWrite(PIN_BUZZER,!BUZZER_LOW);
    //                      }
    //                  }
    //              } else {  //buzz muted
    //                  digitalWrite(PIN_BUZZER,!BUZZER_LOW);
    //              }
    //        } else {//state > 0
    //          digitalWrite(PIN_BUZZER,!BUZZER_LOW);
    //          isbuzzeron=true;        //Inverted logic
    //        }

    //! sleep_mode
  } else {
    if (put_to_sleep) {
      //tft.fillScreen(ILI9341_BLACK);
      digitalWrite(PIN_LCD_EN, HIGH);
      put_to_sleep = false;
      print_bat_time = time;
      print_bat();
      digitalWrite(LCD_SLEEP, LOW);
      digitalWrite(TFT_SLEEP, LOW);
      //digitalWrite(PIN_STEPPER, LOW); //TODO: call it here (now is inside stepper)
      ventilation->forceStop();
      //digitalWrite(PIN_BUZZER, !BUZZER_LOW); //Buzzer inverted
      lcd.clear();
    }
    if (time > print_bat_time + 5000) {
      print_bat();
      print_bat_time = time;
    }
    time = millis();
  }

  #ifdef BAT_TEST
  if ( time > lastShowBat + TIME_SHOW_BAT ){
    lastShowBat = time;
    Serial.println("last show bat " + String(lastShowBat));
    float level = calc_bat(5);
    Serial.println(String(time)+", " +String(level));
  }
  #endif BAT_TEST

}//LOOP

void timer1Isr(void) {
  ventilation->update();
  //alarms->update(ventilation->getPeakInspiratoryPressure());
}

void timer3Isr(void) {
  stepper->run();
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
  }
  return i;
}

bool debounce(bool last, int pin) {
  bool current = digitalRead(pin);
  if (last != current) {
    delay(50);
    current = digitalRead(pin);
  }
  return current;
}

void check_buzzer_mute() {
  curr_mute = debounce ( last_mute, PIN_MUTE );         //Debounce for Up button
  if (last_mute == HIGH && curr_mute == LOW && !buzzmuted) {
    mute_count = time;
    buzzmuted = true;
  }
  last_mute = curr_mute;
  if (buzzmuted) {
    if (time > mute_count  + TIME_MUTE)  //each count is every 500 ms
      buzzmuted = false;
  }
}

void read_memory() {
  int eeAddress = 0;
  EEPROM.get(0, last_cycle); eeAddress += sizeof(unsigned long);
//  EEPROM.get(eeAddress, p_trim);    eeAddress += sizeof(p_trim);
//  EEPROM.get(eeAddress, autopid);   eeAddress += sizeof(autopid);
//  EEPROM.get(eeAddress, min_cd);    eeAddress += sizeof(min_cd);
//  EEPROM.get(eeAddress, max_cd);    eeAddress += sizeof(max_cd);
//  EEPROM.get(eeAddress, min_speed); eeAddress += sizeof(min_speed);
//  EEPROM.get(eeAddress, max_speed); eeAddress += sizeof(max_speed);
//  EEPROM.get(eeAddress, min_accel); eeAddress += sizeof(min_accel);
//  EEPROM.get(eeAddress, max_accel); eeAddress += sizeof(max_accel);
//  EEPROM.get(eeAddress, min_pidk);  eeAddress += sizeof(min_pidk);
//  EEPROM.get(eeAddress, max_pidk);  eeAddress += sizeof(max_pidk);
  EEPROM.get(eeAddress, alarm_vt);  eeAddress += sizeof(alarm_vt);
  EEPROM.get(eeAddress, filter);    eeAddress += sizeof(filter);
//  EEPROM.get(eeAddress, pfmin);     eeAddress += sizeof(pfmin);
//  EEPROM.get(eeAddress, pfmax);     eeAddress += sizeof(pfmax);
//  EEPROM.get(eeAddress, dpip_b);    eeAddress += sizeof(dpip_b);
//  EEPROM.get(eeAddress, min_pidi);  eeAddress += sizeof(min_pidi);
//  EEPROM.get(eeAddress, max_pidi);  eeAddress += sizeof(max_pidi);
//  EEPROM.get(eeAddress, min_pidd);  eeAddress += sizeof(min_pidd);
//  EEPROM.get(eeAddress, max_pidd);  eeAddress += sizeof(max_pidd);
//  EEPROM.get(eeAddress, p_acc);      eeAddress += sizeof(p_acc);
//  EEPROM.get(eeAddress, f_acc_b);    eeAddress += sizeof(f_acc_b);
}

void write_memory() {
  int eeAddress = 0;
  EEPROM.put(0, last_cycle);        eeAddress += sizeof(unsigned long);
//  EEPROM.put(eeAddress, p_trim);    eeAddress += sizeof(p_trim);
//  EEPROM.put(eeAddress, autopid);   eeAddress += sizeof(autopid);
//  EEPROM.put(eeAddress, min_cd);    eeAddress += sizeof(min_cd);
//  EEPROM.put(eeAddress, max_cd);    eeAddress += sizeof(max_cd);
//  EEPROM.put(eeAddress, min_speed); eeAddress += sizeof(min_speed);
//  EEPROM.put(eeAddress, max_speed); eeAddress += sizeof(max_speed);
//  EEPROM.put(eeAddress, min_accel); eeAddress += sizeof(min_accel);
//  EEPROM.put(eeAddress, max_accel); eeAddress += sizeof(max_accel);
//  EEPROM.put(eeAddress, min_pidk);  eeAddress += sizeof(min_pidk);
//  EEPROM.put(eeAddress, max_pidk);  eeAddress += sizeof(max_pidk);
  EEPROM.put(eeAddress, alarm_vt);  eeAddress += sizeof(alarm_vt);
  EEPROM.put(eeAddress, filter);    eeAddress += sizeof(filter);
//  EEPROM.put(eeAddress, pfmin);     eeAddress += sizeof(pfmin);
//  EEPROM.put(eeAddress, pfmax);     eeAddress += sizeof(pfmax);
//  EEPROM.put(eeAddress, dpip_b);    eeAddress += sizeof(dpip_b);
//  EEPROM.put(eeAddress, min_pidi);  eeAddress += sizeof(min_pidi);
//  EEPROM.put(eeAddress, max_pidi);  eeAddress += sizeof(max_pidi);
//  EEPROM.put(eeAddress, min_pidd);  eeAddress += sizeof(min_pidd);
//  EEPROM.put(eeAddress, max_pidd);  eeAddress += sizeof(max_pidd);
//  EEPROM.put(eeAddress, p_acc);      eeAddress += sizeof(p_acc);
//  EEPROM.put(eeAddress, f_acc_b);    eeAddress += sizeof(f_acc_b);
}
