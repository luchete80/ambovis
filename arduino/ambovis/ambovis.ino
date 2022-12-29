#include "src/TimerOne/TimerOne.h"
#include "src/TimerThree/TimerThree.h"
#include "src/AutoPID/AutoPID.h"
#include "src/AccelStepper/AccelStepper.h"

#include "menu.h"
#include "display.h"
#include "pinout.h"
#include "MechVentilation.h"
#include "alarms.h"
#include "sensorcalculation.h"
#include "data_persistence.h"

#ifdef TEMP_TEST
#include <OneWire.h>
#include <DallasTemperature.h>
float temp;
#endif

byte Cdyn;
bool autopid;
bool filter;
bool sleep_mode = false;
bool put_to_sleep = false;
bool wake_up = false;
unsigned long print_bat_time;
bool drawing_cycle = 0;
volatile bool updateLimitsForInitInsuf = false;
byte Cdyn_pass[3];

// FOR ADS
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;
int _mllastInsVol, _mllastExsVol;

AccelStepper *stepper;

short alarm_state = 0;
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

//MUTE
bool last_mute;
bool buzzmuted;
unsigned long mute_count;

unsigned long timebuzz = 0;
bool isbuzzeron = false;
bool is_alarm_vt_on;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

byte vcorr_count = 0;
byte p_trim = 100;
float last_pressure_max, last_pressure_min;

byte vent_mode = VENTMODE_MAN; //0

char tempstr[5];
int curr_sel, old_curr_sel;

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
bool update_options = false;

unsigned long last_update_display;

unsigned long time;
byte cycle_pos;

SensorData sensorData;

int max_accel, min_accel;
int max_speed, min_speed;
int min_pidk, max_pidk;
int min_pidi, max_pidi;
int min_pidd, max_pidd;
byte pfmin, pfmax;
float pf_min, pf_max;
float peep_fac;
int min_cd, max_cd;

unsigned long last_cycle;
unsigned int _timeoutIns, _timeoutEsp; //In ms

byte menu_number = 0;
byte alarm_max_pressure = 35;
byte alarm_peep_pressure = 5;
byte isalarmvt_on;
int alarm_vt = 200;

//MENU
unsigned long lastButtonPress;
float verror, verror_sum, verror_sum_outcycle, vzero = 0.;  //verror sum is intra cycle, verror_sum_outcycle is inter-cycle

byte encoderPos = 1; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
byte oldEncPos = 1; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)

byte max_sel, min_sel; //According to current selection

void initTft(Adafruit_ILI9341& tft);
void waitForFluxDisconnected();
void searchHomePosition(AccelStepper* stepper);

bool isitem_sel =false;

AutoPID * pid;
MechVentilation * ventilation;
VentilationOptions_t options;

int bck_state ;     // current state of the button
int last_bck_state ; // previous state of the button
int startPressed ;    // the moment the button was pressed
int endPressed ;      // the moment the button was released
int holdTime ;        // how long the button was hold
int idleTime ;        // how long the button was idle

float fdiv = (float)(BATDIV_R1 + BATDIV_R2)/(float)BATDIV_R2;
float fac = 1.1/1024.*fdiv;

#ifdef TEMP_TEST
OneWire           oneWire(PIN_TEMP);
DallasTemperature sensors(&oneWire);
#endif
  
void setup() {
  Serial.begin(115200);
  
  analogReference(INTERNAL1V1); // use AREF for reference voltage

  init_display();

  initPins();

  pid = new AutoPID(PID_MIN, PID_MAX, PID_KP, PID_KI, PID_KD);

  max_cd = 40; //T MODIFY: READ FROM MEM
  min_cd = 10;
  min_speed = 250;  // x microsteps
  max_speed = 750;  // x Microsteps, originally 16000 (with 16 ms = 750)
  max_accel = 600;
  min_accel = 200;

  delay(100);
  digitalWrite(PIN_EN, LOW);

  writeLine(1, "RespirAR FIUBA", 4);
  writeLine(2, "v2.0.2", 8);

  initTft(tft);

  ads.begin();

  Menu_inic menuini(&vent_mode, &options.respiratoryRate, &options.percInspEsp);
  vent_mode = VENTMODE_MAN;

  waitForFluxDisconnected();

  digitalWrite(PIN_STEPPER, HIGH);
  delay(1000);

  stepper = new AccelStepper(
    AccelStepper::DRIVER,
    PIN_STEPPER_STEP,
    PIN_STEPPER_DIRECTION);

  ventilation = new MechVentilation(
    stepper,
    pid,
    options
  );

  ventilation -> start();
  ventilation -> update(sensorData);

  tft.fillScreen(ILI9341_BLACK);
  lcd.clear();
  writeLine(1, "Iniciando...", 0);

#ifdef ACCEL_STEPPER
  searchHomePosition(stepper);
#endif

  display_lcd();

  curr_sel = old_curr_sel = 1;

  lastShowSensor = last_update_display = sensorData.last_read_sensor = millis();

  #ifdef BAT_TEST
  lastShowBat = millis();
  #endif

  //STEPPER
  Timer3.initialize(TIME_STEPPER_ISR_MICROS);
  Timer3.attachInterrupt(timer3Isr);

  Timer1.initialize(TIME_BASE_MICROS);  //BEFORE WERE 20...
  Timer1.attachInterrupt(timer1Isr);

  SystemConfiguration_t systemConfiguration = read_memory();
  alarm_vt = systemConfiguration.alarm_vt;
  filter = systemConfiguration.filter;
  autopid = systemConfiguration.autopid;
  last_cycle = systemConfiguration.last_cycle;

  f_acc = (float)f_acc_b / 10.;
  dpip = (float)dpip_b / 10.;

  ventilation->setCycleNum(last_cycle);

  buzzmuted = false;
  last_mute = HIGH;
  mute_count = 0;

  lcd.clear();

  pf_min = (float)pfmin / 50.;
  pf_max = (float)pfmax / 50.;
  peep_fac = -(pf_max - pf_min) / 15.*last_pressure_min + pf_max;

  #ifdef TEMP_TEST
  sensors.begin();
  #endif
}

bool calibration_run = true;
byte calib_cycle = 0;
////////////////////////////////////////
////////////// MAIN LOOP ///////////////
////////////////////////////////////////
void loop() {

  if (!sleep_mode) {
    if (wake_up) {
      digitalWrite(PIN_STEPPER, HIGH);
      digitalWrite(TFT_SLEEP, HIGH);
      digitalWrite(LCD_SLEEP, HIGH);
      lcd.clear();
      init_display();
      display_lcd();
      tft.begin();
      tft.fillScreen(ILI9341_BLACK);
      wake_up = false;
      ventilation->forceStart();
    }
    check_encoder();

    time = millis();
    check_buzzer_mute(last_mute, mute_count, buzzmuted, time);

    if (time > lastSave + TIME_SAVE) {
        SystemConfiguration_t toPersist;
        toPersist.last_cycle = last_cycle;
        toPersist.alarm_vt = alarm_vt;
        toPersist.autopid = autopid;
        toPersist.filter = filter;
        write_memory(toPersist);
        lastSave = millis();
    }

    if ( millis() > lastShowSensor + TIME_SHOW ) {
        tft_draw(tft, sensorData, drawing_cycle, fac, alarm_state);
        lastShowSensor = millis();
    }


    if (time > sensorData.last_read_sensor + TIME_SENSOR) {
        int16_t adc0 = ads.readADC_SingleEnded(0);
        readSensor(sensorData, adc0, vzero, filter);
        float vs = sensorData.v_level;

        //CHECK PIP AND PEEP (OUTSIDE ANY CYCLE!!)
        if (sensorData.pressure_p > pressure_max) {
            pressure_max = sensorData.pressure_p;
        }
        if (sensorData.pressure_p < pressure_min) {
            pressure_min = sensorData.pressure_p;
        }
        if (calibration_run) {
            vcorr_count ++;
            verror_sum += (sensorData.voltage - 0.04 * vs); //-5*0.04
        }
    }//Read Sensor

    if (updateLimitsForInitInsuf) {
        last_pressure_max=pressure_max;
        last_pressure_min=pressure_min;
        pressure_max=0;
        pressure_min=60;
        for (int i=0;i<2;i++) Cdyn_pass[i]=Cdyn_pass[i+1];
        Cdyn_pass[2]=_mllastInsVol/(last_pressure_max-last_pressure_min);
        Cdyn=(Cdyn_pass[0]+Cdyn_pass[1]+Cdyn_pass[2])/3.;
        _mllastInsVol=int(sensorData.ml_ins_vol);
        _mllastExsVol=int(fabs(sensorData.ml_exs_vol));
        sensorData.ml_ins_vol=0.;
        sensorData.ml_exs_vol=0.;
        updateLimitsForInitInsuf = false;
    }

    if ( ventilation->getCycleNum() != last_cycle ) {
      int vt = (_mllastInsVol + _mllastInsVol) / 2;
      is_alarm_vt_on = vt < alarm_vt;
      alarm_state = getAlarmState(last_pressure_max, last_pressure_min,
                                  alarm_max_pressure, alarm_peep_pressure);

      last_cycle = ventilation->getCycleNum();

      if (!calibration_run) {
         display_lcd();
         last_update_display = time;
      } else {
          lcd.clear();
          writeLine(1, "Calibracion flujo", 0);
          writeLine(2, "Ciclo: " + String(calib_cycle+1) + "/" + String(CALIB_CYCLES), 0);
      }

      if (!digitalRead(PIN_POWEROFF)) {
        digitalWrite(YELLOW_LED, HIGH);
      } else {
        digitalWrite(YELLOW_LED, LOW);
      }

      if (calibration_run) {
        verror = verror_sum / float(vcorr_count);
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
      if (time > lastReadTemp + TIME_READ_TEMP) {
          lastReadTemp = time;
          sensors.requestTemperatures();
          temp=sensors.getTempCByIndex(0);
          //Serial.println ("Temp: " + String(temp));
      }
      tft.fillRect(200,100,20,40, ILI9341_BLUE);
      print_float(tft, 100,200,temp);
      #endif

    }//change cycle

    if (!calibration_run) {
      if (display_needs_update) {
        display_lcd();
        display_needs_update = false;
      }
    }
  
    if ( update_options ) {
        ventilation->change_config(options);
        update_options = false;
    }

    if (!calibration_run) {
        if (show_changed_options && ((millis() - last_update_display) > TIME_UPDATE_DISPLAY) ) {
            display_lcd();
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
      digitalWrite(PIN_LCD_EN, HIGH);
      put_to_sleep = false;
      print_bat_time = time;
      print_bat(tft, fac);
      digitalWrite(LCD_SLEEP, LOW);
      digitalWrite(TFT_SLEEP, LOW);
      //digitalWrite(PIN_STEPPER, LOW); //TODO: call it here (now is inside stepper)
      ventilation->forceStop();
      //digitalWrite(PIN_BUZZER, !BUZZER_LOW); //Buzzer inverted
      lcd.clear();
    }
    if (time > print_bat_time + 5000) {
        print_bat(tft, fac);
        print_bat_time = time;
    }
    time = millis();
    check_bck_state();
  }

    #ifdef BAT_TEST
    if ( time > lastShowBat + TIME_SHOW_BAT ) {
        lastShowBat = time;
        Serial.println("last show bat " + String(lastShowBat));
        float level = calc_bat(5, fac);
        Serial.println(String(time)+", " +String(level));
    }
    #endif BAT_TEST

}//LOOP

void timer1Isr() {
    ventilation->update(sensorData);
}

void timer3Isr() {
    stepper->run();
}

void initTft(Adafruit_ILI9341& tft) {
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_BLUE);
    tft.setTextSize(4);
    tft.setCursor(10, 40);     tft.println("RespirAR");
    tft.setCursor(10, 80);     tft.println("FIUBA");
}

void searchHomePosition(AccelStepper* stepper) {
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
    stepper->setCurrentPosition(STEPPER_LOWEST_POSITION);
}

void waitForFluxDisconnected() {
    lcd.clear();
    writeLine(1, "Desconecte flujo", 0);
    writeLine(2, "y presione ok ", 0);
    bool fin = false;
    delay(100); //Otherwise low enter button readed
    long lastButtonPress = millis();
    while (!fin) {
        if (digitalRead(PIN_MENU_EN) == LOW) {
            if (millis() - lastButtonPress > 50) {
                fin = true;
                lastButtonPress = millis();
            }
        }
    }
}
