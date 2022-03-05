#include "pinout.h"
#include "Utils.h"
#include "MechVentilation.h"
#include "DpFluxCalculator.h"
#include "src/TimerOne/TimerOne.h"
#include "src/TimerTwo/TimerTwo.h"
#include "src/TimerThree/TimerThree.h"

#include "menu.h"
#include "display.h"
#include "SensorRead.h"
#include "AlarmValidation.h"
#include "CalibrationUtils.h"

#include "src/AutoPID/AutoPID.h"
#ifdef ACCEL_STEPPER
#include "src/AccelStepper/AccelStepper.h"
#include "src/FlexyStepper/FlexyStepper.h"
#endif

#include "src/Pressure_Sensor/Pressure_Sensor.h"  //LUCIANO: MPX5050DP
#include <EEPROM.h>

#define CALIB_CYCLES  5

bool init_verror;
byte Cdyn;
bool autopid;
bool filter;
//bool sleep_mode;
//bool put_to_sleep, wake_up;
unsigned long print_bat_time;

bool drawing_cycle = 0;//TOD: Move to class member

// FOR ADS
#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads(0x48);           //Conversor AD para leer mejor el flujo a partir de la presion
float _mlInsVol = 0;
float _mlExsVol = 0;
int _mllastInsVol, _mllastExsVol;
unsigned long mute_count;

void read_memory(); //Lee la EEPROM, usa variables externas, quiza deberian englobarse en un vector dinamico todos los offsets
void write_memory();

int Compression_perc = 8; //80%

#ifdef ACCEL_STEPPER
AccelStepper *stepper = new AccelStepper(
  AccelStepper::DRIVER,
  PIN_STEPPER_STEP,
  PIN_STEPPER_DIRECTION);
#else
FlexyStepper * stepper = new FlexyStepper();
#endif

byte alarm_state = 0; //0: No alarm 1: peep 2: pip 3:both
//////////////////////////
// - EXTERNAL VARIABLES //
//////////////////////////
#ifdef LCD_I2C
LiquidCrystal_I2C lcd(0x3F, 20, 4);
#else
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
#endif

//MUTE
boolean last_mute, curr_mute;
unsigned long time_mute;

boolean buzzmuted;
unsigned long timebuzz = 0;
bool isbuzzeron = false;


Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

SystemState systemState;

byte vcorr_count;
byte p_trim = 100;
float pressure_p;   //EXTERN!!
float last_pressure_max, last_pressure_min, last_pressure_peep;
float pressure_peep;

byte vent_mode = VENTMODE_MAN; //0
//Adafruit_BMP280 _pres1Sensor;
Pressure_Sensor _dpsensor;
float verrp;
float _flux, flow_f;
//#ifdef FILTER_FLUX
float _flux_fil[5];
float _mlInsVol2;

//#endif

bool send_data = false;
char tempstr[5], tempstr2[5];
int curr_sel, old_curr_sel;
float p_dpt;

unsigned long lastReadSensor = 0;
unsigned long lastShowSensor = 0;
unsigned long lastSave = 0;

State static lastState;
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
//TODO: READ FROM EEPROM
byte alarm_max_pressure = 35;
byte alarm_peep_pressure = 5;
byte isalarmvt_on;
int alarm_vt = 200;

//MENU
unsigned long lastButtonPress;
float verror, verror_sum, verror_sum_outcycle, vzero;  //verror sum is intra cycle, verror_sum_outcycle is inter-cycle

bool change_pid_params = false;

byte encoderPos = 1; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
byte oldEncPos = 1; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

byte max_sel, min_sel; //According to current selection

void check_buzzer_mute();
void check_sleep_mode();  //Batt charge only

bool isitem_sel;
byte old_menu_pos = 0;
byte old_menu_num = 0;

AutoPID * pid;

MechVentilation * ventilation;
VentilationOptions_t options;


int bck_state ;     // current state of the button
int last_bck_state ; // previous state of the button
int startPressed ;    // the moment the button was pressed
int endPressed ;      // the moment the button was released
int holdTime ;        // how long the button was hold
int idleTime ;        // how long the button was idle

// CALIBRATION: TODO: MAKE A CLASS
float vsupply, vsupply_0;
float vlevel,vfactor;

void setup() {

  Serial.begin(115200);
  
  analogReference(INTERNAL1V1); // use AREF for reference voltage
    
  init_display();
  isitem_sel = false;

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

  max_cd = 40; //T MODIFY: READ FROM MEM
  min_cd = 10;
  min_speed = 250;  // x microsteps
  max_speed = 750;  // x Microsteps, originally 16000 (with 16 ms = 750)
  max_accel = 600;
  min_accel = 200;
  change_pid_params = true; //To calculate at first time

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

  ventilation = new MechVentilation(
    stepper,
    pid,
    options
  );

  delay(100);

  // TODO: Esperar aqui a iniciar el arranque desde el serial

  // Habilita el motor
  digitalWrite(PIN_EN, LOW);

  writeLine(1, "RespirAR FIUBA - MIR", 4);
  writeLine(2, "v2.0.1", 8);


  ads.begin();
  verror = verror_sum = verror_sum_outcycle = 0.;
  //TODO: Put all this inside a calib class
  vcorr_count = 0;
  
  float verrp = 0.;
  vsupply_0 = 0.;
    for (int i = 0; i < 100; i++) {

      vsupply_0 += float(analogRead(PIN_MPX_LEV))/1024.*1.1*VOLTAGE_CONV;
      delay(10);
    }
  vsupply_0 /= 100.; //
  
  Serial.println("Vsupply_0: " + String (vsupply_0));
  //vfactor = 5./vsupply_0; //
  
  Serial.print("dp (Flux) MPX Volt (mV) at p0: "); Serial.println(verror * 1000, 3);
  //  Serial.print("pressure  MPX Volt (mV) at p0: "); Serial.println(verrp * 1000, 3);

  Serial.print("dp  error : "); Serial.println(-verror / (5.*0.09));

  systemState.vent_mode = VENTMODE_MAN; //0
  systemState.sleep_mode = false;
  systemState.put_to_sleep = false;
  systemState.wake_up = false;
  systemState.display_needs_update = false;

  // configura la ventilación
  ventilation -> start();
  ventilation -> update(systemState);

  //
#ifdef ACCEL_STEPPER
  stepper->setSpeed(STEPPER_HOMING_SPEED);

  long initial_homing = -1;
  //// HOMING TODO: PASAR NUEVAMENTE ESTA VARIABLE A PRIVADA
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
  Serial.print("Position "); Serial.print(position);
  stepper->setCurrentPosition(STEPPER_LOWEST_POSITION);
  position = stepper->currentPosition();
  Serial.print("Position "); Serial.print(position);

  Serial.println("home end");
#endif

  //sensors -> readPressure();
  display_lcd(systemState);

  //ENCODER
  curr_sel = old_curr_sel = 1; //COMPRESSION

  pinMode(PIN_ENC_SW, INPUT_PULLUP);

  lastReadSensor =   lastShowSensor = millis();
  lastState = ventilation->getState();
  last_update_display = millis();

#ifdef DEBUG_UPDATE

#endif

  //STEPPER
  last_stepper_time = millis();
  last_vent_time = millis();

  //Serial.print(",0,50");

  Timer3.initialize(TIME_STEPPER_ISR_MICROS);
  Timer3.attachInterrupt(timer3Isr);

  Timer1.initialize(TIME_BASE_MICROS);  //BEFORE WERE 20...
  Timer1.attachInterrupt(timer1Isr);
  //Timer2.setPeriod(20000);
  //Timer2.attachInterrupt(timer2Isr);
  Serial.println("Reading ROM");
#ifdef DEBUG_UPDATE
  Serial.print("Honey Volt at p0: "); Serial.println(analogRead(A0) / 1023.);
#endif

  read_memory();

  f_acc = (float)f_acc_b / 10.;
  dpip = (float)dpip_b / 10.;

  Serial.print("Maxcd: "); Serial.println(max_cd);

  Serial.print("LAST CYCLE: "); Serial.println(last_cycle);
  ventilation->setCycleNum(last_cycle);

  tft.begin();
  tft.fillScreen(ILI9341_BLACK);


  digitalWrite(BCK_LED, LOW);
  buzzmuted = false;
  last_mute = HIGH;
  mute_count = 0;

  lcd.clear();

  pf_min = (float)pfmin / 50.;
  pf_max = (float)pfmax / 50.;
  peep_fac = -(pf_max - pf_min) / 15.*last_pressure_min + pf_max;

}

/////////////// CALIBRATION
bool  calibration_run = true;
byte  calib_cycle = 0;

////////////////////////////////////////
////////////// MAIN LOOP ///////////////
////////////////////////////////////////
void loop() {

  //digitalWrite(LCD_SLEEP, HIGH); //LOW, INVERTED

  if (!systemState.sleep_mode) {
      if (systemState.wake_up) {
          lcd.clear();
          init_display();
          display_lcd(systemState);    //TODO: Pass mech vent as argument in display
          tft.fillScreen(ILI9341_BLACK);
          systemState.wake_up = false;
      }
    State state = ventilation->getState();
    check_encoder(systemState);

    time = millis();
    check_buzzer_mute();

    if (millis() > lastSave + TIME_SAVE) {
      write_memory();
      lastSave = millis();
    }

    if ( time > lastShowSensor + TIME_SHOW ) {
        #ifdef DEBUG_STEPPER
        //      unsigned long reltime = ventilation->getMSecTimerCnt();
        //      Serial.print("Rel Msec: ");Serial.print(reltime);Serial.print(", Abs: ");
        //      Serial.println(time);
        #endif
        lastShowSensor = time;
        //           Serial.print(int(cycle_pos));Serial.print(",");
        //           Serial.print(Voltage,5);Serial.print(",");
        //           Serial.print(verror,3);Serial.print(",");
        //           Serial.print(p_dpt,5);Serial.print(",");
        //           Serial.println(flow_f,2);
        tft_draw();
    }

    if (time > lastReadSensor + TIME_SENSOR) {
//      //According to datasheet
//      //vout = vs(0.09*P + 0.04) +/ERR
//      // P = ( vout/vs - 0.04 )/0.09 So, vout/vs if
//      //IF vs = 5V
//      #ifdef USING_1v1_4PRESS
//      pressure_p = ( analogRead(PIN_PRESSURE) / (1023.)/*- verrp * 0.2 */ - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20;//MPX5010
//      #else //Original, pressure sensor connected to A0
//      pressure_p = ( analogRead(A0) / (1023.) /*- verrp * 0.2 */ - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20;//MPX5010
//      #endif
//
//      vlevel = float(analogRead(PIN_MPX_LEV))/1024.*1.1*VOLTAGE_CONV;
        int16_t pressureFromPin = analogRead(PIN_PRESSURE);
        int16_t mpxLevelFromPin = analogRead(PIN_MPX_LEV);
        int16_t adc0 = ads.readADC_SingleEnded(0);
//      float voltage = (adc0 * 0.1875) * 0.001; //Volts
//
//      //With constant correction
//      p_dpt = ( (voltage - vzero)/vlevel - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20; //WITH TRIM
//
//      _flux = findFlux(p_dpt);
        SensorData sensorData = readSensorData(pressureFromPin, mpxLevelFromPin, adc0, vzero);
        //copy results to extern variables to keep compatibility
        pressure_p = sensorData.pressure_p;
        vlevel = sensorData.vlevel;
        p_dpt = sensorData.p_dpt;
        _flux = sensorData.flux;

//      if (filter) {
//        flux_count++;    //Filter
//        for (int i = 0; i < 4; i++) {
//          _flux_fil[i] = _flux_fil[i + 1];
//        }
//        _flux_fil[4] = _flux;
//        float _flux_sum = 0.;
//        for (int i = 0; i < 5; i++) {
//            _flux_sum += _flux_fil[i];
//        }
//
//        flow_f = _flux_sum / 5.;
//      } else {
//        flow_f = _flux;
//      }
//
//      if (_flux > 0) {
//        _mlInsVol += flow_f * float((millis() - lastReadSensor)) * 0.001; //flux in l and time in msec, results in ml
//      } else {
//        _mlExsVol -= flow_f * float((millis() - lastReadSensor)) * 0.001; //flux in l and time in msec, results in ml
//      }
//
//      lastReadSensor = millis();
//
//      //CHECK PIP AND PEEP (OUTSIDE ANY CYCLE!!)
//      if (pressure_p > pressure_max) {
//        pressure_max = pressure_p;
//      }
//      if (pressure_p < pressure_min) {
//        pressure_min = pressure_p;
//      }
        SensorProcessedData oldData;
        copyFloatArray(_flux_fil, oldData.flux_fil, 5);
        oldData.mlInsVol = _mlInsVol;
        oldData.mlExsVol = _mlExsVol;
        oldData.pressure_min = pressure_min;
        oldData.pressure_max = pressure_max;

        float time = millis();
        SensorProcessedData processedData = updateProcessedSensorData(sensorData, filter, time, lastReadSensor, oldData);

        flow_f = processedData.flow_f;
        copyFloatArray(processedData.flux_fil, _flux_fil , 5);
        _mlInsVol = processedData.mlInsVol;
        _mlExsVol = processedData.mlExsVol;
        pressure_min = processedData.pressure_min;
        pressure_max = processedData.pressure_max;

        lastReadSensor = millis();

    }//Read Sensor

    if ( ventilation->getCycleNum() != last_cycle ) {
      int vt = (_mllastInsVol + _mllastInsVol) / 2; // check with Luciano
//      if (vt < alarm_vt)  isalarmvt_on = 1;
//      else              isalarmvt_on = 0;
//      byte isalarmvt_on = vt < alarm_vt ? 1 : 0;
//      if ( last_pressure_max > alarm_max_pressure + 1 ) {
//        if ( last_pressure_min < alarm_peep_pressure - 1) {
//          if (!isalarmvt_on)  alarm_state = 3;
//          else                alarm_state = 13;
//        } else {
//          if (!isalarmvt_on)  alarm_state = 2;
//          else                alarm_state = 12;
//        }
//      } else {
//        if ( last_pressure_min < alarm_peep_pressure - 1 ) {
//          if (!isalarmvt_on) alarm_state = 1;
//          else               alarm_state = 11;
//        } else {
//          if (!isalarmvt_on)  alarm_state = 0;
//          else                alarm_state = 10;
//        }
//      }
      alarm_state = getAlarmState(vt, alarm_vt, last_pressure_max, last_pressure_min, alarm_max_pressure, alarm_peep_pressure);

      last_cycle = ventilation->getCycleNum();

      display_lcd(systemState);

      last_update_display = time;

//#ifdef DEBUG_PID TODO: is not defined
//      if (systemState.vent_mode = VENTMODE_PCL) {
//        float err = (float)(pressure_max - options.peakInspiratoryPressure) / options.peakInspiratoryPressure;
//        errpid_prom += fabs(err);
//        errpid_prom_sig += err;
//        Serial.println("Error PID: "); Serial.print(err, 5);
//        ciclo_errpid++;
//
//        if (ciclo_errpid > 4) {
//          errpid_prom /= 5.; errpid_prom_sig /= 5.;
//          Serial.print(options.peakInspiratoryPressure); Serial.print(" "); Serial.print(errpid_prom, 5); Serial.print(" "); Serial.println(errpid_prom_sig, 5);
//          errpid_prom = 0.; errpid_prom_sig = 0.;
//          ciclo_errpid = 0;
//        }
//      }
//#endif
      if (digitalRead(PIN_POWEROFF)) {
          digitalWrite(YELLOW_LED, HIGH);
      } else {
          digitalWrite(YELLOW_LED, LOW);
      }

      // TODO: looks like this always is zero
//      if (calibration_run) {
//      //NEW, CALIBRATION
//        verror = verror_sum / float(vcorr_count);
//
//        Serial.println("Calibration iter, cycle, verror, sum: " + String(vcorr_count) + ", " +
//                                                                  String(calib_cycle) + ", " +
//                                                                  String(verror) + ", " +
//                                                                  String(verror_sum_outcycle));
//        vcorr_count = verror_sum = 0.;
//        calib_cycle ++;
//        verror_sum_outcycle += verror;
//        if (calib_cycle >= CALIB_CYCLES ){
//          calibration_run = false;
//          vzero = verror_sum_outcycle / float(CALIB_CYCLES);
//          Serial.println("Calibration verror: " + String(vzero));
//
//        }
//      } else {
//        verror = verror_sum / float(vcorr_count);
//        vcorr_count = verror_sum = 0.;
//      }

        calibrate(calibration_run, verror_sum, vcorr_count, calib_cycle, verror_sum_outcycle, vzero);
    }//change cycle

    if (systemState.display_needs_update) {
        display_lcd(systemState);
        systemState.display_needs_update = false;
    }

    if ( update_options ) {
        ventilation->change_config(options);
        update_options = false;
    }

    //HERE changed_options flag is not updating until cycle hcanges
    if (show_changed_options && ((millis() - last_update_display) > time_update_display) ) {
        display_lcd(systemState);  //WITHOUT CLEAR!
        last_update_display = millis();
        show_changed_options = false;
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
    if (systemState.put_to_sleep) {
      tft.fillScreen(ILI9341_BLACK);
      digitalWrite(PIN_LCD_EN, HIGH);
      systemState.put_to_sleep = false;
      print_bat_time = time;
      print_bat();
      digitalWrite(PIN_BUZZER, !BUZZER_LOW); //Buzzer inverted
      lcd.clear();
    }
    if (time > print_bat_time + 5000) {
      print_bat();
      print_bat_time = time;
    }
    time = millis();
    check_bck_state(systemState);
  }

}//LOOP

void timer1Isr(void) {
  ventilation->update(systemState);
  //alarms->update(ventilation->getPeakInspiratoryPressure());
}

void timer3Isr(void)
{
#ifdef ACCEL_STEPPER
  stepper->run();
#else
  stepper -> processMovement(); //LUCIANO
#endif
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
  EEPROM.get(eeAddress, p_trim);    eeAddress += sizeof(p_trim);
  EEPROM.get(eeAddress, autopid);   eeAddress += sizeof(autopid);
  EEPROM.get(eeAddress, min_cd);    eeAddress += sizeof(min_cd);
  EEPROM.get(eeAddress, max_cd);    eeAddress += sizeof(max_cd);
  EEPROM.get(eeAddress, min_speed); eeAddress += sizeof(min_speed);
  EEPROM.get(eeAddress, max_speed); eeAddress += sizeof(max_speed);
  EEPROM.get(eeAddress, min_accel); eeAddress += sizeof(min_accel);
  EEPROM.get(eeAddress, max_accel); eeAddress += sizeof(max_accel);
  EEPROM.get(eeAddress, min_pidk);  eeAddress += sizeof(min_pidk);
  EEPROM.get(eeAddress, max_pidk);  eeAddress += sizeof(max_pidk);
  EEPROM.get(eeAddress, alarm_vt);  eeAddress += sizeof(alarm_vt);
  EEPROM.get(eeAddress, filter);    eeAddress += sizeof(filter);
  EEPROM.get(eeAddress, pfmin);     eeAddress += sizeof(pfmin);
  EEPROM.get(eeAddress, pfmax);     eeAddress += sizeof(pfmax);
  EEPROM.get(eeAddress, dpip_b);    eeAddress += sizeof(dpip_b);
  EEPROM.get(eeAddress, min_pidi);  eeAddress += sizeof(min_pidi);
  EEPROM.get(eeAddress, max_pidi);  eeAddress += sizeof(max_pidi);
  EEPROM.get(eeAddress, min_pidd);  eeAddress += sizeof(min_pidd);
  EEPROM.get(eeAddress, max_pidd);  eeAddress += sizeof(max_pidd);
  EEPROM.get(eeAddress, p_acc);      eeAddress += sizeof(p_acc);
  EEPROM.get(eeAddress, f_acc_b);    eeAddress += sizeof(f_acc_b);
}

void write_memory() {
  int eeAddress = 0;
  EEPROM.put(0, last_cycle);        eeAddress += sizeof(unsigned long);
  EEPROM.put(eeAddress, p_trim);    eeAddress += sizeof(p_trim);
  EEPROM.put(eeAddress, autopid);   eeAddress += sizeof(autopid);
  EEPROM.put(eeAddress, min_cd);    eeAddress += sizeof(min_cd);
  EEPROM.put(eeAddress, max_cd);    eeAddress += sizeof(max_cd);
  EEPROM.put(eeAddress, min_speed); eeAddress += sizeof(min_speed);
  EEPROM.put(eeAddress, max_speed); eeAddress += sizeof(max_speed);
  EEPROM.put(eeAddress, min_accel); eeAddress += sizeof(min_accel);
  EEPROM.put(eeAddress, max_accel); eeAddress += sizeof(max_accel);
  EEPROM.put(eeAddress, min_pidk);  eeAddress += sizeof(min_pidk);
  EEPROM.put(eeAddress, max_pidk);  eeAddress += sizeof(max_pidk);
  EEPROM.put(eeAddress, alarm_vt);  eeAddress += sizeof(alarm_vt);
  EEPROM.put(eeAddress, filter);    eeAddress += sizeof(filter);
  EEPROM.put(eeAddress, pfmin);     eeAddress += sizeof(pfmin);
  EEPROM.put(eeAddress, pfmax);     eeAddress += sizeof(pfmax);
  EEPROM.put(eeAddress, dpip_b);    eeAddress += sizeof(dpip_b);
  EEPROM.put(eeAddress, min_pidi);  eeAddress += sizeof(min_pidi);
  EEPROM.put(eeAddress, max_pidi);  eeAddress += sizeof(max_pidi);
  EEPROM.put(eeAddress, min_pidd);  eeAddress += sizeof(min_pidd);
  EEPROM.put(eeAddress, max_pidd);  eeAddress += sizeof(max_pidd);
  EEPROM.put(eeAddress, p_acc);      eeAddress += sizeof(p_acc);
  EEPROM.put(eeAddress, f_acc_b);    eeAddress += sizeof(f_acc_b);
}

