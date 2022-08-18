#include "pinout.h"
#include "MechVentilation.h"
#include "src/TimerOne/TimerOne.h"
#include "src/TimerTwo/TimerTwo.h"
#include "src/TimerThree/TimerThree.h"

#include "menu.h"
#include "display.h"

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
bool sleep_mode;
bool put_to_sleep, wake_up;
unsigned long print_bat_time;
bool motor_stopped;

bool drawing_cycle = 0;//TOD: Move to class member

// FOR ADS
#include <Wire.h>
//OLD
//#include <Adafruit_ADS1015.h>
//Adafruit_ADS1115 ads(0x48);           //Conversor AD para leer mejor el flujo a partir de la presion
//NEW
#include <Adafruit_ADS1X15.h>           //Currently is 
Adafruit_ADS1115 ads;

float Voltage = 0.0;
int vt;
float _mlInsVol = 0;
float _mlExsVol = 0;
int _mllastInsVol, _mllastExsVol;
unsigned long mute_count;

void read_memory(); //Lee la EEPROM, usa variables externas, quiza deberian englobarse en un vector dinamico todos los offsets
void write_memory();

int Compression_perc = 8; //80%

#ifdef ACCEL_STEPPER
AccelStepper *stepper; 
#else
FlexyStepper * stepper;
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

byte vcorr_count;
byte p_trim = 100;
float pressure_p;   //EXTERN!!
float last_pressure_max, last_pressure_min, last_pressure_peep;
float pressure_peep;

byte vent_mode = VENTMODE_MAN; //0
//Adafruit_BMP280 _pres1Sensor;
Pressure_Sensor _dpsensor;
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
int16_t adc0;

int max_accel, min_accel;
int max_speed, min_speed;
int min_pidk, max_pidk;
int min_pidi, max_pidi;
int min_pidd, max_pidd;
byte pfmin, pfmax;
float pf_min, pf_max;
float peep_fac;

//min_pidk=250;
//max_pidk=1000;
int min_cd, max_cd;
//max_cd=40;  //T MODIFY: READ FROM MEM
//min_cd=10;

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

//FLUX IS -100 to +100, has to be added 100
//ASSIMETRY IN MAX FLOW IS IN NEGATIVE (ORIGINAL CURVE)
//float dp[]={-2.444452733,-2.030351958,-1.563385753,-1.207061607,-0.877207832,-0.606462279,-0.491216024,-0.377891785,-0.295221736,-0.216332764,-0.151339196,-0.096530072,-0.052868293,-0.047781395,-0.039664506,-0.03312327,-0.028644966,-0.023566372,-0.020045692,-0.014830113,-0.011688636,-0.008176254,-0.006117271,-0.003937171,-0.001999305,-0.00090924,-0.00030358,0,0.000242233,0.000837976,0.002664566,0.004602432,0.007024765,0.009325981,0.012111664,0.01441288,0.017561913,0.023012161,0.029794693,0.037061691,0.043771552,0.051474571,0.05874157,0.109004974,0.176879848,0.260808033,0.365700986,0.504544509,0.630753349,0.795599072,1.216013465,1.60054669,2.087678384,2.547210457,3.074176245};
//byte po_flux[]={0,10,20,30,40,50,55,60,65,70,75,80,85,86,87,88,89,90,91,92,93,94,95,96,97,100,100,100,100,100,103,104,105,106,107,108,109,110,111,112,113,114,115,120,125,130,135,140,145,150,160,170,180,190,200};

//MAX FLUX IS IN ISPIRING POSITIVE (1st quad)
float dp[] = { -3.074176245, -2.547210457, -2.087678384, -1.60054669, -1.216013465, -0.795599072, -0.630753349, -0.504544509, -0.365700986, -0.260808033, -0.176879848, -0.109004974, -0.05874157, -0.051474571, -0.043771552, -0.037061691, -0.029794693, -0.023012161, -0.017561913, -0.01441288, -0.012111664, -0.009325981, -0.007024765, -0.004602432, -0.002664566, 0.00090924, 0.00030358, 0, -0.000242233, -0.000837976, 0.001999305, 0.003937171, 0.006117271, 0.008176254, 0.011688636, 0.014830113, 0.020045692, 0.023566372, 0.028644966, 0.03312327, 0.039664506, 0.047781395, 0.052868293, 0.096530072, 0.151339196, 0.216332764, 0.295221736, 0.377891785, 0.491216024, 0.606462279, 0.877207832, 1.207061607, 1.563385753, 2.030351958, 2.444452733};
byte po_flux[] = {0, 10, 20, 30, 40, 50, 55, 60, 65, 70, 75, 80, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 100, 100, 100, 100, 100, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 120, 125, 130, 135, 140, 145, 150, 160, 170, 180, 190, 200};

bool change_pid_params = false;

byte encoderPos = 1; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
byte oldEncPos = 1; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

byte max_sel, min_sel; //According to current selection

void check_buzzer_mute();
void autotrim_flux();
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

  pinMode(PIN_STEPPER, OUTPUT);
  digitalWrite(PIN_STEPPER, LOW);
  
  Serial.begin(115200);
  
  //analogReference(INTERNAL1V1); // use AREF for reference voltage
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

  delay(100);

  if (!digitalRead(PIN_POWEROFF)) {
    digitalWrite(YELLOW_LED, HIGH);
    Serial.println("Poweroff");
  }


  //  Serial.println("Tiempo del ciclo (seg):" + String(ventilation -> getExsuflationTime() + ventilation -> getInsuflationTime()));
  //  Serial.println("Tiempo inspiratorio (mseg):" + String(ventilation -> getInsuflationTime()));
  //  Serial.println("Tiempo espiratorio (mseg):" + String(ventilation -> getExsuflationTime()));

  // TODO: Esperar aqui a iniciar el arranque desde el serial

  // Habilita el motor
  digitalWrite(PIN_EN, LOW);

  writeLine(1, "RespirAR FIUBA", 4);
  writeLine(2, "v2.0.1", 8);

  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_BLUE);
  tft.setTextSize(4); 
  tft.setCursor(10, 40);     tft.println("RespirAR");   
  tft.setCursor(10, 80);     tft.println("FIUBA");

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

  ////// ANTES DE CONFIGURAR LA VENTILACION Y CHEQUEAR EL FIN DE CARRERA INICIO LOS MENUES
  bool init=false;
  byte bpm = DEFAULT_RPM;
  byte i_e = 2;
  Menu_inic menuini(&vent_mode, &bpm, &i_e);

  options.respiratoryRate = bpm;
  options.percInspEsp = i_e; //1:1 to 1:4, is denom
  vent_mode = VENTMODE_MAN;

  /////////////////// CALIBRACION /////////////////////////////////////
  bool fin = false;
  lcd.clear();
  writeLine(1, "Desconecte flujo", 0);
  writeLine(2, "y presione ok ", 0);

  delay (100); //Otherwise low enter button readed
  lastButtonPress = millis();
  while (!fin){
    if (digitalRead(PIN_MENU_EN) == LOW)  //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6) v
    if (millis() - lastButtonPress > 50) {
      fin = true;
      lastButtonPress = millis();
    }// if time > last button press  
  }


digitalWrite(PIN_STEPPER, HIGH);
delay(1000);
#ifdef ACCEL_STEPPER
stepper = new AccelStepper(
  AccelStepper::DRIVER,
  PIN_STEPPER_STEP,
  PIN_STEPPER_DIRECTION);
#else
stepper = new FlexyStepper();
#endif

  ventilation = new MechVentilation(
    stepper,
    pid,
    options
  );


  tft.fillScreen(ILI9341_BLACK);
  
  /////
  // configura la ventilación
  ventilation -> start();
  ventilation -> update();

  lcd.clear();
  writeLine(1, "Iniciando...", 0);
  
  ////
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
  //

  //sensors -> readPressure();
  display_lcd();

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


  digitalWrite(BCK_LED, LOW);
  buzzmuted = false;
  last_mute = HIGH;
  mute_count = 0;

  lcd.clear();

  pf_min = (float)pfmin / 50.;
  pf_max = (float)pfmax / 50.;
  peep_fac = -(pf_max - pf_min) / 15.*last_pressure_min + pf_max;

  sleep_mode = false;
  put_to_sleep = false;
  wake_up = false;

  //Serial.println("Vcc & Out MPX: " + String(analogRead(PIN_MPX_LEV)) + String(", ") + String(Voltage));
    
  Serial.println("Exiting setup");
  //TODO: CALIBRATION RUN ALSO SHOULD BE HERE

}


bool update_display = false;
byte pos;

/////////////// CALIBRATION
bool  calibration_run = true;
int   start_cyle = last_cycle;  //Used for calibration
byte  calib_cycle = 0;
float vs;
////////////////////////////////////////
////////////// MAIN LOOP ///////////////
////////////////////////////////////////
void loop() {

  //digitalWrite(LCD_SLEEP, HIGH); //LOW, INVERTED

  if (!sleep_mode) {
    if (wake_up) {
      digitalWrite(PIN_STEPPER, HIGH);
      digitalWrite(TFT_SLEEP, HIGH);
      digitalWrite(LCD_SLEEP, HIGH);
      lcd.clear();
      init_display();
      display_lcd();    //TODO: Pass mech vent as argument in display
      tft.begin();
      tft.fillScreen(ILI9341_BLACK);
      wake_up = false;
      ventilation->forceStart();
    }
    State state = ventilation->getState();
    check_encoder();

    time = millis();
    check_buzzer_mute();
    //Serial.print("Carga: ");Serial.println(analogRead(PIN_BAT_LEV));

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
//                 Serial.print(int(cycle_pos));Serial.print(",");
//                 Serial.println(Voltage,5);Serial.print(",");
//                 Serial.print(verror,3);Serial.print(",");
//                 Serial.print(p_dpt,5);Serial.print(",");
//      
//                 Serial.println(flow_f,2);

      tft_draw();

    }


    if (time > lastReadSensor + TIME_SENSOR) {
      //According to datasheet
      //vout = vs(0.09*P + 0.04) +/ERR
      // P = ( vout/vs - 0.04 )/0.09 So, vout/vs if 
      //IF vs = 5V     
      #ifdef USING_1v1_4PRESS
      pressure_p = ( analogRead(PIN_PRESSURE)/ (1023.)/*- verrp * 0.2 */ - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20;//MPX5010      
      #else //Original, pressure sensor connected to A0
      pressure_p = ( analogRead(A0) / (1023.) /*- verrp * 0.2 */ - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20;//MPX5010
      #endif

      vlevel = float(analogRead(PIN_MPX_LEV))/1024.*1.1*VOLTAGE_CONV;

      // Is like 1/vs
      vs = vlevel /** vfactor*/; 
      
      adc0 = ads.readADC_SingleEnded(0);
      Voltage = (adc0 * 0.1875) * 0.001; //Volts
      //DATASHEET:
      // Vo = Vs (  
      //ORIGINAL. 0.45 = 0.09 x 5V and 0.2 = 0.04 x 5V
       //According to datasheet
      //vout = vs(0.09*P + 0.04) +/ERR
      //p_dpt = ( Voltage /*- verror */- vzero - 0.20 ) / 0.45 * 1000 * DEFAULT_PA_TO_CM_H20; //WITH TRIM
      //BEFORE VERSION 2.0.1
      //p_dpt = ( Voltage /*- verror */        - 0.20 ) / 0.45 * 1000 * DEFAULT_PA_TO_CM_H20; 
      
      //With zero (see above) dp 
      //vzero = Vo - 0.04*vs
      p_dpt = ( (Voltage - vzero)/vs   - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20; //WITH TRIM
      
      pos = findClosest(dp, 55, p_dpt);
      //flux should be shifted up (byte storage issue)
      _flux = po_flux[pos] - 100 + ( float (po_flux[pos + 1] - 100) - float (po_flux[pos] - 100) ) * ( p_dpt - float(dp[pos]) ) / (float)( dp[pos + 1] - dp[pos]);
      _flux *= 16.6667;

      if (filter) {
        flux_count++;    //Filter
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
        Serial.println("readed: "+ String(Voltage - 0.04 * vs));
      } 
//      else { //This sums the feed error
//          verror_sum += vlevel;       // -5*0.04
//          vcorr_count ++;
//      }
      
    }//Read Sensor

    
    if (alarm_vt) {

    }
    if ( ventilation -> getCycleNum () != last_cycle ) {
      vt = (_mllastInsVol + _mllastInsVol) / 2;
      if (vt < alarm_vt)  isalarmvt_on = 1;
      else              isalarmvt_on = 0;
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

      last_cycle = ventilation->getCycleNum();

      if (!calibration_run){
        display_lcd();
        update_display = true;
        last_update_display = time;
      } else {
          lcd.clear();
          writeLine(1, "Calibracion flujo", 0);
          writeLine(2, "Ciclo: " + String(calib_cycle+1) + "/" + String(CALIB_CYCLES), 0);
      }
      
#ifdef DEBUG_PID
      if (vent_mode = VENTMODE_PCL) {
        float err = (float)(pressure_max - options.peakInspiratoryPressure) / options.peakInspiratoryPressure;
        errpid_prom += fabs(err);
        errpid_prom_sig += err;
        Serial.println("Error PID: "); Serial.print(err, 5);
        ciclo_errpid++;

        if (ciclo_errpid > 4) {
          errpid_prom /= 5.; errpid_prom_sig /= 5.;
          Serial.print(options.peakInspiratoryPressure); Serial.print(" "); Serial.print(errpid_prom, 5); Serial.print(" "); Serial.println(errpid_prom_sig, 5);
          errpid_prom = 0.; errpid_prom_sig = 0.;
          ciclo_errpid = 0;
        }
      }
#endif
      if (!digitalRead(PIN_POWEROFF)) {
        digitalWrite(YELLOW_LED, HIGH);
        Serial.println("Poweroff");
      } else {
        digitalWrite(YELLOW_LED, LOW);
      }

      if (calibration_run) {
      //NEW, CALIBRATION
        verror = verror_sum / float(vcorr_count);
        
       Serial.println("Calibration iter, cycle, verror, sum: " + String(vcorr_count) + ", " + 
                                                                  String(calib_cycle) + ", " + 
                                                                  String(verror) + ", " + 
                                                                  String(verror_sum_outcycle));
        vcorr_count = verror_sum = 0.;
        calib_cycle ++;
        //if (calib_cycle>1)
          verror_sum_outcycle += verror;
        if (calib_cycle >= CALIB_CYCLES ){
          calibration_run = false;
          vzero = verror_sum_outcycle / float(CALIB_CYCLES);
          Serial.println("Calibration verror: " + String(vzero));
          lcd.clear();
          tft.fillScreen(ILI9341_BLACK);

      }
    } 
//    else {
//        verror = verror_sum / float(vcorr_count);
//        vcorr_count = verror_sum = 0.;
//      }
    
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
      //show_changed_options=true;
    }

    //////////////// CAUTION
    // // //		WITH LATEST HARDWARE; IF VENTILATION UPDATE IS IN THE MAIN LOOP LIKE THIS
    // // //		MENU ACTIONS INTERFERE WITH VENTILATION MECHANICS
    //      if ( millis () - last_vent_time > TIME_BASE ) {
    //        ventilation -> update();
    //      }

    //HERE changed_options flag is not updating until cycle hcanges
    if (!calibration_run){
      if (show_changed_options && ((millis() - last_update_display) > time_update_display) ) {
        display_lcd();  //WITHOUT CLEAR!
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
      digitalWrite(PIN_STEPPER, LOW);
      ventilation->forceStop();
      //digitalWrite(PIN_BUZZER, !BUZZER_LOW); //Buzzer inverted
      lcd.clear();
    }
    if (time > print_bat_time + 5000) {
      print_bat();
      print_bat_time = time;
    }
    time = millis();
    check_bck_state();
  }

  //    #ifdef ACCEL_STEPPER
  //    stepper->run();
  //  #else
  //    stepper -> processMovement(); //LUCIANO
  //  #endif

}//LOOP

void timer1Isr(void) {
  ventilation->update();
  //alarms->update(ventilation->getPeakInspiratoryPressure());
}

//void timer2Isr(void)
//{
//  ventilation -> update();
//}

void timer3Isr(void)
{
#ifdef ACCEL_STEPPER
  stepper->run();
#else
  stepper -> processMovement(); //LUCIANO
#endif
}
//


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

void autotrim_flux() {

}