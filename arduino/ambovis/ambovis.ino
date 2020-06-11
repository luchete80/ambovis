#include "pinout.h"
#include "MechVentilation.h"
#include "src/TimerOne/TimerOne.h"
#include "menu.h"

#include "src/AutoPID/AutoPID.h"
#ifdef ACCEL_STEPPER
#include "src/AccelStepper/AccelStepper.h"
#include "src/FlexyStepper/FlexyStepper.h"
#endif

#include "src/Pressure_Sensor/Pressure_Sensor.h"  //LUCIANO: MPX5050DP
#include <EEPROM.h>

#ifdef DEBUG_FLUX
float ins_prom, ins_error;
float ins_max, ins_min, err_sum;
unsigned long ciclo, ins_sum;
#endif
bool init_verror;
byte Cdyn;
bool autopid;

// FOR ADS
#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads(0x48);
float Voltage = 0.0;

float _mlInsVol = 0;
float _mlExsVol = 0;
int _mllastInsVol, _mllastExsVol;

int Compression_perc = 8; //80%

#ifdef ACCEL_STEPPER
AccelStepper *stepper = new AccelStepper(
  //AccelStepper::DRIVER,
  PIN_STEPPER_DIRECTION,
  PIN_STEPPER_STEP);
#else
FlexyStepper * stepper = new FlexyStepper();
#endif

static byte alarm_state = 0; //0: No alarm 1: peep 2: pip 3:both
//////////////////////////
// - EXTERNAL VARIABLES //
//////////////////////////
#ifdef LCD_I2C
LiquidCrystal_I2C lcd(0x3F, 20, 4);
#else
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
#endif

byte vcorr_count;
byte p_trim = 100;
float pressure_p;   //EXTERN!!
float last_pressure_max, last_pressure_min, last_pressure_peep;
float pressure_peep;

byte vent_mode = VENTMODE_PCL; //0
//Adafruit_BMP280 _pres1Sensor;
Pressure_Sensor _dpsensor;
float verrp;
float _flux,    flow_f;;
#ifdef FILTER_FLUX
float _flux_fil[5];
float _mlInsVol2;
float _flux_sum;
byte flux_count;
#endif

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

unsigned long last_cycle;

byte menu_number = 0;
byte alarm_max_pressure = 35;
byte alarm_peep_pressure = 5;

//MENU
unsigned long lastButtonPress;
float verror, verror_sum;

//FLUX IS -100 to +100, has to be added 100
//ASSIMETRY IN MAX FLOW IS IN NEGATIVE (ORIGINAL CURVE)
//float dp[]={-2.444452733,-2.030351958,-1.563385753,-1.207061607,-0.877207832,-0.606462279,-0.491216024,-0.377891785,-0.295221736,-0.216332764,-0.151339196,-0.096530072,-0.052868293,-0.047781395,-0.039664506,-0.03312327,-0.028644966,-0.023566372,-0.020045692,-0.014830113,-0.011688636,-0.008176254,-0.006117271,-0.003937171,-0.001999305,-0.00090924,-0.00030358,0,0.000242233,0.000837976,0.002664566,0.004602432,0.007024765,0.009325981,0.012111664,0.01441288,0.017561913,0.023012161,0.029794693,0.037061691,0.043771552,0.051474571,0.05874157,0.109004974,0.176879848,0.260808033,0.365700986,0.504544509,0.630753349,0.795599072,1.216013465,1.60054669,2.087678384,2.547210457,3.074176245};
//byte po_flux[]={0,10,20,30,40,50,55,60,65,70,75,80,85,86,87,88,89,90,91,92,93,94,95,96,97,100,100,100,100,100,103,104,105,106,107,108,109,110,111,112,113,114,115,120,125,130,135,140,145,150,160,170,180,190,200};

//MAX FLUX IS IN ISPIRING POSITIVE (1st quad)
float dp[] = { -3.074176245, -2.547210457, -2.087678384, -1.60054669, -1.216013465, -0.795599072, -0.630753349, -0.504544509, -0.365700986, -0.260808033, -0.176879848, -0.109004974, -0.05874157, -0.051474571, -0.043771552, -0.037061691, -0.029794693, -0.023012161, -0.017561913, -0.01441288, -0.012111664, -0.009325981, -0.007024765, -0.004602432, -0.002664566, 0.00090924, 0.00030358, 0, -0.000242233, -0.000837976, 0.001999305, 0.003937171, 0.006117271, 0.008176254, 0.011688636, 0.014830113, 0.020045692, 0.023566372, 0.028644966, 0.03312327, 0.039664506, 0.047781395, 0.052868293, 0.096530072, 0.151339196, 0.216332764, 0.295221736, 0.377891785, 0.491216024, 0.606462279, 0.877207832, 1.207061607, 1.563385753, 2.030351958, 2.444452733};
byte po_flux[] = {0, 10, 20, 30, 40, 50, 55, 60, 65, 70, 75, 80, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 100, 100, 100, 100, 100, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 120, 125, 130, 135, 140, 145, 150, 160, 170, 180, 190, 200};



int max_speed = 2000;
int max_accel = 2000;

//Encoder from https://www.instructables.com/id/Improved-Arduino-Rotary-Encoder-Reading/
int pinA = PIN_ENC_CL; // Our first hardware interrupt pin is digital pin 2
int pinB = PIN_ENC_DIR; // Our second hardware interrupt pin is digital pin 3
byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

byte max_sel, min_sel; //According to current selection


//
void PinA() {
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB() {
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

AutoPID * pid;

MechVentilation * ventilation;
VentilationOptions_t options;

//#ifdef LCD_I2C
//LiquidCrystal_I2C lcd;
//lcd = LiquidCrystal_I2C(0x3F, 20, 4);
//extern LiquidCrystal_I2C lcd;
//#else
////extern LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
//lcd=LiquidCrystal(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
////extern LiquidCrystal lcd;
//#endif

float p_dpt0;
void setup() {

  Serial.begin(250000);
  init_display();

  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, LOW); // test zumbador
  delay(500);
  digitalWrite(PIN_BUZZ, HIGH);
  //beep(100); //Beep
  //tone(PIN_BUZZ, 1000, 500);

  // PID
  pid = new AutoPID(PID_MIN, PID_MAX, PID_KP, PID_KI, PID_KD);
  // if pressure is more than PID_BANGBANG below or above setpoint,
  // output will be set to min or max respectively
  pid -> setBangBang(PID_BANGBANG);
  // set PID update interval
  pid -> setTimeStep(PID_TS);

  // Parte motor
  pinMode(PIN_EN, OUTPUT);
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

  //  Serial.println("Tiempo del ciclo (seg):" + String(ventilation -> getExsuflationTime() + ventilation -> getInsuflationTime()));
  //  Serial.println("Tiempo inspiratorio (mseg):" + String(ventilation -> getInsuflationTime()));
  //  Serial.println("Tiempo espiratorio (mseg):" + String(ventilation -> getExsuflationTime()));

  // TODO: Esperar aqui a iniciar el arranque desde el serial

  // Habilita el motor
  digitalWrite(PIN_EN, LOW);

  writeLine(1, "AMBOVIS 0529_1", 4);

  p_dpt0 = 0;
  ads.begin();
  verror = 0;
  float verrp = 0.;
  for (int i = 0; i < 10; i++) {
    adc0 = ads.readADC_SingleEnded(0);
    Voltage = (adc0 * 0.1875) * 0.001; //VOLT!
    Serial.print("Voltage dp: "); Serial.println(Voltage, 3);
    verror += ( Voltage - 5.0 * 0.04 ); //IF VOUT IS: Vo=VS(0.09*P0.04) +/- ERR
    //vo=vs(0.09 dp +0.04)+/-verr
    p_dpt0 += 0.5 * (( Voltage /* 5.0/V_SUPPLY_HONEY */ - 0.1 * 4.8/* - corr_fs */) / (0.8 * 4.8) * DEFAULT_PSI_TO_CM_H20 * 2. - DEFAULT_PSI_TO_CM_H20);
    verrp += (analogRead(A0) * 5. / 1024. - 5.*0.04);
    Serial.print("Voltage p: "); Serial.println(analogRead(A0));
    delay(10);
  }

  verror /= 10.; //
  verrp /= 10.;
  p_dpt0 /= 10.0;
  Serial.print("dp (Flux) MPX Volt (mV) at p0: "); Serial.println(verror * 1000, 3);
  Serial.print("pressure  MPX Volt (mV) at p0: "); Serial.println(verrp * 1000, 3);

  Serial.print("dp  error : "); Serial.println(-verror / (5.*0.09));
  p_dpt0 = 0.20;


  // configura la ventilación
  ventilation -> start();
  ventilation -> update();

  //sensors -> readPressure();

  display_lcd();

  //ENCODER
  curr_sel = old_curr_sel = 0; //COMPRESSION
  encoderPos = oldEncPos = options.tidalVolume;

  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0, PinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1, PinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  //btnState=digitalRead(9);

  lastReadSensor =   lastShowSensor = millis();
  lastState = ventilation->getState();
  last_update_display = millis();

#ifdef DEBUG_UPDATE

#endif

  //STEPPER
  last_stepper_time = millis();
  last_vent_time = millis();

  //Serial.print(",0,50");

  Timer1.initialize(50);
  Timer1.attachInterrupt(timer1Isr);

#ifdef DEBUG_UPDATE
  Serial.print("Honey Volt at p0: "); Serial.println(analogRead(A0) / 1023.);
#endif
  EEPROM.get(0, last_cycle);
  EEPROM.get(1, p_trim);
  EEPROM.get(2,autopid);
  Serial.print("LAST CYCLE: "); Serial.println(last_cycle);
  ventilation->setCycleNum(last_cycle);

}


bool update_display = false;
byte pos;

void loop() {

  State state = ventilation->getState();
  check_encoder();

  time = millis();

  if (millis() > lastSave + TIME_SAVE) {
    EEPROM.put(0, last_cycle);
    EEPROM.put(1, p_trim);
    EEPROM.put(2,autopid);
    lastSave = millis();
  }

#ifdef DEBUG_OFF
  if ( millis() > lastShowSensor + TIME_SHOW ) {
    lastShowSensor = millis();
    Serial.print(int(cycle_pos)); Serial.print(",");
    Serial.print(int(pressure_p)); Serial.print(",");
#ifdef FILTER_FLUX
    Serial.print(int(flow_f)); Serial.print(",");
#else
    Serial.print(int(_flux)); Serial.print(",");
#endif
    Serial.print(int(alarm_state)); Serial.print(",");
    Serial.print(int(_mlInsVol - _mlExsVol)); Serial.print(",");
    Serial.print(int(_mllastInsVol)); Serial.print(",");
    Serial.println(int(_mllastExsVol));

    //Serial.print(",");Serial.println(int(alarm_state));
    //      #ifdef FILTER_FLUX
    //      Serial.print(Voltage*1000);Serial.print(",");Serial.print(p_dpt,4);Serial.print(",");Serial.println(_flux);/*Serial.print(",");/*Serial.print(",");Serial.println(_flux_sum/5.);*/
    //      Serial.print("Vcorr");Serial.print(",");Serial.println((Voltage-verror));
    //      #endif
    //Serial.print(int(_mlInsVol));Serial.print(",");Serial.println(int(_mlExsVol));

  }
#endif

  if (time > lastReadSensor + TIME_SENSOR) {

    //pressure_p=( analogRead(A0)/(1023.) - 0.04 )/0.09*1000*DEFAULT_PA_TO_CM_H20*1.05;
    pressure_p = ( analogRead(A0) / (1023.) - verrp * 0.2 - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20 * 0.75 - 1.0;

    adc0 = ads.readADC_SingleEnded(0);
    Voltage = (adc0 * 0.1875) / 1000.; //Volts

    // p_dpt=( ( Voltage - verror)*0.2 /*-0.00001*/ - 0.04 )/0.09*1000*DEFAULT_PA_TO_CM_H20+(float(p_trim)-100.0)*1e-3;
    p_dpt = ( Voltage - verror  - 0.20 ) / 0.45 * 1000 * DEFAULT_PA_TO_CM_H20 + (float(p_trim) - 100.0) * 1e-3;

    update_error();

    pos = findClosest(dp, 55, p_dpt);
    //flux should be shifted up (byte storage issue)
    _flux = po_flux[pos] - 100 + ( float (po_flux[pos + 1] - 100) - float (po_flux[pos] - 100) ) * ( p_dpt - float(dp[pos]) ) / (float)( dp[pos + 1] - dp[pos]);
    _flux *= 16.6667;

#ifdef FILTER_FLUX
    flux_count++;    //Filter
    for (int i = 0; i < 4; i++) {
      _flux_fil[i] = _flux_fil[i + 1];
    }
    _flux_fil[4] = _flux;
    _flux_sum = 0.;
    for (int i = 0; i < 5; i++)
      _flux_sum += _flux_fil[i];
#endif

#ifdef FILTER_FLUX
    flow_f = _flux_sum / 5.;
#else
    flow_f = _flux;
#endif

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
  }//Read Sensor

  //enum _state {NO_ALARM=0,PEEP_ALARM=1,PIP_ALARM=2,PEEP_PIP_ALARM=3};
  if ( last_pressure_max > alarm_max_pressure + 1 ) {
    if ( last_pressure_min < alarm_peep_pressure - 1) {
      alarm_state = 3;
    } else {
      alarm_state = 2;
    }
  } else {
    if ( last_pressure_min < alarm_peep_pressure - 1 ) {
      alarm_state = 1;
    } else {
      alarm_state = 0;
    }
  }

  if ( ventilation -> getCycleNum () != last_cycle ) {
    last_cycle = ventilation->getCycleNum();
    //lcd.clear();  //display_lcd do not clear screnn in order to not blink
    display_lcd();
    update_display = true;
    last_update_display = millis();

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

  }

  if (display_needs_update) {
    //lcd.clear();  //display_lcd do not clear screnn in order to not blink
    display_lcd();
    display_needs_update = false;
  }

  if ( update_options ) {
    ventilation->change_config(options);
    update_options = false;
    //show_changed_options=true;
  }//

  if ( millis () - last_vent_time > TIME_BASE ) {
    ventilation -> update();
  }

  //HERE changed_options flag is not updating until cycle hcanges
  if (show_changed_options && ((millis() - last_update_display) > time_update_display) ) {
    display_lcd();  //WITHOUT CLEAR!
    last_update_display = millis();
    show_changed_options = false;
  }

}//LOOP

void timer1Isr(void)
{
  #ifdef ACCEL_STEPPER
    stepper->run();
  #else
    stepper -> processMovement(); //LUCIANO
    //Serial.print("Speed");Serial.println(_stepperSpeed);
  #endif
}

void update_error() {
  //UPDATING VERROR
  if (cycle_pos > 115) {
    if (vcorr_count < 20) {
      vcorr_count += 1;
      verror_sum += ( Voltage - 5.0 * 0.04 );
      //verror+=Voltage;
      init_verror = true;
    }
    //Serial.print("Verror (mV) and count: ");Serial.print(verror_sum*1000);Serial.print(",  ");Serial.println(vcorr_count);
  }
  if (cycle_pos < 10 && init_verror) {
    verror = verror_sum / ((float)vcorr_count + 1);
    //Serial.print("Verror (mV) and count: ");Serial.print(verror*1000);Serial.print(",  ");Serial.println(vcorr_count);
    verror_sum = 0.;
    vcorr_count = 0;
    init_verror = false;
  }
}

void timer2Isr(void)
{
  ventilation -> update();
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
