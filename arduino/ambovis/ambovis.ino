#include "defaults.h"
#include "pinout.h"
#include "MechVentilation.h"
#include "src/TimerOne/TimerOne.h"
#include "src/TimerTwo/TimerTwo.h"

#include "src/AutoPID/AutoPID.h"
#ifdef ACCEL_STEPPER
#include "src/AccelStepper/AccelStepper.h"
#else
#include "src/FlexyStepper/FlexyStepper.h"
#endif

#ifdef LCD_I2C
#include "LiquidCrystal_I2C.h"
#else
#include <LiquidCrystal.h>
#endif
#include "src/Pressure_Sensor/Pressure_Sensor.h"  //LUCIANO: MPX5050DP

int pressure_flux;  //For calculating flux
float _mlInsVol=0,_mllastInsVol;
//float _mlInsVol2;

int Compression_perc = 8; //80%

#if DEBUG_STATE_MACHINE
volatile String debugMsg[15];
volatile byte debugMsgCounter = 0;
#endif

#ifdef ACCEL_STEPPER
AccelStepper *stepper = new AccelStepper(
  //AccelStepper::DRIVER,
  1,
  PIN_STEPPER_DIRECTION,
  PIN_STEPPER_STEP);
#else
FlexyStepper * stepper = new FlexyStepper();
#endif

//////////////////////////
// - EXTERNAL VARIABLES //
//////////////////////////
float pressure_p;   //EXTERN!!
float last_pressure_max,last_pressure_min;

byte vent_mode = VENTMODE_MAN; //0
//Adafruit_BMP280 _pres1Sensor;
Pressure_Sensor _dpsensor;
float pressure_p0;
float _flux,_flux_0;
//float _stepperSpeed;
bool send_data=false;
byte time_encoder=50;
char tempstr[5],tempstr2[5];
unsigned long lastButtonPress;
int curr_sel, old_curr_sel;
float _currentPressure = 0.0;
float p_dpt;

unsigned long lastReadSensor = 0;
unsigned long lastShowSensor = 0;
bool display_needs_update=false;
byte flux_count;
unsigned long flux_filter_time;


State static lastState;
bool show_changed_options = false; //Only for display
bool update_options = false;

unsigned long time_update_display = 20; //ms
unsigned long last_update_display;

extern float _mlInsVol,_mllastInsVol;
extern byte stepper_time = 50;
unsigned long last_stepper_time;
unsigned long last_vent_time;
unsigned long time;
float flux_sum;


//float dp_neg[]={-0.877207832,-0.606462279,-0.491216024,-0.377891785,-0.295221736,-0.216332764,-0.151339196,-0.096530072,-0.052868293,-0.047781395,-0.039664506,-0.03312327,-0.028644966,-0.023566372,-0.020045692,-0.014830113,-0.011688636,-0.008176254,-0.006117271,-0.003937171,-0.001999305,-0.00090924,-0.00030358,0};
float dp_pos[]={0.,0.000242233,0.000837976,0.002664566,0.004602432,0.007024765,0.009325981,0.012111664,0.01441288,0.017561913,0.023012161,0.029794693,0.037061691,0.043771552,0.051474571,0.05874157,0.109004974,0.176879848,0.260808033,0.365700986,0.504544509,0.630753349,0.795599072,1.216013465,1.60054669,2.087678384,2.547210457,3.074176245,3.676588011,4.385391541,5.220403813,5.947168311,6.794489065,7.662011691,8.642594913,9.810447693,10.7793808,11.95257389};
//byte po_flux_neg[]={200,190,180,170,160,150,140,130,120,110,100,90,80,70,60,50,45,40,35,30,25,20,15,14,13,12,11,10,9,8,7,6,5,4,3,0,0};  //Negative values!
//byte po_flux_pos[]={0,0,0,3,4,5,6,7,8,9,10,11,12,13,14,15,20,25,30,35,40,45,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200};
//byte po_flux_neg[]={-1000,-833.3333333,-750,-666.6666667,-583.3333333,-500,-416.6666667,-333.3333333,-250,-233.3333333,-216.6666667,-200,-183.3333333,-166.6666667,-150,-133.3333333,-116.6666667,-100,-83.33333333,-66.66666667,-50,0,0,0};
byte po_flux_pos[]={0,0,0,50,66.66666667,83.33333333,100,116.6666667,133.3333333,150,166.6666667,183.3333333,200,216.6666667,233.3333333,250,333.3333333,416.6666667,500,583.3333333,666.6666667,750,833.3333333,1000};


int max_speed=2000;
int max_accel=2000;

//Encoder from https://www.instructables.com/id/Improved-Arduino-Rotary-Encoder-Reading/
int pinA = PIN_ENC_CL; // Our first hardware interrupt pin is digital pin 2
int pinB = PIN_ENC_DIR; // Our second hardware interrupt pin is digital pin 3
byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

byte max_sel,min_sel; //According to current selection

void display_lcd ( );
float f_dpt,corr_dpt;
float f1_honey,f2_honey;

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

#ifdef LCD_I2C
LiquidCrystal_I2C lcd(0x3F, 20, 4);
#else
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
#endif

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

void setup() {
  Serial.begin(115200);

#ifdef LCD_I2C
  lcd.begin();  //I2C
#else
  lcd.begin(20, 4); //NO I2C
#endif
  //lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);

  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, HIGH); // test zumbador
  delay(100);
  digitalWrite(PIN_BUZZ, LOW);

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
  options.percInspEsp=3;//1:1 to 1:4, is denom
  //options.peakInspiratoryPressure = DEFAULT_PEAK_INSPIRATORY_PRESSURE;
  options.peakInspiratoryPressure = 25.;
  options.peakEspiratoryPressure = DEFAULT_PEAK_ESPIRATORY_PRESSURE;
  options.triggerThreshold = DEFAULT_TRIGGER_THRESHOLD;
  options.hasTrigger = false;
  options.tidalVolume = 300;
  options.percVolume= 80;  //1 to 10

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

  writeLine(1, "AMBOVIS 0305_v1",4);

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

  //MAKE AN IF IF_2_PRESS_SENSORS
  //_pres1Sensor.begin();
  //pressure_p0 = _pres1Sensor.readPressure() * DEFAULT_PA_TO_CM_H20;
  //pressure_p0 = 103100.0* DEFAULT_PA_TO_CM_H20;
  #ifdef DEBUG_UPDATE
    Serial.print("Pressure_p0");Serial.print(pressure_p0);
  #endif

  //calcularCaudalVenturi(_dpsensor.get_dp(), &_flux_0);

  //STEPPER
  last_stepper_time = millis();
  last_vent_time = millis();

  //Serial.print(",0,50");

  Timer1.initialize(50);
  Timer1.attachInterrupt(timer1Isr);

  f_dpt=(float)RANGE_DPT*DEFAULT_PA_TO_CM_H20/1023.;
  //corr_dpt=RANGE_DPT*DEFAULT_PA_TO_CM_H20/2.;
  f1_honey=5.0*DEFAULT_PSI_TO_CM_H20*2/(1023*0.8*V_SUPPLY_HONEY);
  
  //#ifdef DEBUG_UPDATE
    //Serial.print("Honey Volt at p0: ");Serial.println(analogRead(A0)/1023.);
  //#endif

}

/**
   Loop
*/
//
byte last_cycle = 0;
bool update_display = false;
char string[100];
byte pos;
void loop() {

  check_encoder();

  time = millis();

//    
    #ifdef DEBUG_OFF
    if (millis() > lastShowSensor + TIME_SHOW) {
      /*Serial.print(millis());Serial.print(" ");*/Serial.print(int(pressure_p));Serial.print(" ");Serial.print(int(_flux));Serial.print(" ");Serial.println(int(_mlInsVol));
      lastShowSensor=millis();
    }
    #endif
    
  if (time > lastReadSensor + TIME_SENSOR){

    //A0: PRESSURE (HOEYWELL) A1: Volume (DPT) A2: Test Mode pressure (DPT)
    //0.1 is from the 0.5 readed initially by the honeywell
    p_dpt    =f_dpt*float(analogRead(A1)); //ONE DIRECTION
    //p_dpt      = f_dpt*float(analogRead(A1)) - corr_dpt;
    pressure_p = (( float ( analogRead(A0) )/1023.) * 5.0/V_SUPPLY_HONEY  - 0.1 + (V_HONEY_P0-0.5))/0.8*DEFAULT_PSI_TO_CM_H20*2.-DEFAULT_PSI_TO_CM_H20; //Data sheet figure 2 analog pressure, calibration from 10% to 90%

    //pos=findClosest(dp_pos,38,p_dpt);
//    if ( p_dpt > 0 ) {
      pos=findClosest(dp_pos,24,p_dpt);
      _flux = po_flux_pos[pos] + ( po_flux_pos[pos+1] - po_flux_pos[pos] ) * ( p_dpt - dp_pos[pos] ) / ( dp_pos[pos+1] - dp_pos[pos]);
//    } else {
//        pos=findClosest(dp_neg,24,p_dpt);
//        _flux = po_flux_neg[pos] + ( po_flux_neg[pos+1] - po_flux_neg[pos] ) * ( p_dpt - dp_neg[pos] ) / ( dp_neg[pos+1] - dp_neg[pos]);      
//      }
    flux_count++;    //Filter

//    #ifdef DEBUG_OFF
//      Serial.print(millis());Serial.print(" ");Serial.print(int(pressure_p));Serial.print(" ");Serial.print(int(_flux));Serial.print(" ");Serial.println(int(_mlInsVol));
//    #endif

    #ifdef DEBUG_UPDATE
    //Serial.print(millis()-_msecTimerStartCycle);Serial.print(" ");Serial.print(_flux);Serial.print(" ");Serial.println(_mlInsVol); //Flux
    #endif

    lastReadSensor = millis();

      //CHECK PIP AND PEEP (OUTSIDE ANY CYCLE!!)
      if (pressure_p>pressure_max) {
            pressure_max=pressure_p;
       }
      if (pressure_p < pressure_min){
          pressure_min=pressure_p;
      }
  }//Read Sensor


  /////////////////////// ORIG ///////////////////

//      if (ventilation->getCycleNum() != last_cycle)
//      update_display = false;
//    State state = ventilation->getState();
//    if (!update_display)
//      if (ventilation->getCycleNum() != last_cycle && state == State_Exsufflation) {
//        //Serial.print("Insuflated Vol: "); Serial.println(ventilation->getInsVol());
//        lcd.clear();  //display_lcd do not clear screnn in order to not blink
//        display_lcd();
//        update_display = true;
//        last_cycle = ventilation->getCycleNum();
//        last_update_display = millis();
//
//        if (update_options) { //Changed options applies when cycle changed
//          ventilation->change_config(options);
//          update_options = false;
//        }
//      }
      //////////////////////////////////////
    State state = ventilation->getState();
    if ( ventilation -> getCycleNum () != last_cycle ) {
        last_cycle = ventilation->getCycleNum(); 
        lcd.clear();  //display_lcd do not clear screnn in order to not blink
        display_lcd();
        update_display = true;
        last_update_display = millis();
    }

    if (display_needs_update) {
        lcd.clear();  //display_lcd do not clear screnn in order to not blink
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
      show_changed_options=false;
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

void timer2Isr(void)
{
  ventilation -> update();
}

//

void check_encoder()
{
  //LUCIANO------------------------
  byte btnState = digitalRead(PIN_ENC_SW);
  //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6) 
  if (btnState == LOW) {
    if (millis() - lastButtonPress > 200) {
      //Serial.println(curr_sel);
      //Clean all marks
      
      curr_sel++; //NOT +=1, is a byte

      //if ((vent_mode==VENTMODE_VCL || vent_mode==VENTMODE_MAN) && curr_sel==5) curr_sel++; //Not selecting pip in VCL
      if (vent_mode==VENTMODE_PCL && curr_sel==4) curr_sel++; //Not selecting pip in VCL 
            
      if (curr_sel > 5)
        curr_sel = 0;
      switch (curr_sel){
        case 1: 
          min_sel=0;max_sel=2;
          encoderPos=oldEncPos=vent_mode;
        break;
        case 2: 
          encoderPos=oldEncPos=options.respiratoryRate;
          min_sel=DEFAULT_MIN_RPM;max_sel=DEFAULT_MAX_RPM;
        break;
        case 3:
          encoderPos=oldEncPos=options.percInspEsp;
          min_sel=1;max_sel=4;        
        break;
        case 4: 
          if ( vent_mode==VENTMODE_VCL || vent_mode==VENTMODE_PCL){
            encoderPos=oldEncPos=options.tidalVolume;
            min_sel=200;max_sel=800;
          } else {//Manual
            encoderPos=oldEncPos=options.percVolume;
//            Serial.print("Encoder pos: ");Serial.println(encoderPos);
            min_sel=40;max_sel=100;            
          }
        break;
        case 5: 
          encoderPos=oldEncPos=options.peakInspiratoryPressure;
          min_sel=10;max_sel=40;
        break;
        case 6: 
          encoderPos=oldEncPos=options.peakEspiratoryPressure;
          min_sel=5;max_sel=20;
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
            vent_mode = encoderPos;
            break;
          case 2:
            options.respiratoryRate = encoderPos;
            break;
          case 3:
            options.percInspEsp=encoderPos;
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

  dtostrf(ventilation->getInsVol(), 4, 0, tempstr);
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
    
  writeLine(3, "PEEP: -", 8);
  dtostrf(last_pressure_min, 2, 1, tempstr);
  writeLine(3, String(tempstr), 16);  
 


  lcd_clearxy(0,0);
  lcd_clearxy(0,1);lcd_clearxy(9,1);
  lcd_clearxy(0,2);lcd_clearxy(7,2);
  lcd_clearxy(7,3);
  
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
        case 6: 
          lcd_selxy(7,3);break;
    }

}

int findClosest(float arr[], int n, float target) { 
    int i = 0, j = n-1, mid = 0; 
    while ( j - i > 1 ) { 
        mid = (i + j) / 2;  
        if (target < arr[mid]) { 
            j = mid; 
        } else {       // If target is greater than mid 
            i = mid;  } 
        //Serial.print("i,j: ");Serial.print(i);Serial.print(",");Serial.println(j);
        } 
    return i;
} 
