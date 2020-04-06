/**
   @file rees_firmware.ino
   @author Reesistencia Team
   @brief
   @version 0.1
   @date 2020-03-29

   @copyright GPLv3

*/

/**
   Dependencies
*/

//#define ACCEL_STEPPER 0

#include "defaults.h"
#include "pinout.h"
#include "calc.h"
#include "Sensors.h"
#include "MechVentilation.h"

#include "src/AutoPID/AutoPID.h"
#ifdef ACCEL_STEPPER
#include "src/AccelStepper/AccelStepper.h"
#else
#include "src/FlexyStepper/FlexyStepper.h"
#endif

//#include "src/TimerOne/TimerOne.h"
//#include "src/TimerThree/TimerThree.h"  //PARA PLC?

//LUCIANO -------
#include "LiquidCrystal_I2C.h"
#include "src/Pressure_Sensor/Pressure_Sensor.h"  //LUCIANO: MPX5050DP
//LUCIANO -------

//LUCIANO
Pressure_Sensor bmp1;
int Compression_perc = 80; //Similar to israeli

float flux, dp;
//LUCIANO --------

/**
   Variables
*/

//Rotary rotary(3,2);

#if DEBUG_STATE_MACHINE
volatile String debugMsg[15];
volatile byte debugMsgCounter = 0;
#endif

//TimerOne Timer2;

#ifdef ACCEL_STEPPER
AccelStepper *stepper = new AccelStepper(
  AccelStepper::DRIVER,
  PIN_STEPPER_DIRECTION,
  PIN_STEPPER_STEP);
#else
FlexyStepper * stepper = new FlexyStepper();
#endif

//Encoder encoder(
//  DTpin,
//  CLKpin,
//  SWpin
//);

//Encoder encoder(
//  2,
//  3,
//  7
//);
//
//
#define PIN_ENC_SW  9
#define PIN_ENC_CL  2
#define PIN_ENC_DIR 3

//
static int pinA = PIN_ENC_CL; // Our first hardware interrupt pin is digital pin 2
static int pinB = PIN_ENC_DIR; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent


void PinA(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB(){
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

Sensors * sensors;
AutoPID * pid;
MechVentilation * ventilation;

VentilationOptions_t options;

/**
   Read commands
*/


void readIncomingMsg (void) {
  char* msg = (char*)malloc(100);
  //Serial2.readStringUntil('\n').toCharArray(msg, 100);
  int pip, peep, fr;
  if (String(msg).substring(0, 6) == "CONFIG") {
    int rc = sscanf(msg, "CONFIG PIP %d", &pip);
    if (rc == 1) {
      ventilation->setPeakInspiratoryPressure(pip);
    } else {
      int rc = sscanf(msg, "CONFIG PEEP %d", &peep);
      if (rc == 1) {
        ventilation->setPeakEspiratoryPressure(peep);
      } else {
        int rc = sscanf(msg, "CONFIG BPM %d", &fr);
        if (rc == 1) {
          ventilation->setRPM(fr);
        }
      }
    }
  }
  else if (String(msg).substring(0, 7) == "RECRUIT")
  {
    uint8_t tmp = 255;
    int rc = sscanf(msg, "RECRUIT %d", &tmp);
    switch (tmp)
    {
      case 0:
        Serial.println("ACK 0");
        ventilation->deactivateRecruitment();
        break;
      case 1:
        Serial.println("ACK 1");
        ventilation->activateRecruitment();
        break;
      default:
        break;
    }
  }
  free(msg);
}

LiquidCrystal_I2C lcd(0x3F, 20, 4);
void writeLine(int line, String message = "", int offsetLeft = 0)
{
  lcd.setCursor(0, line);
  lcd.print("");
  lcd.setCursor(offsetLeft, line);
  lcd.print(message);
}


/**
   Setup
*/

void setup() {
  // Puertos serie
  Serial.begin(9600);
  //Serial2.begin(115200);
  Serial.println(F("Setup"));

  //LUCIANO-------------
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  //---------------------

  // Zumbador
  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, HIGH); // test zumbador
  delay(100);
  digitalWrite(PIN_BUZZ, LOW);

  // FC efecto hall
  pinMode(PIN_ENDSTOP, INPUT_PULLUP); // el sensor de efecto hall da un 1 cuando detecta

  // Solenoid
  pinMode(PIN_SOLENOID, OUTPUT);

  // Sensores de presión
  sensors = new Sensors();
  int check = sensors -> begin();
#if 0
  if (check) {
    if (check == 1) {
      Serial.println(F("Could not find sensor BME280 number 1, check wiring!"));
    } else if (check == 2) {
      Serial.println(F("Could not find sensor BME280 number 2, check wiring!"));
    }
    while (1);
  }
#endif

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

  options.height = DEFAULT_HEIGHT;
  options.sex = DEFAULT_SEX;
  options.respiratoryRate = DEFAULT_RPM;
  options.peakInspiratoryPressure = DEFAULT_PEAK_INSPIRATORY_PRESSURE;
  options.peakEspiratoryPressure = DEFAULT_PEAK_ESPIRATORY_PRESSURE;
  options.triggerThreshold = DEFAULT_TRIGGER_THRESHOLD;
  options.hasTrigger = false;

  ventilation = new MechVentilation(
    stepper,
    sensors,
    pid,
    options
  );

  Serial.println("Tiempo del ciclo (seg):" + String(ventilation -> getExsuflationTime() + ventilation -> getInsuflationTime()));
  Serial.println("Tiempo inspiratorio (mseg):" + String(ventilation -> getInsuflationTime()));
  Serial.println("Tiempo espiratorio (mseg):" + String(ventilation -> getExsuflationTime()));

  // TODO: Esperar aqui a iniciar el arranque desde el serial

  // Habilita el motor
  digitalWrite(PIN_EN, LOW);

  // configura la ventilación
  ventilation -> start();
  ventilation -> update();

  delay(1000);

  sensors -> readPressure();
  // TODO: Make this period dependant of TIME_BASE
  // TIME_BASE * 1000 does not work!!
  // Timer1.initialize(50000); // 50 ms
  //Timer1.initialize(20000);

  //Timer2.initialize(50000);

  //Timer1.attachInterrupt(timer1Isr);
  //TODO: Option: if (Sensores ok) { arranca timer3 }

  //
  //    Timer3.initialize(50); //50us
  //    Timer3.attachInterrupt(timer3Isr);

  //    //LUCIANO
  //Timer2.attachInterrupt(timer3Isr);
  bmp1 = Pressure_Sensor(A0);
  //--

//  menu();
}

/**
   Loop
*/
//
void loop() {
  //
  //LUCIANO @TODO: remove

  //  if (!encoder.readButton())
  //    menu();
  //
  dp = bmp1.get_dp();
  //Serial.print("Sensor value: ");Serial.println(bmp1.get_dp());
 //writeLine(0, "dV: " + String(analogRead(A0) * 5. / 1024.) + " V");
  //writeLine(1, "dp: " + String(bmp1.get_dp()) + " Pa");

  calcularCaudalVenturi(dp, &flux);
  //writeLine(2, "fv: " + String(flux * 1000) + " ml/s");
  //-----------LUCIANO

  unsigned long time;
  time = millis();
  unsigned long static lastReadSensor = 0;
  unsigned long static lastSendConfiguration = 0;
  State static lastState;

  if (time > lastSendConfiguration + TIME_SEND_CONFIGURATION)
  {
    Serial.print(F("CONFIG "));
    Serial.print(ventilation -> getPeakInspiratoryPressure());
    Serial.print(F(" "));
    Serial.print(ventilation -> getPeakEspiratoryPressure());
    Serial.print(F(" "));
    Serial.println(ventilation -> getRPM());
    lastSendConfiguration = time;
  }

  if (time > lastReadSensor + TIME_SENSOR)
  {
    sensors -> readPressure();
    SensorPressureValues_t pressure = sensors -> getRelativePressureInCmH20();

    sensors -> readVolume();
    SensorVolumeValue_t volume = sensors -> getVolume();
    //writeLine(1, "p: " + String((int)pressure.pressure1) + " cmH20");
    char* string = (char*)malloc(100);
    sprintf(string, "DT %05d %05d %05d %06d", ((int)pressure.pressure1), ((int)pressure.pressure2), volume.volume, ((int)(sensors->getFlow() * 1000)));

    //        Serial2.println(string);
    // Serial.println(string);
    free(string);

    if (pressure.state == SensorStateFailed) {
      //TODO sensor fail. do something
      Serial.println(F("FALLO Sensor"));
      // TODO: BUZZ ALARMS LIKE HELL
    }
    lastReadSensor = time;

    /*
       Notify insufflated volume
    */
    if (ventilation->getState() != lastState)
    {
      SensorLastPressure_t lastPressure = sensors->getLastPressure();
      if (ventilation->getState() == Init_Exsufflation)
      {
        //                Serial2.println("EOC " + String(lastPressure.maxPressure) + " " +
        //                    String(lastPressure.minPressure) + " " + String(volume.volume));
      }
      else if (ventilation->getState() == State_Exsufflation)
      {
        if (lastState != Init_Exsufflation)
        {
          //                    Serial2.println("EOC " + String(lastPressure.maxPressure) + " " +
          //                    String(lastPressure.minPressure) + " " + String(volume.volume));
        }
      }
    }
    lastState = ventilation->getState();
  }
  //
  //    if (Serial2.available()) {
  //        readIncomingMsg();
  //    }

#if DEBUG_STATE_MACHINE
  if (debugMsgCounter) {
    for (byte i = 0; i < debugMsgCounter; i++) {
      Serial.println(debugMsg[i]);
    }
    debugMsgCounter = 0;
  }
#endif

  //LUCIANO----------------------
  ventilation -> update();
  //stepper->setSpeedInStepsPerSecond(600);
  //stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
  stepper -> processMovement(); //LUCIANO

  ///////////--- LUCIANO
}

unsigned long lastButtonPress;
int curr_sel,old_curr_sel;

////LUCIANO
void check_encoder()
{
      //LUCIANO------------------------
byte btnState = digitalRead(9);

if (btnState == LOW) {
    if (millis() - lastButtonPress > 50) {
        Serial.println(curr_sel);
        curr_sel++; //NOT +=1, is a byte
        if (curr_sel>2)
          curr_sel=0;
    }
    lastButtonPress = millis();
}


    if(oldEncPos != encoderPos) {
      Serial.println(encoderPos);
      oldEncPos = encoderPos;
      switch (curr_sel){
        case 0:
          //A_comp=encoderPos;
          break;
        case 1:
          options.respiratoryRate;
          break;
        case 2:
          //A_pres=encoderPos;
          break;        
              }}

if (curr_sel!=old_curr_sel){
 switch (curr_sel){
  case 0:
//    encoderPos=oldEncPos=A_comp;
    break;
  case 1:
    encoderPos=oldEncPos=options.respiratoryRate;
    break;
  case 2:
//    encoderPos=oldEncPos=A_pres;
    break;
  }
  old_curr_sel=curr_sel;}
 //----
}
