
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

#ifdef LCD_I2C
LiquidCrystal_I2C lcd(0x3F, 20, 4);
#else
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
#endif

/**
   Setup
*/

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

//  float a=dp[0];
  // Puertos serie
  Serial.begin(115200);
  //Serial2.begin(115200);
  //Serial.println(F("Setup"));

//  flux_neg[0]=1.;

#ifdef LCD_I2C
  lcd.begin();  //I2C
#else
  lcd.begin(20, 4); //NO I2C
#endif
  //lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  //---------------------
  //
  //  // Zumbador
  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, HIGH); // test zumbador
  delay(100);
  digitalWrite(PIN_BUZZ, LOW);
  //
  //  // FC efecto hall
  //  pinMode(PIN_ENDSTOP, INPUT_PULLUP); // el sensor de efecto hall da un 1 cuando detecta
  //
  //  // Solenoid
  //  pinMode(PIN_SOLENOID, OUTPUT);

  // Sensores de presión
  sensors = new Sensors();

  int check = sensors -> begin();
  if (check) {
    if (check == 1) {
      writeLine("Sensor BMP ERROR");
      writeLine("Error BMP280");
      Serial.println(F("Could not find sensor BME280 number 1, check wiring!"));
    } else if (check == 2) {
      Serial.println(F("Could not find sensor BME280 number 2, check wiring!"));
    }
    delay(500);
    sensors -> begin();
    while (1);
  }

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
  options.peakInspiratoryPressure = DEFAULT_PEAK_INSPIRATORY_PRESSURE;
  options.peakEspiratoryPressure = DEFAULT_PEAK_ESPIRATORY_PRESSURE;
  options.triggerThreshold = DEFAULT_TRIGGER_THRESHOLD;
  options.hasTrigger = false;
  options.tidalVolume = 300;
  options.percVolume= 8;  //1 to 10

  ventilation = new MechVentilation(
    stepper,
    sensors,
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

  writeLine(1, "AMBOVIS 0423_v1",1);

  // configura la ventilación
  ventilation -> start();
  ventilation -> update();

  sensors -> readPressure();

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

  lastReadSensor = millis();
  lastState = ventilation->getState();
  last_update_display = millis();

  //MAKE AN IF IF_2_PRESS_SENSORS
  pressure_p0 = _pres1Sensor.readPressure() * DEFAULT_PA_TO_CM_H20;
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
}

/**
   Loop
*/
//
byte last_cycle = 0;
bool update_display = false;
float ptest;
char string[100];
void loop() {

  check_encoder();

  time = millis();
  //  unsigned long static lastSendConfiguration = 0;
  //
  //  if (time > lastSendConfiguration + TIME_SEND_CONFIGURATION)
  //  {
  //    Serial.print(F("CONFIG "));
  //    Serial.print(ventilation -> getPeakInspiratoryPressure());
  //    Serial.print(F(" "));
  //    Serial.print(ventilation -> getPeakEspiratoryPressure());
  //    Serial.print(F(" "));
  //    Serial.println(ventilation -> getRPM());
  //    lastSendConfiguration = time;
  //  }
  //
  if (time > lastReadSensor + TIME_SENSOR)
  {
    //    //Is not anymore in classes
    //Serial.print("PRessure");Serial.println(pressure_p);

    ////////////////////////////// MOTOR RUNNING MAKING NOISE //////////////////////////////
    //    sensors -> readPressure();
    //    SensorPressureValues_t pressure = sensors -> getRelativePressureInCmH20();
    //
    // A0: PRESSURE (HOEYWELL)
    //A1: Volume (DPT)
    //A2: Test Mode pressure (DPT)
    //ptest   =float(analogRead(A2))*25.49/1024.; //From DPT, AS MAX RANGE, for testing
    //p_honey    =float(analogRead(A1))*1.01972/1024.; //From DPT, AS MAX RANGE
    
    #ifdef DEBUG_UPDATE
      Serial.print("Honey Volt at p0: ");Serial.println(analogRead(A0)/1023.);
    #endif
    //0.42 is level (0 to 1) of zero dp
    //0.1 is a correction
    //p_honey = (( float ( analogRead(A0) )/1023.- 0.51) * 5.0/V_SUPPLY_HONEY  - 0.1)/0.8*DEFAULT_PSI_TO_CM_H20*2.; //Data sheet figure 2 analog pressure, calibration from 10% to 90%
    p_honey = (( float ( analogRead(A0) )/1023.) * 5.0/V_SUPPLY_HONEY  - 0.1 + (V_HONEY_P0-0.5))/0.8*DEFAULT_PSI_TO_CM_H20*2.-DEFAULT_PSI_TO_CM_H20; //Data sheet figure 2 analog pressure, calibration from 10% to 90%
    p_bmp = _pres1Sensor.readPressure() * DEFAULT_PA_TO_CM_H20 - pressure_p0;
    
    #ifdef P_HONEYWELL
      pressure_p = p_honey;
    #else
      pressure_p = p_bmp;
    #endif
    
//    if (p_honey<0)
//      _flux=1000./60.*(1.005747e-1*pow(p_honey,4) + 2.247666*pow(p_honey,3) + 1.760981e+1*(p_honey,2) + 7.057159E+1*p_honey - 8.168219E+00);
//    else
//      _flux=1000./60.*(-3.779710E-02*pow(p_honey,4) + 1.046894E+00*pow(p_honey,3) - 1.029272E+01*pow(p_honey,2) + 5.379200E+01*p_honey + 8.455071E+00);
//    
//    _flux-=150.;
    //Serial.print("Flujo: "); Serial.print(_flux);Serial.println(" ");
    
    #ifdef DEBUG_OFF
      Serial.print(p_bmp);Serial.print(" ");Serial.print(p_honey);Serial.print(" ");/*Serial.print(ptest);Serial.print(" ");*/Serial.println(_flux);
      //Serial.print(int(p_bmp));Serial.print(" ");Serial.print(int(p_honey));Serial.print(" ");Serial.print(int(ptest));Serial.print(" ");Serial.println(int(_flux);
      //sprintf(string, "%f %f",(float)( pressure_p - pressure_p0), temp);
//      Serial.print(pressure_p - pressure_p0);Serial.print(" ");Serial.println(temp);
//     Serial.println(byte(_mlInsVol));
    #endif
        
    //Serial.print(",");Serial.println(sensors->getFlow());
    //    Serial.print("Flow: ");Serial.println(sensors->getFlow());
    //    SensorVolumeValue_t volume = sensors -> getVolume();
    //writeLine(1, "p: " + String((int)pressure.pressure1) + " cmH20");
    //    char* string = (char*)malloc(100);
    //    //sprintf(string, "DT %05d %05d %05d %06d", ((int)pressure.pressure1), ((int)pressure.pressure2), volume.volume, ((int)(sensors->getFlow() * 1000)));
    //    //Serial.println("Insuflated: "+String(ventilation->getInsVol()));
    //
    //    //        Serial2.println(string);
    //    //Serial.println(string);
    //    //free(string);

    ////////////////////////////// MOTOR RUNNING MAKING NOISE //////////////////////////////


    //    if (pressure.state == SensorStateFailed) {
    //      //TODO sensor fail. do something
    //      //Serial.println(F("FALLO Sensor"));
    //      // TODO: BUZZ ALARMS LIKE HELL
    //    }
    lastReadSensor = millis();

    /*
       Notify insufflated volume
    */
    if (ventilation->getCycleNum() != last_cycle)
      update_display = false;
    State state = ventilation->getState();
    if (!update_display)
      if (ventilation->getCycleNum() != last_cycle && state == State_Exsufflation) {
        //Serial.print("Insuflated Vol: "); Serial.println(ventilation->getInsVol());
        lcd.clear();  //display_lcd do not clear screnn in order to not blink
        display_lcd();
        update_display = true;
        last_cycle = ventilation->getCycleNum();
        last_update_display = millis();

        if (update_options) { //Changed options applies when cycle changed
          ventilation->change_config(options);
          update_options = false;
        }
      }
  }//Read Sensor

  //  //    if (Serial2.available()) {
  //  //        readIncomingMsg();
  //  //    }
#if DEBUG_STATE_MACHINE
  if (debugMsgCounter) {
    for (byte i = 0; i < debugMsgCounter; i++) {
      Serial.println(debugMsg[i]);
    }
    debugMsgCounter = 0;
  }
#endif

  //LUCIANO----------------------
  if ( millis () - last_vent_time > TIME_BASE ) {
    ventilation -> update();
    last_vent_time = millis();
  }

  //HERE changed_options flag is not updating until cycle hcanges
  if ( show_changed_options && ((millis() - last_update_display) > time_update_display) ) {
      display_lcd();  //WITHOUT CLEAR!
      last_update_display = millis();
      show_changed_options=false;
    }
  

}

void timer1Isr(void)
{
    #ifdef ACCEL_STEPPER
    stepper->run();
    #else
    stepper -> processMovement(); //LUCIANO
    //Serial.print("Speed");Serial.println(_stepperSpeed);
    #endif
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

      if ((vent_mode==VENTMODE_VCL || vent_mode==VENTMODE_MAN) && curr_sel==5) curr_sel++; //Not selecting pip in VCL
      if (vent_mode==VENTMODE_PCL && curr_sel==4) curr_sel++; //Not selecting pip in VCL 
            
      if (curr_sel > 6)
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
            min_sel=DEFAULT_MIN_VOLUMEN_TIDAL;max_sel=DEFAULT_MAX_VOLUMEN_TIDAL;
          } else {//Manual
            encoderPos=oldEncPos=options.percVolume;
//            Serial.print("Encoder pos: ");Serial.println(encoderPos);
            min_sel=1;max_sel=10;            
          }
        break;
        case 5: 
          encoderPos=oldEncPos=options.peakInspiratoryPressure;
          min_sel=20;max_sel=40;
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
  }


  if (oldEncPos != encoderPos) {

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
            break;
          case 4:
            if ( vent_mode==VENTMODE_VCL || vent_mode==VENTMODE_PCL)
              options.tidalVolume = encoderPos;
            else{ //manual
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
      writeLine(0, "MOD:VCL", 1); 
      writeLine(1, "V:" + String(options.tidalVolume), 10);    
      writeLine(2, "PIP : - ", 8);
    break;
    case VENTMODE_PCL:
      writeLine(0, "MOD:PCL", 1); 
      writeLine(2, "PIP :" + String(options.peakInspiratoryPressure), 8);
      writeLine(1, "V: - ", 10);
    break;    
    case VENTMODE_MAN:
      writeLine(0, "MOD:MAN", 1); 
      writeLine(2, "PIP : -", 8);
      writeLine(1, "V:" + String(options.percVolume*10)+"%", 10);    
    break;
  }
   
    
  writeLine(0, "SET | ME", 11);
  writeLine(1, "BPM:" + String(options.respiratoryRate), 1);
  writeLine(2, "IE:1:", 1);

  dtostrf(ventilation->getInsVol(), 4, 0, tempstr);
  //writeLine(1, String(tempstr), 15);
  writeLine(1, "---", 16);

  writeLine(2, String(options.percInspEsp), 6);
  
  dtostrf(pressure_max, 2, 0, tempstr);
  writeLine(2, String(tempstr), 16);  
  
  #ifdef DEBUG_UPDATE
  Serial.print("Max press conv: ");Serial.println(tempstr);
    Serial.print("Min Max press");  Serial.print(pressure_min);Serial.print(" ");Serial.println(pressure_max);
  #endif
    
  writeLine(3, "PEEP:" + String(options.peakEspiratoryPressure), 8);
  dtostrf(pressure_min, 2, 0, tempstr);
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