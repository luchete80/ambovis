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
#else
#include "src/FlexyStepper/FlexyStepper.h"
#endif

//#include "src/Pressure_Sensor/Pressure_Sensor.h"  //LUCIANO: MPX5050DP
#include <EEPROM.h>

bool init_verror;
byte Cdyn;
bool autopid;
bool filter;
unsigned long print_bat_time;

// FOR ADS
#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads(0x48);           //Conversor AD para leer mejor el flujo a partir de la presion

//TODO extract to struct for mechVentilation
float Voltage = 0.0;
float _mlInsVol = 0;
float _mlExsVol = 0;
int _mllastInsVol, _mllastExsVol;

void read_memory(); //Lee la EEPROM, usa variables externas, quiza deberian englobarse en un vector dinamico todos los offsets
void write_memory();

#ifdef ACCEL_STEPPER
AccelStepper *stepper = new AccelStepper(
  AccelStepper::DRIVER,
  PIN_STEPPER_STEP,
  PIN_STEPPER_DIRECTION);
#else
FlexyStepper * stepper = new FlexyStepper();
#endif

byte alarm_state = 0; //0: No alarm 1: peep 2: pip 3:both

// Init LCD
#ifdef LCD_I2C
LiquidCrystal_I2C lcd(0x3F, 20, 4);
#else
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
#endif

//MUTE : mute alarm buzzer configuration
unsigned long mute_count;
boolean last_mute;
boolean buzzmuted;
unsigned long timebuzz=0;
bool isbuzzeron=false;

// Init tft
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

MenuState menuState;
SystemState systemState;
SensorParams sensorParams;

byte vcorr_count;

unsigned long lastReadSensor = 0;
unsigned long lastShowSensor = 0;
unsigned long lastSave = 0;

unsigned long time_update_display = 20; //ms TODO this is a constant
unsigned long last_update_display;

extern float _mlInsVol, _mlExsVol;
unsigned long last_vent_time;
unsigned long time;
byte cycle_pos;
//int16_t adc0;

int max_accel,min_accel;
int max_speed, min_speed;
int min_pidk,max_pidk;
int min_pidi,max_pidi;
int min_pidd,max_pidd;
byte pfmin,pfmax;
float pf_min,pf_max;
float peep_fac;

int min_cd,max_cd;

unsigned long last_cycle;

unsigned int _timeoutIns,_timeoutEsp; //In ms

//TODO: READ FROM EEPROM
byte alarm_max_pressure = 35;
byte alarm_peep_pressure = 5;
byte isalarmvt_on;
int alarm_vt = 200;

//MENU
float verror_sum;

//FLUX IS -100 to +100, has to be added 100
//ASSIMETRY IN MAX FLOW IS IN NEGATIVE (ORIGINAL CURVE)
//float dp[]={-2.444452733,-2.030351958,-1.563385753,-1.207061607,-0.877207832,-0.606462279,-0.491216024,-0.377891785,-0.295221736,-0.216332764,-0.151339196,-0.096530072,-0.052868293,-0.047781395,-0.039664506,-0.03312327,-0.028644966,-0.023566372,-0.020045692,-0.014830113,-0.011688636,-0.008176254,-0.006117271,-0.003937171,-0.001999305,-0.00090924,-0.00030358,0,0.000242233,0.000837976,0.002664566,0.004602432,0.007024765,0.009325981,0.012111664,0.01441288,0.017561913,0.023012161,0.029794693,0.037061691,0.043771552,0.051474571,0.05874157,0.109004974,0.176879848,0.260808033,0.365700986,0.504544509,0.630753349,0.795599072,1.216013465,1.60054669,2.087678384,2.547210457,3.074176245};
//byte po_flux[]={0,10,20,30,40,50,55,60,65,70,75,80,85,86,87,88,89,90,91,92,93,94,95,96,97,100,100,100,100,100,103,104,105,106,107,108,109,110,111,112,113,114,115,120,125,130,135,140,145,150,160,170,180,190,200};

//MAX FLUX IS IN ISPIRING POSITIVE (1st quad)
float dp[] = { -3.074176245, -2.547210457, -2.087678384, -1.60054669, -1.216013465, -0.795599072, -0.630753349, -0.504544509, -0.365700986, -0.260808033, -0.176879848, -0.109004974, -0.05874157, -0.051474571, -0.043771552, -0.037061691, -0.029794693, -0.023012161, -0.017561913, -0.01441288, -0.012111664, -0.009325981, -0.007024765, -0.004602432, -0.002664566, 0.00090924, 0.00030358, 0, -0.000242233, -0.000837976, 0.001999305, 0.003937171, 0.006117271, 0.008176254, 0.011688636, 0.014830113, 0.020045692, 0.023566372, 0.028644966, 0.03312327, 0.039664506, 0.047781395, 0.052868293, 0.096530072, 0.151339196, 0.216332764, 0.295221736, 0.377891785, 0.491216024, 0.606462279, 0.877207832, 1.207061607, 1.563385753, 2.030351958, 2.444452733};
byte po_flux[] = {0, 10, 20, 30, 40, 50, 55, 60, 65, 70, 75, 80, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 100, 100, 100, 100, 100, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 120, 125, 130, 135, 140, 145, 150, 160, 170, 180, 190, 200};
bool change_pid_params = false;
void check_buzzer_mute();

AutoPID * pid;
MechVentilation * ventilation;
VentilationOptions_t options;

//Shared with Mech and Menu
float f_acc;
byte f_acc_b;
byte  p_acc;
float dpip;
byte dpip_b;

void setup() {
    Serial.begin(115200);
    init_display();

    pinMode(TFT_SLEEP, OUTPUT); //Set buzzerPin as output
    digitalWrite(TFT_SLEEP,HIGH); //LOW, INVERTED
    
    pinMode(PIN_BUZZER, OUTPUT); //Set buzzerPin as output
    pinMode(GREEN_LED,  OUTPUT); //Set buzzerPin as output
    pinMode(BCK_LED,    OUTPUT); //Set buzzerPin as output
    pinMode(YELLOW_LED, OUTPUT); //Set buzzerPin as output
    pinMode(RED_LED, OUTPUT); //Set buzzerPin as output

    digitalWrite(PIN_BUZZER,BUZZER_LOW); //LOW, INVERTED
        
    // PID
    pid = new AutoPID(PID_MIN, PID_MAX, PID_KP, PID_KI, PID_KD);
    // if pressure is more than PID_BANGBANG below or above setpoint,
    // output will be set to min or max respectively
    pid -> setBangBang(PID_BANGBANG);
    // set PID update interval
    pid -> setTimeStep(PID_TS);

    max_cd=40;  //T MODIFY: READ FROM MEM
    min_cd=10;
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

    // Habilita el motor
    digitalWrite(PIN_EN, LOW);

    writeLine(1, "RespirAR FIUBA", 4);
    writeLine(2, "v1.1.1", 8);
  
    ads.begin();

    Serial.print("dp  error : "); Serial.println(- sensorParams.verror / (5.*0.09));

    // Configure menu
    menuState.p_trim = 100;
    menuState.update_options = false;
    menuState.show_changed_options = false;

    //Initial System State
    systemState.vent_mode = VENTMODE_MAN; //0
    systemState.sleep_mode = false;
    systemState.put_to_sleep = false;
    systemState.wake_up = false;

    // Configure ventilation
    ventilation -> start();
    ventilation -> update(systemState, sensorParams);

    #ifdef ACCEL_STEPPER
    stepper->setSpeed(STEPPER_HOMING_SPEED);
          
    long initial_homing=-1;
    // HOMING TODO: PASAR NUEVAMENTE ESTA VARIABLE A PRIVADA
    while (digitalRead(PIN_ENDSTOP)) {  // Make the Stepper move CCW until the switch is activated
        stepper->moveTo(initial_homing);  // Set the position to move to
        initial_homing--;  // Decrease by 1 for next move if needed
        stepper->run();  // Start moving the stepper
        delay(5);
    }
    stepper->setCurrentPosition(0);  // Set the current position as zero for now
    initial_homing=1;
        
    while (!digitalRead(PIN_ENDSTOP)) { // Make the Stepper move CW until the switch is deactivated
        stepper->moveTo(initial_homing);
        stepper->run();
        initial_homing++;
        delay(5);
    }
    long position=stepper->currentPosition();
    Serial.print("Position ");Serial.print(position);
    stepper->setCurrentPosition(STEPPER_LOWEST_POSITION);
    position=stepper->currentPosition();
    Serial.print("Position ");Serial.print(position);
         
    Serial.println("home end");
    #endif

    //sensors -> readPressure();
    display_lcd(menuState, systemState.vent_mode, sensorParams);

    pinMode(PIN_ENC_SW, INPUT_PULLUP);

    lastReadSensor = lastShowSensor = millis();
    last_update_display = millis();

    //STEPPER
    last_vent_time = millis();

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

    f_acc=(float)f_acc_b/10.;
    dpip=(float)dpip_b/10.;
  
    Serial.print("Maxcd: ");Serial.println(max_cd);
        
    Serial.print("LAST CYCLE: "); Serial.println(last_cycle);
    ventilation->setCycleNum(last_cycle);

    tft.begin();
    tft.fillScreen(ILI9341_BLACK);

    digitalWrite(BCK_LED,LOW);
    buzzmuted=false;
    last_mute=HIGH;
    mute_count=0;

    lcd.clear();

    pf_min=(float)pfmin/50.;
    pf_max=(float)pfmax/50.;
    peep_fac = -(pf_max-pf_min)/15.* sensorParams.last_pressure_min + pf_max;

    Serial.println("Exiting setup");
}


bool update_display = false;
byte pos;

void loop() {

    if ( !systemState.sleep_mode ) {
        if ( systemState.wake_up ) {
            lcd.clear();
            init_display();
            display_lcd(menuState, systemState.vent_mode, sensorParams);
            tft.fillScreen(ILI9341_BLACK);
            systemState.wake_up = false;
        }
        State state = ventilation->getState();
        check_encoder(menuState, systemState, sensorParams);
   
        time = millis();
        check_buzzer_mute();
        //Serial.print("Carga: ");Serial.println(analogRead(PIN_BAT_LEV));
      
        if (millis() > lastSave + TIME_SAVE) {
            write_memory();
        
            lastSave = millis();
        }


        if ( time > lastShowSensor + TIME_SHOW ) {
            lastShowSensor=time;
            tft_draw(alarm_state, sensorParams);
        }
    
    
        if (time > lastReadSensor + TIME_SENSOR) {
    
            sensorParams.pressure_p = ( analogRead(A0) / (1023.) /*- verrp * 0.2 */ - 0.04 ) / 0.09 * 1000 * DEFAULT_PA_TO_CM_H20;//MPX5010
        
            //pressure_p = (( float(analogRead(A0))/1023.*V_SUPPLY_HONEY - 0.1 * V_SUPPLY_HONEY/* - corr_fs */) / (0.8 * V_SUPPLY_HONEY) * DEFAULT_PSI_TO_CM_H20 * 2. - DEFAULT_PSI_TO_CM_H20);//HONEYWELL

            int16_t adc0 = ads.readADC_SingleEnded(0);
            Voltage = (adc0 * 0.1875) *0.001; //Volts
    
            float p_dpt = ( Voltage /*- verror*/ - 0.20 ) / 0.45 * 1000 * DEFAULT_PA_TO_CM_H20; //WITH TRIM
            //ORIGINAL CON ERROR NEN TENSION
            // p_dpt = ( Voltage - 0.20 -verror ) / 0.45 * 1000 * DEFAULT_PA_TO_CM_H20 + (float(menuState.p_trim) - 100.0) * 1e-3 - verror; //WITH TRIM
            //p_dpt = ( Voltage - 0.20 - verror - 0.004) / 0.45 * 1000 * DEFAULT_PA_TO_CM_H20; //WITH TRIM  //ADDED TRIM
            update_error(p_dpt);
    
            p_dpt -= sensorParams.verror + (float(menuState.p_trim) - 100.0) * 1e-3; //WITH TRIM
            pos = findClosest(dp, 55, p_dpt);
            //flux should be shifted up (byte storage issue)
            float _flux = po_flux[pos] - 100 + ( float (po_flux[pos + 1] - 100) - float (po_flux[pos] - 100) ) * ( p_dpt - float(dp[pos]) ) / (float)( dp[pos + 1] - dp[pos]);
            _flux *= 16.6667;

            float _flux_fil[5];
            if ( filter ) {
                for (int i = 0; i < 4; i++) {
                    _flux_fil[i] = _flux_fil[i + 1];
                }
                _flux_fil[4] = _flux;
                float _flux_sum = 0.;
                for (int i = 0; i < 5; i++)
                    _flux_sum += _flux_fil[i];
                
                sensorParams.flow_f = _flux_sum / 5.;
            } else {
                sensorParams.flow_f = _flux;
            }
    
            if ( _flux > 0 ) {
                _mlInsVol += sensorParams.flow_f * float((millis() - lastReadSensor)) * 0.001; //flux in l and time in msec, results in ml
            } else {
                _mlExsVol -= sensorParams.flow_f * float((millis() - lastReadSensor)) * 0.001; //flux in l and time in msec, results in ml
            }
    
            lastReadSensor = millis();
    
            //CHECK PIP AND PEEP (OUTSIDE ANY CYCLE!!)
            if ( sensorParams.pressure_p > sensorParams.pressure_max ) {
                sensorParams.pressure_max = sensorParams.pressure_p;
            }
            if ( sensorParams.pressure_p < sensorParams.pressure_min) {
                sensorParams.pressure_min = sensorParams.pressure_p;
            }
        }//Read Sensor
    
        if (alarm_vt) {
        
        }

        if ( ventilation -> getCycleNum () != last_cycle ) {
            int vt = (_mllastInsVol + _mllastInsVol)/2;
            if ( vt < alarm_vt )  isalarmvt_on=1;
            else              isalarmvt_on=0;
            if ( sensorParams.last_pressure_max > alarm_max_pressure + 1 ) {
                if ( sensorParams.last_pressure_min < alarm_peep_pressure - 1) {
                    if (!isalarmvt_on)  alarm_state = 3;
                    else                alarm_state = 13;
                } else {
                    if (!isalarmvt_on)  alarm_state = 2;
                    else                alarm_state = 12;
                }
            } else {
                if ( sensorParams.last_pressure_min < alarm_peep_pressure - 1 ) {
                    if (!isalarmvt_on) alarm_state = 1;
                    else               alarm_state = 11;
                } else {
                    if (!isalarmvt_on)  alarm_state = 0;
                    else                alarm_state = 10;
                }
            }
        
            last_cycle = ventilation->getCycleNum();
    
            display_lcd(menuState, systemState.vent_mode, sensorParams);
            update_display = true;
            last_update_display = time;
    
            #ifdef DEBUG_PID
            if (systemState.vent_mode = VENTMODE_PCL) {
                float err = (float)( sensorParams.pressure_max - options.peakInspiratoryPressure) / options.peakInspiratoryPressure;
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

            if (digitalRead(PIN_POWEROFF)) {
                digitalWrite(YELLOW_LED,HIGH);
                //Serial.println("Yellow high");
            } else {
                digitalWrite(YELLOW_LED,LOW);
            }
        }//change cycle
    
        if (systemState.display_needs_update) {
            display_lcd(menuState, systemState.vent_mode, sensorParams);
            systemState.display_needs_update = false;
        }
    
        if ( menuState.update_options ) {
            ventilation->change_config(options);
            menuState.update_options = false;
            //menuState.show_changed_options=true;
        }
 
//////////////// CAUTION
// // //		WITH LATEST HARDWARE; IF VENTILATION UPDATE IS IN THE MAIN LOOP LIKE THIS
// // //		MENU ACTIONS INTERFERE WITH VENTILATION MECHANICS
//      if ( millis () - last_vent_time > TIME_BASE ) {
//        ventilation -> update();
//      }
    
        //HERE changed_options flag is not updating until cycle hcanges
        if ( menuState.show_changed_options && ((millis() - last_update_display) > time_update_display) ) {
            display_lcd(menuState, systemState.vent_mode, sensorParams);  //WITHOUT CLEAR!
            last_update_display = millis();
            menuState.show_changed_options = false;
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

    } else {
        if ( systemState.put_to_sleep ) {
            tft.fillScreen(ILI9341_BLACK);
            digitalWrite(PIN_LCD_EN,HIGH);
            systemState.put_to_sleep = false;
            print_bat_time=time;
            print_bat();
            digitalWrite(PIN_BUZZER,!BUZZER_LOW); //Buzzer inverted
            lcd.clear();
        }
        if (time > print_bat_time + 5000) {
            print_bat();
            print_bat_time=time;
        }
        time = millis();
        check_bck_state(menuState, systemState);
    }

}//LOOP

void timer1Isr(void) {
    ventilation->update(systemState, sensorParams);
    //alarms->update(ventilation->getPeakInspiratoryPressure());
}

void update_error(float p_dpt) {
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
        sensorParams.verror = verror_sum / ((float)vcorr_count + 1.);
        verror_sum = 0.;
        vcorr_count = 0;
        init_verror = false;
    }
}

//void timer2Isr(void)
//{
//  ventilation -> update();
//}

void timer3Isr(void) {
    #ifdef ACCEL_STEPPER
    stepper->run();
    #else
    stepper -> processMovement(); //LUCIANO
    #endif
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
    bool curr_mute = debounce(last_mute, PIN_MUTE);         //Debounce for Up button
    if (last_mute== HIGH && curr_mute == LOW && !buzzmuted){
        mute_count=time;
        buzzmuted=true;
    }
    last_mute = curr_mute;
    if( buzzmuted ) {
        if (time > mute_count  + TIME_MUTE) { //each count is every 500 ms
            buzzmuted=false;
        }
    }
}

void read_memory(){
    int eeAddress=0;
    EEPROM.get(0, last_cycle); eeAddress+= sizeof(unsigned long);
    EEPROM.get(eeAddress, menuState.p_trim);    eeAddress+= sizeof(menuState.p_trim);
    EEPROM.get(eeAddress, autopid);   eeAddress+= sizeof(autopid);
    EEPROM.get(eeAddress, min_cd);    eeAddress+= sizeof(min_cd);
    EEPROM.get(eeAddress, max_cd);    eeAddress+= sizeof(max_cd);
    EEPROM.get(eeAddress, min_speed); eeAddress+= sizeof(min_speed);
    EEPROM.get(eeAddress, max_speed); eeAddress+= sizeof(max_speed);
    EEPROM.get(eeAddress, min_accel); eeAddress+= sizeof(min_accel);
    EEPROM.get(eeAddress, max_accel); eeAddress+= sizeof(max_accel);
    EEPROM.get(eeAddress, min_pidk);  eeAddress+= sizeof(min_pidk);
    EEPROM.get(eeAddress, max_pidk);  eeAddress+= sizeof(max_pidk);
    EEPROM.get(eeAddress, alarm_vt);  eeAddress+= sizeof(alarm_vt);
    EEPROM.get(eeAddress, filter);    eeAddress+= sizeof(filter);
    EEPROM.get(eeAddress, pfmin);     eeAddress+= sizeof(pfmin);
    EEPROM.get(eeAddress, pfmax);     eeAddress+= sizeof(pfmax);
    EEPROM.get(eeAddress, dpip_b);    eeAddress+= sizeof(dpip_b);
    EEPROM.get(eeAddress, min_pidi);  eeAddress+= sizeof(min_pidi);
    EEPROM.get(eeAddress, max_pidi);  eeAddress+= sizeof(max_pidi);
    EEPROM.get(eeAddress, min_pidd);  eeAddress+= sizeof(min_pidd);
    EEPROM.get(eeAddress, max_pidd);  eeAddress+= sizeof(max_pidd);
    EEPROM.get(eeAddress, p_acc);      eeAddress+= sizeof(p_acc);
    EEPROM.get(eeAddress, f_acc_b);    eeAddress+= sizeof(f_acc_b);
}

void write_memory(){
    int eeAddress=0;
    EEPROM.put(0, last_cycle);        eeAddress+= sizeof(unsigned long);
    EEPROM.put(eeAddress, menuState.p_trim);    eeAddress+= sizeof(menuState.p_trim);
    EEPROM.put(eeAddress, autopid);   eeAddress+= sizeof(autopid);
    EEPROM.put(eeAddress, min_cd);    eeAddress+= sizeof(min_cd);
    EEPROM.put(eeAddress, max_cd);    eeAddress+= sizeof(max_cd);
    EEPROM.put(eeAddress, min_speed); eeAddress+= sizeof(min_speed);
    EEPROM.put(eeAddress, max_speed); eeAddress+= sizeof(max_speed);
    EEPROM.put(eeAddress, min_accel); eeAddress+= sizeof(min_accel);
    EEPROM.put(eeAddress, max_accel); eeAddress+= sizeof(max_accel);
    EEPROM.put(eeAddress, min_pidk);  eeAddress+= sizeof(min_pidk);
    EEPROM.put(eeAddress, max_pidk);  eeAddress+= sizeof(max_pidk);
    EEPROM.put(eeAddress, alarm_vt);  eeAddress+= sizeof(alarm_vt);
    EEPROM.put(eeAddress, filter);    eeAddress+= sizeof(filter);   
    EEPROM.put(eeAddress, pfmin);     eeAddress+= sizeof(pfmin);
    EEPROM.put(eeAddress, pfmax);     eeAddress+= sizeof(pfmax);
    EEPROM.put(eeAddress, dpip_b);    eeAddress+= sizeof(dpip_b);
    EEPROM.put(eeAddress, min_pidi);  eeAddress+= sizeof(min_pidi);
    EEPROM.put(eeAddress, max_pidi);  eeAddress+= sizeof(max_pidi);  
    EEPROM.put(eeAddress, min_pidd);  eeAddress+= sizeof(min_pidd);
    EEPROM.put(eeAddress, max_pidd);  eeAddress+= sizeof(max_pidd);
    EEPROM.put(eeAddress, p_acc);      eeAddress+= sizeof(p_acc);
    EEPROM.put(eeAddress, f_acc_b);    eeAddress+= sizeof(f_acc_b);
}
