#include "src/TimerOne/TimerOne.h"
#include "src/TimerThree/TimerThree.h"

#include "menu.h"
#include "display.h"
#include "alarms.h"
#include "MechVentilation.h"
#include "sensorcalculation.h"
#include "data_persistence.h"

#ifdef TEMP_TEST
#include <OneWire.h>
#include <DallasTemperature.h>
float temp;
unsigned lastReadTemp = 0;
OneWire           oneWire(PIN_TEMP);
DallasTemperature sensors(&oneWire);
#endif

//SLEEP
bool sleep_mode = false;
bool put_to_sleep = false;
bool wake_up = false;
unsigned long print_bat_time;

//TFT DISPLAY
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
bool drawing_cycle = 0;
unsigned long lastShowSensor = 0;
float fdiv = (float)(BATDIV_R1 + BATDIV_R2)/(float)BATDIV_R2;
float fac = 1.1/1024.*fdiv;

//SENSORS
Adafruit_ADS1115 ads;
int _mllastInsVol, _mllastExsVol;
SensorData sensorData;

//MUTE
bool last_mute;
bool buzzmuted;
unsigned long mute_count;
unsigned long timebuzz = 0;
bool isbuzzeron = false;

//ALARMS
AlarmData alarm_data;

//CALIBRATION
byte vcorr_count = 0;
float verror_sum, verror_sum_outcycle, vzero = 0.;  //verror sum is intra cycle, verror_sum_outcycle is inter-cycle
bool calibration_run = true;
byte calib_cycle = 0;

//PERSISTENCE
unsigned long lastSave = 0;

#ifdef BAT_TEST
unsigned long lastShowBat = 0;
#endif

unsigned long time;

byte p_trim = 100;
int max_accel, min_accel;
int max_speed, min_speed;
int min_pidk, max_pidk;
int min_pidi, max_pidi;
int min_pidd, max_pidd;
byte pfmin, pfmax;
float pf_min, pf_max;
float peep_fac;
int min_cd, max_cd;

//MENU
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
unsigned long lastButtonPress;
byte encoderPos = 1; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
byte oldEncPos = 1; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
byte max_sel, min_sel; //According to current selection
bool isitem_sel = false;
char tempstr[5];
int curr_sel, old_curr_sel;
byte menu_number = 0;
unsigned long last_update_display = 0;
bool display_needs_update = false;
bool show_changed_options = false; //Only for display

//KEYBOARD
int bck_state ;     // current state of the button
int last_bck_state ; // previous state of the button
int startPressed ;    // the moment the button was pressed
int endPressed ;      // the moment the button was released
int holdTime ;        // how long the button was hold
int idleTime ;        // how long the button was idle

//MECH VENTILATION
AutoPID * pid;
AccelStepper *stepper;
MechVentilation * ventilation;
VentilationOptions_t options;
byte vent_mode = VENTMODE_MAN;
bool autopid;
bool filter;
unsigned long last_cycle;
unsigned int _timeoutIns, _timeoutEsp; //In ms
byte cycle_pos;
bool update_options = false;

void wait_for_flux_disconnected();
void searchHomePosition(AccelStepper* stepper);
float calculate_calib_cycle_verror(byte& calib_cycle, float& verror_sum, byte& vcorr_count);
void show_power_off_led();
  
void setup() {
    Serial.begin(115200);
    analogReference(INTERNAL1V1); // use AREF for reference voltage
    initPins();
    init_display();

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

    delay(100);

    // Habilita el motor
    digitalWrite(PIN_EN, LOW);

    writeLine(1, "RespirAR FIUBA", 4);
    writeLine(2, "v2.0.2", 8);

    init_display_tft(tft);
    init_sensor(ads);
    Menu_inic menuini(&vent_mode, &options.respiratoryRate, &options.percInspEsp);
    curr_sel = old_curr_sel = 1;
    wait_for_flux_disconnected();

    digitalWrite(PIN_STEPPER, HIGH);
    delay(1000);

    stepper = new AccelStepper(
        AccelStepper::DRIVER,
        PIN_STEPPER_STEP,
        PIN_STEPPER_DIRECTION);

    ventilation = new MechVentilation(stepper, pid, options);
    ventilation -> start();
    ventilation -> update(sensorData);

    searchHomePosition(stepper);
    lastShowSensor = last_update_display = sensorData.last_read_sensor = millis();

    #ifdef BAT_TEST
    lastShowBat = millis();
    #endif

    //STEPPER
    Timer3.initialize(TIME_STEPPER_ISR_MICROS);
    Timer3.attachInterrupt(timer3Isr);

    Timer1.initialize(TIME_BASE_MICROS);
    Timer1.attachInterrupt(timer1Isr);

    SystemConfiguration_t systemConfiguration = read_memory();
    alarm_data.alarm_vt = systemConfiguration.alarm_vt;
    filter = systemConfiguration.filter;
    autopid = systemConfiguration.autopid;
    last_cycle = systemConfiguration.last_cycle;

    f_acc = (float)f_acc_b / 10.;
    dpip = (float)dpip_b / 10.;

    ventilation->setCycleNum(last_cycle);

    buzzmuted = false;
    last_mute = HIGH;
    mute_count = 0;

    alarm_data.alarm_max_pressure = 35;
    alarm_data.alarm_peep_pressure = 5;
    alarm_data.alarm_vt = 200;

    lcd.clear();

    pf_min = (float)pfmin / 50.;
    pf_max = (float)pfmax / 50.;
    peep_fac = -(pf_max - pf_min) / 15.*sensorData.last_pressure_min + pf_max;

    sleep_mode = false;
    put_to_sleep = false;
    wake_up = false;

    #ifdef TEMP_TEST
    sensors.begin();
    #endif
}
////////////////////////////////////////
////////////// MAIN LOOP ///////////////
////////////////////////////////////////
void loop() {

    time = millis();
    if (!sleep_mode) {
        if (wake_up) {
            digitalWrite(PIN_STEPPER, HIGH);
            init_display();
            init_display_tft(tft);
            wake_up = false;
            ventilation->forceStart();
        }

        check_encoder();
        buzzmuted = check_buzzer_mute(last_mute, buzzmuted, mute_count, time);

        if (calibration_run) {
            if (time > sensorData.last_read_sensor + TIME_SENSOR) {
                readSensor(ads, sensorData, vzero, filter);
                vcorr_count++;
                verror_sum += (sensorData.voltage - 0.04 * sensorData.v_level);
            }

            if (ventilation->getCycleNum() != last_cycle) {
                last_cycle = ventilation->getCycleNum();
                verror_sum_outcycle += calculate_calib_cycle_verror(calib_cycle, verror_sum, vcorr_count);
            }

            if (calib_cycle >= CALIB_CYCLES) {
                calibration_run = false;
                vzero = verror_sum_outcycle / float(CALIB_CYCLES);
                Serial.println("Calibration verror: " + String(vzero));
                lcd.clear();
                clean_tft(tft);
            }
        } else {
            if (time > lastSave + TIME_SAVE) {
                SystemConfiguration_t toPersist;
                toPersist.last_cycle = last_cycle;
                toPersist.alarm_vt = alarm_data.alarm_vt;
                toPersist.autopid = autopid;
                toPersist.filter = filter;
                write_memory(toPersist);
                lastSave = millis();
            }

            if (time > lastShowSensor + TIME_SHOW) {
                tft_draw(tft, sensorData, drawing_cycle, fac, alarm_data.alarm_state);
                lastShowSensor = time;
            }

            if (time > sensorData.last_read_sensor + TIME_SENSOR) {
                readSensor(ads, sensorData, vzero, filter);
            }//Read Sensor

            if (ventilation->getCycleNum() != last_cycle) {
                alarm_data.is_alarm_vt_on = calc_alarm_vt_is_on(_mllastInsVol, _mllastInsVol, alarm_data.alarm_vt);
                alarm_data.alarm_state = get_alarm_state(sensorData.last_pressure_max,
                                              sensorData.last_pressure_min,
                                              alarm_data.alarm_max_pressure,
                                              alarm_data.alarm_peep_pressure);

                last_cycle = ventilation->getCycleNum();
                display_needs_update = true;
                show_power_off_led();

                #ifdef TEMP_TEST
                if (time > lastReadTemp + TIME_READ_TEMP) {
                    lastReadTemp = time;
                    sensors.requestTemperatures();
                    temp=sensors.getTempCByIndex(0);
                    //Serial.println ("Temp: " + String(temp));
                }
                tft.fillRect(200, 100, 20, 40, ILI9341_BLUE);
                print_float(tft, 100, 200, temp);
                #endif //TEMP_TEST
            }//change cycle

            if (display_needs_update) {
                display_lcd();
                last_update_display = millis();
                display_needs_update = false;
            }

            if (update_options) {
                ventilation->change_config(options);
                update_options = false;
            }

            if (show_changed_options && ((millis() - last_update_display) > TIME_UPDATE_DISPLAY)) {
                display_lcd();
                last_update_display = millis();
                show_changed_options = false;
            }

            set_alarm_buzzer(alarm_data.alarm_state, buzzmuted, timebuzz, isbuzzeron);
        }
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
        check_bck_state();
    }

    #ifdef BAT_TEST
    if ( time > lastShowBat + TIME_SHOW_BAT ) {
        lastShowBat = time;
        Serial.println("last show bat " + String(lastShowBat));
        float level = calc_bat(5, fac);
        Serial.println(String(time)+", " +String(level));
    }
    #endif//BAT_TEST
}//LOOP

void timer1Isr(void) {
    ventilation->update(sensorData);
}

void timer3Isr(void) {
    stepper->run();
}

void searchHomePosition(AccelStepper* stepper) {
    lcd.clear();
    writeLine(1, "Iniciando...", 0);
    // Move to Mech Ventilation
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

void wait_for_flux_disconnected() {
    lcd.clear();
    writeLine(1, "Desconecte flujo", 0);
    writeLine(2, "y presione ok ", 0);
    bool enterPressed = false;
    delay(100); //Otherwise low enter button is read
    long lastButtonPress = millis();
    while (!enterPressed) {
        if (digitalRead(PIN_MENU_EN) == LOW) {
            if (millis() - lastButtonPress > 50) {
                enterPressed = true;
                lastButtonPress = millis();
            }
        }
    }
}

void show_power_off_led() {
    if (!digitalRead(PIN_POWEROFF)) {
        digitalWrite(YELLOW_LED, HIGH);
    } else {
        digitalWrite(YELLOW_LED, LOW);
    }
}

float calculate_calib_cycle_verror(byte& calib_cycle, float& verror_sum, byte& vcorr_count) {
    lcd.clear();
    writeLine(1, "Calibracion flujo", 0);
    writeLine(2, "Ciclo: " + String(calib_cycle+1) + "/" + String(CALIB_CYCLES), 0);

    float verror = verror_sum / float(vcorr_count);
    vcorr_count = verror_sum = 0.;
    calib_cycle++;
    return verror;
}