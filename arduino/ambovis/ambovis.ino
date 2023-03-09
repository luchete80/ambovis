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
#include "initialactions.h"
#include "data_persistence.h"

#define CALIB_CYCLES  5

#ifdef TEMP_TEST
#include <OneWire.h>
#include <DallasTemperature.h>
float temp;
#endif

byte Cdyn;
bool autopid;
bool filter;
bool sleep_mode;
bool put_to_sleep, wake_up;
unsigned long print_bat_time;
bool drawing_cycle = 0;

// FOR ADS
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;

int _mllastInsVol, _mllastExsVol;

AccelStepper *stepper;

short alarm_state = 0; //0: No alarm 1: peep 2: pip 3:both

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

//MUTE
bool last_mute;
bool buzzmuted;
unsigned long timebuzz = 0;
bool isbuzzeron = false;
unsigned long mute_count;
bool is_alarm_vt_on;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

byte vcorr_count = 0;
float verror, verror_sum, verror_sum_outcycle, vzero = 0.;  //verror sum is intra cycle, verror_sum_outcycle is inter-cycle

float last_pressure_max, last_pressure_min;

byte vent_mode = VENTMODE_MAN;

char tempstr[5];
int curr_sel, old_curr_sel;

unsigned long lastShowSensor = 0;
unsigned long lastSave = 0;
unsigned long last_update_display = 0;

#ifdef BAT_TEST
unsigned long lastShowBat = 0;
#endif
#ifdef TEMP_TEST
unsigned lastReadTemp = 0;
#endif

bool display_needs_update = false;
bool show_changed_options = false; //Only for display
bool update_options = false;

unsigned long time;
byte cycle_pos;

SensorData sensorData;

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

byte encoderPos = 1; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
byte oldEncPos = 1; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
byte max_sel, min_sel; //According to current selection
bool isitem_sel =false;

void init_display_tft(Adafruit_ILI9341& tft);
void init_ventilation_options(VentilationOptions_t& options);

AutoPID * pid;
MechVentilation * ventilation;
VentilationOptions_t options;

int bck_state ;     // current state of the button
int last_bck_state ; // previous state of the button
int startPressed ;    // the moment the button was pressed
int endPressed ;      // the moment the button was released
int holdTime ;        // how long the button was hold
int idleTime ;        // how long the button was idle

// TODO: move to constants
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

    init_ventilation_options(options);

    delay(100);

    if (!digitalRead(PIN_POWEROFF)) {
        digitalWrite(YELLOW_LED, HIGH);
    }

    // Habilita el motor
    digitalWrite(PIN_EN, LOW);

    writeLine(1, "RespirAR FIUBA", 4);
    writeLine(2, "v2.0.2", 8);

    init_display_tft(tft);

    ads.begin();

    byte bpm = DEFAULT_RPM;
    byte i_e = 2;
    Menu_inic menuini(&vent_mode, &bpm, &i_e);

    options.respiratoryRate = bpm;
    options.percInspEsp = i_e; //1:1 to 1:4, is denom
    vent_mode = VENTMODE_MAN;

    lcd.clear();
    writeLine(1, "Desconecte flujo", 0);
    writeLine(2, "y presione ok ", 0);
    waitForFluxDisconnected();

    digitalWrite(PIN_STEPPER, HIGH);
    delay(1000);

    stepper = new AccelStepper(
        AccelStepper::DRIVER,
        PIN_STEPPER_STEP,
        PIN_STEPPER_DIRECTION);

    ventilation = new MechVentilation(stepper, pid, options);
    ventilation -> start();
    ventilation -> update(sensorData);

    tft.fillScreen(ILI9341_BLACK);
    lcd.clear();
    writeLine(1, "Iniciando...", 0);

    searchHomePosition(stepper);

    display_lcd();

    curr_sel = old_curr_sel = 1;

    lastShowSensor = last_update_display = millis();
    sensorData.last_read_sensor = millis();

    #ifdef BAT_TEST
    lastShowBat = millis();
    #endif

    //STEPPER
    Timer3.initialize(TIME_STEPPER_ISR_MICROS);
    Timer3.attachInterrupt(timer3Isr);

    Timer1.initialize(TIME_BASE_MICROS);
    Timer1.attachInterrupt(timer1Isr);

    SystemConfiguration_t systemConfiguration = read_memory();
    alarm_vt = systemConfiguration.alarm_vt;
    filter = systemConfiguration.filter;
    autopid = systemConfiguration.autopid;
    last_cycle = systemConfiguration.last_cycle;

    f_acc = (float)f_acc_b / 10.;
    dpip = (float)dpip_b / 10.;

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

    time = millis();
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

        buzzmuted = check_buzzer_mute(last_mute, buzzmuted, mute_count, time);

        if (calibration_run) {
            if (time > sensorData.last_read_sensor + TIME_SENSOR) {
                readSensor(ads, sensorData, vzero, filter);
                vcorr_count++;
                verror_sum += (sensorData.voltage - 0.04 * sensorData.v_level);
            }

            if (ventilation->getCycleNum() != last_cycle) {
                lcd.clear();
                writeLine(1, "Calibracion flujo", 0);
                writeLine(2, "Ciclo: " + String(calib_cycle+1) + "/" + String(CALIB_CYCLES), 0);

                last_cycle = ventilation->getCycleNum();

                verror = verror_sum / float(vcorr_count);
                vcorr_count = verror_sum = 0.;
                calib_cycle++;
                verror_sum_outcycle += verror;
            }

            if (calib_cycle >= CALIB_CYCLES) {
                calibration_run = false;
                vzero = verror_sum_outcycle / float(CALIB_CYCLES);
                Serial.println("Calibration verror: " + String(vzero));
                lcd.clear();
                tft.fillScreen(ILI9341_BLACK);
            }
        }
        else {
            if (time > lastSave + TIME_SAVE) {
                SystemConfiguration_t toPersist;
                toPersist.last_cycle = last_cycle;
                toPersist.alarm_vt = alarm_vt;
                toPersist.autopid = autopid;
                toPersist.filter = filter;
                write_memory(toPersist);
                lastSave = millis();
            }

            if (time > lastShowSensor + TIME_SHOW) {
                tft_draw(tft, sensorData, drawing_cycle, fac);
                lastShowSensor = time;
            }

            if (time > sensorData.last_read_sensor + TIME_SENSOR) {
                readSensor(ads, sensorData, vzero, filter);

                //CHECK PIP AND PEEP (OUTSIDE ANY CYCLE!!)
                if (sensorData.pressure_p > pressure_max) {
                    pressure_max = sensorData.pressure_p;
                }
                if (sensorData.pressure_p < pressure_min) {
                    pressure_min = sensorData.pressure_p;
                }

            }//Read Sensor

            if (ventilation->getCycleNum() != last_cycle) {
                is_alarm_vt_on = calc_alarm_vt_is_on(_mllastInsVol, _mllastInsVol, alarm_vt);
                alarm_state = get_alarm_state(last_pressure_max, last_pressure_min,
                                            alarm_max_pressure, alarm_peep_pressure);

                last_cycle = ventilation->getCycleNum();
                display_needs_update = true;

                if (!digitalRead(PIN_POWEROFF)) {
                    digitalWrite(YELLOW_LED, HIGH);
                } else {
                    digitalWrite(YELLOW_LED, LOW);
                }

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

            set_alarm_buzzer(alarm_state, buzzmuted, timebuzz, isbuzzeron);
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
    //alarms->update(ventilation->getPeakInspiratoryPressure());
}

void timer3Isr(void) {
    stepper->run();
}

void init_display_tft(Adafruit_ILI9341& tft) {
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_BLUE);
    tft.setTextSize(4);
    tft.setCursor(10, 40);     tft.println("RespirAR");
    tft.setCursor(10, 80);     tft.println("FIUBA");
}

void init_ventilation_options(VentilationOptions_t& options) {
    options.respiratoryRate = DEFAULT_RPM;
    options.percInspEsp = 2; //1:1 to 1:4, is denom
    options.peakInspiratoryPressure = DEFAULT_PEAK_INSPIRATORY_PRESSURE;
    options.peakEspiratoryPressure = DEFAULT_PEAK_ESPIRATORY_PRESSURE;
    options.triggerThreshold = DEFAULT_TRIGGER_THRESHOLD;
    options.hasTrigger = false;
    options.tidalVolume = 300; // TODO: might be removed
    options.percVolume = 100; //1 to 10
}
