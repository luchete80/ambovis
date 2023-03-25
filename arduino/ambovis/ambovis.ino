#include "src/TimerOne/TimerOne.h"
#include "src/TimerThree/TimerThree.h"

#include "menu.h"
#include "display.h"
#include "MechanicalVentilation.h"
#include "alarms.h"
#include "initialactions.h"
#include "data_persistence.h"

#define CALIB_CYCLES  5

#ifdef TEMP_TEST
#include <OneWire.h>
#include <DallasTemperature.h>
float temp;
#endif

bool autopid;
bool filter;
bool sleep_mode;
bool put_to_sleep, wake_up;
unsigned long print_bat_time;

bool drawing_cycle = 0;

Adafruit_ADS1115 ads;
SensorData sensorData;

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
byte menu_number = 0;
byte alarm_max_pressure = 35;
byte alarm_peep_pressure = 5;
int alarm_vt = 200;

unsigned long lastButtonPress;
byte encoderPos = 1; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
byte oldEncPos = 1; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
byte max_sel, min_sel; //According to current selection
bool isitem_sel =false;

MechanicalVentilation mechanicalVentilation;
AccelStepper* stepper;

void init_display_tft(Adafruit_ILI9341& tft);

int bck_state ;     // current state of the button
int last_bck_state ; // previous state of the button
int startPressed ;    // the moment the button was pressed
int endPressed ;      // the moment the button was released
int holdTime ;        // how long the button was hold
int idleTime ;        // how long the button was idle

// TODO: move to constants
float fdiv = (float)(BATDIV_R1 + BATDIV_R2)/(float)BATDIV_R2;
float fac = 1.1/1024.*fdiv;

float dpip;
byte dpip_b;
float f_acc;
byte f_acc_b;
byte  p_acc;

#ifdef TEMP_TEST
OneWire           oneWire(PIN_TEMP);
DallasTemperature sensors(&oneWire);
#endif
  
void setup() {
    Serial.begin(115200);
    analogReference(INTERNAL1V1); // use AREF for reference voltage
    init_display();
    initPins();

    max_cd = 40; //T MODIFY: READ FROM MEM
    min_cd = 10;
    min_speed = 250;  // x microsteps
    max_speed = 750;  // x Microsteps, originally 16000 (with 16 ms = 750)
    max_accel = 600;
    min_accel = 200;

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

    mechanicalVentilation.ventilationConfig.vent_mode = VENTMODE_MAN;
    mechanicalVentilation.ventilationConfig.respiratoryRate = DEFAULT_RPM;
    mechanicalVentilation.ventilationConfig.percIE = 2;
    mechanicalVentilation.ventilationConfig.peakInspiratoryPressure = DEFAULT_PEAK_INSPIRATORY_PRESSURE;
    mechanicalVentilation.ventilationConfig.peakExpiratoryPressure = DEFAULT_PEAK_ESPIRATORY_PRESSURE;
    mechanicalVentilation.ventilationConfig.percVolume = 100;
    mechanicalVentilation.ventilationConfig.stepper_speed_max = STEPPER_MICROSTEPS * 1500;
    mechanicalVentilation.ventilationConfig.stepper_accel_max= STEPPER_MICROSTEPS * 1500;
    mechanicalVentilation.ventilationStatus.running = true;
    mechanicalVentilation.ventilationStatus.currentState = State_Homing;
    mechanicalVentilation.ventilationStatus.startCycleTimeInMs = 0;
    mechanicalVentilation.ventilationStatus.endedWhileMoving = false;
    Menu_inic menuini(mechanicalVentilation.ventilationConfig);

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

    mechanicalVentilation.stepper = stepper;
    start(mechanicalVentilation);
    update(mechanicalVentilation, sensorData);

    tft.fillScreen(ILI9341_BLACK);
    lcd.clear();
    writeLine(1, "Iniciando...", 0);
    search_home_position(stepper);

    display_lcd(mechanicalVentilation.ventilationStatus, mechanicalVentilation.ventilationConfig);

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
    mechanicalVentilation.ventilationStatus.last_cycle = systemConfiguration.last_cycle;
    mechanicalVentilation.ventilationStatus.cycleNum = systemConfiguration.last_cycle;
    alarm_vt = systemConfiguration.alarm_vt;
    filter = systemConfiguration.filter;
    autopid = systemConfiguration.autopid;

    f_acc = (float)f_acc_b / 10.;
    dpip = (float)dpip_b / 10.;

    digitalWrite(BCK_LED, LOW);
    buzzmuted = false;
    last_mute = HIGH;
    mute_count = 0;

    lcd.clear();

    pf_min = (float)pfmin / 50.;
    pf_max = (float)pfmax / 50.;
    peep_fac = -(pf_max - pf_min) / 15.* mechanicalVentilation.ventilationStatus.lastMinPressure + pf_max;

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
            init_display();
            display_lcd(mechanicalVentilation.ventilationStatus, mechanicalVentilation.ventilationConfig);
            tft.begin();
            tft.fillScreen(ILI9341_BLACK);
            wake_up = false;
            start(mechanicalVentilation);
        }

        buzzmuted = check_buzzer_mute(last_mute, buzzmuted, mute_count, time);
        check_encoder(mechanicalVentilation.ventilationStatus, mechanicalVentilation.ventilationConfig);
        VentilationStatus* vent_status = &mechanicalVentilation.ventilationStatus;
        VentilationConfig* vent_config = &mechanicalVentilation.ventilationConfig;

        if (calibration_run) {
            if (time > sensorData.last_read_sensor + TIME_SENSOR) {
                readSensor(ads, sensorData, vzero, filter);
                vcorr_count++;
                verror_sum += (sensorData.voltage - 0.04 * sensorData.v_level);
            }

            if (vent_status->cycleNum != vent_status->last_cycle) {
                lcd.clear();
                writeLine(1, "Calibracion flujo", 0);
                writeLine(2, "Ciclo: " + String(calib_cycle+1) + "/" + String(CALIB_CYCLES), 0);

                vent_status->last_cycle = vent_status->cycleNum;

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
                toPersist.last_cycle = vent_status->last_cycle;
                toPersist.alarm_vt = alarm_vt;
                toPersist.autopid = autopid;
                toPersist.filter = filter;
                write_memory(toPersist);
                lastSave = millis();
            }

            if (time > lastShowSensor + TIME_SHOW) {
                tft_draw(tft, sensorData, mechanicalVentilation.ventilationStatus, drawing_cycle, fac);
                lastShowSensor = time;
            }

            if (time > sensorData.last_read_sensor + TIME_SENSOR) {
                readSensor(ads, sensorData, vzero, filter);

                //CHECK PIP AND PEEP (OUTSIDE ANY CYCLE!!)
                if (sensorData.pressure_p > vent_status->pressure_max) {
                    vent_status->pressure_max = sensorData.pressure_p;
                }
                if (sensorData.pressure_p < vent_status->pressure_min) {
                    vent_status->pressure_min = sensorData.pressure_p;
                }

            }//Read Sensor

            if (vent_status->cycleNum != vent_status->last_cycle) {
                is_alarm_vt_on = calc_alarm_vt_is_on(vent_status->mlLastInsVol,
                                                     vent_status->mlLastInsVol,
                                                     alarm_vt);
                alarm_state = get_alarm_state(vent_status->lastMaxPressure,
                                              vent_status->lastMinPressure,
                                              alarm_max_pressure,
                                              alarm_peep_pressure);

                vent_status->last_cycle = vent_status->cycleNum;
                display_needs_update = true;

                if (!digitalRead(PIN_POWEROFF)) {
                    digitalWrite(YELLOW_LED, HIGH);
                } else {
                    digitalWrite(YELLOW_LED, LOW);
                }
                Serial.println("Cycle changed " + String(millis()));
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
                display_lcd(mechanicalVentilation.ventilationStatus, mechanicalVentilation.ventilationConfig);
                last_update_display = millis();
                display_needs_update = false;
            }

            if (update_options) {
                update_config(mechanicalVentilation);
                update_options = false;
            }

            if (show_changed_options && ((millis() - last_update_display) > TIME_UPDATE_DISPLAY)) {
                display_lcd(mechanicalVentilation.ventilationStatus, mechanicalVentilation.ventilationConfig);
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
            lcd.clear();
            digitalWrite(LCD_SLEEP, LOW);
            digitalWrite(TFT_SLEEP, LOW);
            stop(mechanicalVentilation);
            //digitalWrite(PIN_BUZZER, !BUZZER_LOW); //Buzzer inverted
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
    update(mechanicalVentilation, sensorData);
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
