#include "src/TimerOne/TimerOne.h"
#include "src/TimerThree/TimerThree.h"

#include "menu.h"
#include "display.h"
#include "alarms.h"
#include "MechanicalVentilation.h"
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

//SENSORS
Adafruit_ADS1115 ads;
SensorData sensorData;

//MUTE
Buzzer_State_t buzzer_state;

//ALARMS
AlarmData alarm_data;

//CALIBRATION
Calibration_Data_t calibration_data;

//PERSISTENCE
unsigned long lastSave = 0;

#ifdef BAT_TEST
unsigned long lastShowBat = 0;
#endif
#ifdef TEMP_TEST
unsigned lastReadTemp = 0;
OneWire           oneWire(PIN_TEMP);
DallasTemperature sensors(&oneWire);
#endif

unsigned long time2;

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
bool show_changed_options = false; //Only for display
bool update_options = false;

//KEYBOARD
int bck_state ;     // current state of the button
int last_bck_state ; // previous state of the button
int startPressed ;    // the moment the button was pressed
int endPressed ;      // the moment the button was released
int holdTime ;        // how long the button was hold
int idleTime ;        // how long the button was idle

//MECH VENTILATION
AccelStepper *stepper;
Mechanical_Ventilation_t mech_vent;
bool autopid;
bool filter;

float dpip;
byte dpip_b;
float f_acc;
byte f_acc_b;
byte  p_acc;

void setup() {
    Serial.begin(115200);
    analogReference(INTERNAL1V1); // use AREF for reference voltage
    initPins();
    show_power_led();
    init_display();

    max_cd = 40; //T MODIFY: READ FROM MEM
    min_cd = 10;
    min_speed = 250;  // x microsteps
    max_speed = 750;  // x Microsteps, originally 16000 (with 16 ms = 750)
    max_accel = 600;
    min_accel = 200;

    delay(100);
    writeLine(1, "RespirAR FIUBA", 4);
    writeLine(2, "v2.0.2", 8);

    init_display_tft(tft);
    init_sensor(ads);

    mech_vent.status.running = true;
    mech_vent.status.current_state = State_Homing;
    mech_vent.status.start_cycle_time_ms = 0;
    mech_vent.status.ended_while_moving = false;
    Menu_inic menuini(mech_vent.config);

    wait_for_flux_disconnected();

    stepper = new AccelStepper(
        AccelStepper::DRIVER,
        PIN_STEPPER_STEP,
        PIN_STEPPER_DIRECTION);

    mech_vent.stepper = stepper;
    start(mech_vent);
    update(mech_vent, sensorData);

    clean_tft(tft);
    lcd.clear();
    writeLine(1, "Iniciando...", 0);
    search_home_position(stepper);

    display_lcd(mech_vent.status, mech_vent.config);

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
    mech_vent.status.last_cycle = systemConfiguration.last_cycle;
    mech_vent.status.cycle = systemConfiguration.last_cycle;
    alarm_data.alarm_vt = systemConfiguration.alarm_vt;
    filter = systemConfiguration.filter;
    autopid = systemConfiguration.autopid;

    f_acc = (float)f_acc_b / 10.;
    dpip = (float)dpip_b / 10.;

    lcd.clear();

    pf_min = (float)pfmin / 50.;
    pf_max = (float)pfmax / 50.;
    peep_fac = -(pf_max - pf_min) / 15.* mech_vent.status.last_min_pressure + pf_max;

    sleep_mode = false;
    put_to_sleep = false;
    wake_up = false;

    calibration_data.calibration_run = true;

    #ifdef TEMP_TEST
    sensors.begin();
    #endif
}
////////////////////////////////////////
////////////// MAIN LOOP ///////////////
////////////////////////////////////////
void loop() {

    time2 = millis();
    if (!sleep_mode) {
        if (wake_up) {
            init_display();
            init_display_tft(tft);
            start(mech_vent);
            wake_up = false;
        }

        buzzer_state = check_buzzer_mute(buzzer_state, time2);
        check_encoder(mech_vent.status, mech_vent.config);
        Ventilation_Status_t* vent_status = &mech_vent.status;

        if (calibration_data.calibration_run) {
            if (time2 > sensorData.last_read_sensor + TIME_SENSOR) {
                read_sensor(ads, sensorData, calibration_data.vzero, filter);
                update_verror_sum(calibration_data, sensorData);
            }

            if (vent_status->cycle != vent_status->last_cycle) {
                vent_status->last_cycle = vent_status->cycle;
                update_cycle_verror_sum(calibration_data);
                show_calibration_cycle(calibration_data.calib_cycle);
            }

            if (calibration_data.calib_cycle >= CALIB_CYCLES) {
                calibration_data.calibration_run = false;
                calibration_data.vzero = calibration_data.verror_sum_outcycle / float(CALIB_CYCLES);
                Serial.println("Calibration verror: " + String(calibration_data.vzero));
                lcd.clear();
                clean_tft(tft);
            }
        } else {
            if (time2 > lastSave + TIME_SAVE) {
                SystemConfiguration_t toPersist;
                toPersist.last_cycle = vent_status->last_cycle;
                toPersist.alarm_vt = alarm_data.alarm_vt;
                toPersist.autopid = autopid;
                toPersist.filter = filter;
                write_memory(toPersist);
                lastSave = millis();
            }

            if (time2 > lastShowSensor + TIME_SHOW) {
                tft_draw(tft, sensorData, mech_vent.status, drawing_cycle, alarm_data);
                lastShowSensor = time2;
            }

            if (time2 > sensorData.last_read_sensor + TIME_SENSOR) {
                read_sensor(ads, sensorData, calibration_data.vzero, filter);
                eval_max_min_pressure(sensorData);
            }//Read Sensor

            if (vent_status->cycle != vent_status->last_cycle) {
                Serial.println("Last press max " + String(vent_status->last_max_pressure));
                Serial.println("Last press min " + String(vent_status->last_min_pressure));
                Serial.println("alarm_pre state " + String(alarm_data.alarm_state));
                alarm_data = check_alarms(mech_vent.status, alarm_data);
                Serial.println("alarm_post state " + String(alarm_data.alarm_state));
                vent_status->last_cycle = vent_status->cycle;
                vent_status->update_display = true;
                show_power_led();

                #ifdef TEMP_TEST
                if (time2 > lastReadTemp + TIME_READ_TEMP) {
                    lastReadTemp = time2;
                    sensors.requestTemperatures();
                    temp=sensors.getTempCByIndex(0);
                    //Serial.println ("Temp: " + String(temp));
                }
                tft.fillRect(200, 100, 20, 40, ILI9341_BLUE);
                print_float(tft, 100, 200, temp);
                #endif //TEMP_TEST
            }//change cycle

            if (vent_status->update_display) {
                display_lcd(mech_vent.status, mech_vent.config);
                last_update_display = millis();
                vent_status->update_display = false;
            }

            if (update_options) {
                update_config(mech_vent);
                update_options = false;
            }

            if (show_changed_options && ((millis() - last_update_display) > TIME_UPDATE_DISPLAY)) {
                display_lcd(mech_vent.status, mech_vent.config);
                last_update_display = millis();
                show_changed_options = false;
            }

            buzzer_state = set_alarm_buzzer(alarm_data.alarm_state, buzzer_state);
        }
    } else {
        if (put_to_sleep) {
            digitalWrite(PIN_LCD_EN, HIGH);
            put_to_sleep = false;
            print_bat_time = time2;
            print_bat(tft, FAC);
            lcd.clear();
            digitalWrite(LCD_SLEEP, LOW);
            digitalWrite(TFT_SLEEP, LOW);
            stop(mech_vent);
            //digitalWrite(PIN_BUZZER, !BUZZER_LOW); //Buzzer inverted
        }
        if (time2 > print_bat_time + 5000) {
            print_bat(tft, FAC);
            print_bat_time = time2;
        }
        check_bck_state();
    }

    #ifdef BAT_TEST
    if ( time2 > lastShowBat + TIME_SHOW_BAT ) {
        lastShowBat = time2;
        Serial.println("last show bat " + String(lastShowBat));
        float level = calc_bat(5, FAC);
        Serial.println(String(time2)+", " +String(level));
    }
    #endif//BAT_TEST
}//LOOP

void timer1Isr(void) {
    update(mech_vent, sensorData);
}

void timer3Isr(void) {
    stepper->run();
}
