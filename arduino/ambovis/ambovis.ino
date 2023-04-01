#include "src/TimerOne/TimerOne.h"
#include "src/TimerThree/TimerThree.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

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

bool show_changed_options = false; //Only for display
bool update_options = false;

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
byte menu_number = 0;
byte alarm_max_pressure = 35;
byte alarm_peep_pressure = 5;
int alarm_vt = 200;

unsigned long lastButtonPress;
byte encoderPos = 1; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
byte oldEncPos = 1; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
byte max_sel, min_sel; //According to current selection
bool isitem_sel =false;

Mechanical_Ventilation_t mech_vent;
AccelStepper* stepper;

void init_display_tft(Adafruit_ILI9341& tft);
void process_sensor_data(SensorData& sensorData);

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

    mech_vent.config.vent_mode = VENTMODE_MAN;
    mech_vent.config.respiratory_rate = DEFAULT_RPM;
    mech_vent.config.perc_IE = 2;
    mech_vent.config.peak_ins_pressure = DEFAULT_PEAK_INSPIRATORY_PRESSURE;
    mech_vent.config.peak_exp_pressure = DEFAULT_PEAK_ESPIRATORY_PRESSURE;
    mech_vent.config.perc_volume = 100;
    mech_vent.config.stepper_speed_max = STEPPER_MICROSTEPS * 1500;
    mech_vent.config.stepper_accel_max= STEPPER_MICROSTEPS * 1500;
    mech_vent.status.running = true;
    mech_vent.status.current_state = State_Homing;
    mech_vent.status.start_cycle_time_ms = 0;
    mech_vent.status.ended_while_moving = false;
    Menu_inic menuini(mech_vent.config);

    lcd.clear();
    writeLine(1, "Desconecte flujo", 0);
    writeLine(2, "y presione ok ", 0);
    waitForFluxDisconnected();

    stepper = new AccelStepper(
        AccelStepper::DRIVER,
        PIN_STEPPER_STEP,
        PIN_STEPPER_DIRECTION);

    mech_vent.stepper = stepper;
    start(mech_vent);
    update(mech_vent, sensorData);

    tft.fillScreen(ILI9341_BLACK);
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
    peep_fac = -(pf_max - pf_min) / 15.* mech_vent.status.last_min_pressure + pf_max;

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

    time2 = millis();
    if (!sleep_mode) {
        if (wake_up) {
            digitalWrite(TFT_SLEEP, HIGH);
            digitalWrite(LCD_SLEEP, HIGH);
            init_display();
            display_lcd(mech_vent.status, mech_vent.config);
            tft.begin();
            tft.fillScreen(ILI9341_BLACK);
            wake_up = false;
            start(mech_vent);
        }

        buzzmuted = check_buzzer_mute(last_mute, buzzmuted, mute_count, time2);
        check_encoder(mech_vent.status, mech_vent.config);
        Ventilation_Status_t* vent_status = &mech_vent.status;

        if (calibration_run) {
            if (time2> sensorData.last_read_sensor + TIME_SENSOR) {
                process_sensor_data(sensorData);
                vcorr_count++;
                verror_sum += (sensorData.voltage - 0.04 * sensorData.v_level);
            }

            if (vent_status->cycle != vent_status->last_cycle) {
                lcd.clear();
                writeLine(1, "Calibracion flujo", 0);
                writeLine(2, "Ciclo: " + String(calib_cycle+1) + "/" + String(CALIB_CYCLES), 0);

                vent_status->last_cycle = vent_status->cycle;

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
            if (time2 > lastSave + TIME_SAVE) {
                SystemConfiguration_t toPersist;
                toPersist.last_cycle = vent_status->last_cycle;
                toPersist.alarm_vt = alarm_vt;
                toPersist.autopid = autopid;
                toPersist.filter = filter;
                write_memory(toPersist);
                lastSave = millis();
            }

            if (time2 > lastShowSensor + TIME_SHOW) {
                tft_draw(tft, sensorData, mech_vent.status, drawing_cycle, fac);
                lastShowSensor = time2;
            }

            if (time2 > sensorData.last_read_sensor + TIME_SENSOR) {
                process_sensor_data(sensorData);
                check_pip_and_peep(sensorData);
            }//Read Sensor

            if (vent_status->cycle != vent_status->last_cycle) {
                is_alarm_vt_on = calc_alarm_vt_is_on(vent_status->ml_last_ins_vol,
                                                     vent_status->ml_last_ins_vol,
                                                     alarm_vt);
                alarm_state = get_alarm_state(vent_status->last_max_pressure,
                                              vent_status->last_min_pressure,
                                              alarm_max_pressure,
                                              alarm_peep_pressure);

                vent_status->last_cycle = vent_status->cycle;
                vent_status->update_display = true;

                if (!digitalRead(PIN_POWEROFF)) {
                    digitalWrite(YELLOW_LED, HIGH);
                } else {
                    digitalWrite(YELLOW_LED, LOW);
                }
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

            set_alarm_buzzer(alarm_state, buzzmuted, timebuzz, isbuzzeron);
        }
    } else {
        if (put_to_sleep) {
            digitalWrite(PIN_LCD_EN, HIGH);
            put_to_sleep = false;
            print_bat_time = time2;
            print_bat(tft, fac);
            lcd.clear();
            digitalWrite(LCD_SLEEP, LOW);
            digitalWrite(TFT_SLEEP, LOW);
            stop(mech_vent);
            //digitalWrite(PIN_BUZZER, !BUZZER_LOW); //Buzzer inverted
        }
        if (time2 > print_bat_time + 5000) {
            print_bat(tft, fac);
            print_bat_time = time2;
        }
        check_bck_state();
    }

    #ifdef BAT_TEST
    if ( time2 > lastShowBat + TIME_SHOW_BAT ) {
        lastShowBat = time2;
        Serial.println("last show bat " + String(lastShowBat));
        float level = calc_bat(5, fac);
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

void init_display_tft(Adafruit_ILI9341& tft) {
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_BLUE);
    tft.setTextSize(4);
    tft.setCursor(10, 40);     tft.println("RespirAR");
    tft.setCursor(10, 80);     tft.println("FIUBA");
}

void process_sensor_data(SensorData& sensorData) {
    int16_t adc0 = ads.readADC_SingleEnded(0);
    int pressure = analogRead(PIN_PRESSURE);
    int mpx_lev = analogRead(PIN_MPX_LEV);

    convert_sensor_data(adc0, pressure, mpx_lev, sensorData);
    float p_dpt = get_dpt(sensorData.voltage, sensorData.v_level, vzero);
    sensorData.flux = find_flux(p_dpt, dp, po_flux);

    sensorData.flow_f = get_flow(sensorData, filter);
    update_vol(sensorData, millis());
    sensorData.last_read_sensor = millis();
}