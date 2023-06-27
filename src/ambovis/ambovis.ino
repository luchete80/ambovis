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
unsigned long last_show_sensor = 0;

//SENSORS
Adafruit_ADS1115 ads;
SensorData sensorData;
void process_sensor_data(SensorData& sensorData, float vzero);

//MUTE
Buzzer_State_t buzzer_state;

//ALARMS
AlarmData alarm_data;

//CALIBRATION
Calibration_Data_t calibration_data;

//PERSISTENCE
unsigned long last_save = 0;

#ifdef BAT_TEST
unsigned long lastShowBat = 0;
#endif

unsigned long time2;

//MENU
bool display_needs_update = false;

//KEYBOARD
Keyboard_data_t keyboard_data;
Menu_state_t menu_state;
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
unsigned long last_update_display;

//MECH VENTILATION
AccelStepper *stepper;
Mechanical_Ventilation_t mech_vent;
void search_home_position(AccelStepper* stepper);

void setup() {
    Serial.begin(115200);
    analogReference(INTERNAL1V1); // use AREF for reference voltage
    init_pins();
    show_power_led();
    init_display_lcd();

    writeLine(1, "RespirAR FIUBA", 4);
    writeLine(2, "v2.0.2", 8);

    init_display_tft(tft);
    init_sensor(ads);

    menu_state.menu_position = 0;
    menu_state.old_menu_position = 0;
    initialize_menu(keyboard_data, menu_state, mech_vent.config);
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
    display_lcd(menu_state, mech_vent.config, mech_vent.status, alarm_data);

    last_show_sensor = last_update_display = millis();
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
    lcd.clear();

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
            init_display_lcd();
            display_lcd(menu_state, mech_vent.config, mech_vent.status, alarm_data);
            init_display_tft(tft);
            start(mech_vent);
            wake_up = false;
        }

        buzzer_state = check_buzzer_mute(buzzer_state, time2);
        check_buttons(keyboard_data, time2);
        check_encoder(keyboard_data, menu_state, mech_vent.config, alarm_data, time2);
        Ventilation_Status_t* vent_status = &mech_vent.status;

        if (calibration_data.calibration_run) {
            if (time2 > sensorData.last_read_sensor + TIME_SENSOR) {
                process_sensor_data(sensorData, calibration_data.vzero);
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
            if (time2 > last_save + TIME_SAVE) {
                SystemConfiguration_t toPersist;
                toPersist.last_cycle = vent_status->last_cycle;
                toPersist.alarm_vt = alarm_data.alarm_vt;
                write_memory(toPersist);
                last_save = millis();
            }

            if (time2 > last_show_sensor + TIME_SHOW) {
                tft_draw(tft, sensorData, mech_vent.status, drawing_cycle, alarm_data);
                last_show_sensor = time2;
            }

            if (time2 > sensorData.last_read_sensor + TIME_SENSOR) {
                process_sensor_data(sensorData, calibration_data.vzero);
                eval_max_min_pressure(sensorData);
            }//Read Sensor

            if (vent_status->cycle != vent_status->last_cycle) {
//                Serial.println("Last press max " + String(vent_status->last_max_pressure));
//                Serial.println("Last press min " + String(vent_status->last_min_pressure));
//                Serial.println("alarm_pre state " + String(alarm_data.alarm_state));
                alarm_data = check_alarms(mech_vent.status, alarm_data);
//                Serial.println("alarm_post state " + String(alarm_data.alarm_state));
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
                display_lcd(menu_state, mech_vent.config, mech_vent.status, alarm_data);
                last_update_display = millis();
                vent_status->update_display = false;
            }

            if (menu_state.update_options) {
                update_config(mech_vent);
                menu_state.update_options = false;
            }

            if (menu_state.show_changed_options && ((millis() - last_update_display) > TIME_UPDATE_DISPLAY)) {
                display_lcd(menu_state, mech_vent.config, mech_vent.status, alarm_data);
                last_update_display = millis();
                menu_state.show_changed_options = false;
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
        check_bck_state(keyboard_data, time2);
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

void process_sensor_data(SensorData& sensorData, float vzero) {
    int16_t adc0 = ads.readADC_SingleEnded(0);
    int pressure = analogRead(PIN_PRESSURE);
    int mpx_lev = analogRead(PIN_MPX_LEV);

    convert_sensor_data(adc0, pressure, mpx_lev, sensorData);
    float p_dpt = get_dpt(sensorData.voltage, sensorData.v_level, vzero);
    sensorData.flux = find_flux(p_dpt, dp, po_flux, DP_LENGTH);

    sensorData.flow_f = get_flow(sensorData);
    update_vol(sensorData, millis());
//    Serial.println(String(sensorData.flow_f) + ", " + String(p_dpt) + ", " + String(sensorData.flux) + ", " + String(sensorData.ml_ins_vol) + ", " +String(sensorData.v_level));
    sensorData.last_read_sensor = millis();
}

void search_home_position(AccelStepper* stepper) {
    ccw_search_home(stepper);
    cw_search_home(stepper);
}
