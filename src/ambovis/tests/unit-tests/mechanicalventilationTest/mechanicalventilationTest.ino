#include <AUnit.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "../../../MechanicalVentilation.h"

Mechanical_Ventilation_t mech_vent;
AccelStepper* accelStepper;
SensorData sensorData;

test(start_and_set_initial_cycle_time) {
    mech_vent.config.respiratory_rate = 20;
    mech_vent.config.perc_IE = 2;
    start(mech_vent);
    assertEqual(mech_vent.status.running, true);
    assertEqual(mech_vent.status.timeout_cycle, 3000.); // 60000.0 / respiratory_rate
    assertEqual(mech_vent.status.time_ins, (unsigned int) 1000); // 30000 / (perc-IE + 1)
    assertEqual(mech_vent.status.time_exp, (unsigned int) 2000); // 30000 - time_ins
}

test(update_config_should_update_cycle_time) {
    //prepare
    mech_vent.config.respiratory_rate = 26;
    mech_vent.config.perc_IE = 3;

    //actual call
    update_config(mech_vent);

    //verify
    float expected = 60000. / (float) mech_vent.config.respiratory_rate;
    assertEqual(mech_vent.status.timeout_cycle, expected);
    unsigned int expected_time_ins = expected / 4;
    assertEqual(mech_vent.status.time_ins, expected_time_ins);
    unsigned int expected_time_exp = expected - expected_time_ins;
    assertEqual(mech_vent.status.time_exp, expected_time_exp);
}

test(stop_mech_vent) {
    stop(mech_vent);
    assertEqual(mech_vent.status.running, false);
}

test(search_home_position_ccw) {
    // Accel Stepper max speed by default is 1.
    accelStepper->setMaxSpeed(3000.);
    accelStepper->setSpeed(STEPPER_HOMING_SPEED);
    assertEqual(accelStepper->speed(), (float) STEPPER_HOMING_SPEED);

    digitalReadValue(PIN_ENDSTOP, LOW); // force returning 0, will not enter the while loop
    ccw_search_home(accelStepper);
    assertEqual(accelStepper->currentPosition(), 0L);
    assertEqual(accelStepper->speed(), 0.); // When current position is set, the speed is reset to 0.
}

test(search_home_position_cw) {
    digitalReadValue(PIN_ENDSTOP, HIGH);
    cw_search_home(accelStepper);
    assertEqual(accelStepper->currentPosition(), 40L);
    assertEqual(accelStepper->speed(), 0.); // When current position is set, the speed is reset to 0.
}

test(update_vent_when_status_is_Homing) {
    //prepare
    mech_vent.config.respiratory_rate = 20;
    mech_vent.config.perc_IE = 2;
    update_config(mech_vent);
    mech_vent.status.current_state = State_Homing;
    // Assume millis() in the update function will return almost the same value as here.
    mech_vent.status.start_cycle_time_ms = millis() - 100;

    //actual call
    update(mech_vent, sensorData);

    //verify
    assertMoreOrEqual(mech_vent.status.ms_timer_cnt, (unsigned long) 100); // millis() - start_cycle_time_ms
    byte expected_cycle_pos = byte(mech_vent.status.ms_timer_cnt / 3000. * 127.0);
    assertEqual(mech_vent.status.cycle_pos, expected_cycle_pos);
    assertEqual(mech_vent.status.current_state, Init_Insufflation);
}

test(update_vent_when_status_is_Init_Insufflation) {
    //prepare
    mech_vent.config.respiratory_rate = 20;
    mech_vent.config.perc_IE = 2;
    mech_vent.config.stepper_speed_max = 10;
    mech_vent.stepper->setMaxSpeed(100);
    start(mech_vent);
    mech_vent.status.current_state = Init_Insufflation;
    mech_vent.status.c_dyn_pass[0] = 1.;
    mech_vent.status.c_dyn_pass[1] = 2.;
    mech_vent.status.c_dyn_pass[2] = 3.;
    mech_vent.status.ml_last_ins_vol = 100;

    //prepare sensor data
    sensorData.pressure_min = 50;
    sensorData.pressure_max = 100;
    sensorData.ml_ins_vol = 1.9;
    sensorData.ml_exs_vol = 3.8;

    //actual call
    update(mech_vent, sensorData);

    //verify
    assertEqual(mech_vent.status.current_state, State_Insufflation);
    assertEqual(mech_vent.status.last_max_pressure, 100);
    assertEqual(mech_vent.status.last_min_pressure, 50);
    assertEqual(mech_vent.stepper->targetPosition(), (long) -STEPPER_HIGHEST_POSITION);
    assertNotEqual(mech_vent.stepper->speed(), 10.); // stepper moveTo recalculates the speed

    //verify with sensor data
    assertEqual(mech_vent.status.last_max_pressure, 100);
    assertEqual(mech_vent.status.last_min_pressure, 50);
    assertEqual(mech_vent.status.c_dyn_pass[2], 2); // ml_last_ins_vol / (pressure_max - pressure_min)
    assertEqual(mech_vent.status.c_dyn, (float) (7./3.)); //c_dyn_pass[0] + c_dyn_pass[1] + c_dyn_pass[2] / 3.
    assertEqual(mech_vent.status.ml_last_ins_vol, int(1.9)); // ml_ins_vol
}

test(update_vent_when_status_is_State_Insufflation) {
    //prepare
    mech_vent.config.respiratory_rate = 20;
    mech_vent.config.perc_IE = 2;
    start(mech_vent);
    mech_vent.status.current_state = State_Insufflation;

    // Assume millis() in the update function will return almost the same value as here.
    mech_vent.status.start_cycle_time_ms = millis() - mech_vent.status.time_ins - 1;

    //actual call
    update(mech_vent, sensorData);

    //verify
    assertEqual(mech_vent.status.current_state, Init_Exufflation);
}

test(update_vent_when_status_is_State_Insufflation_and_is_not_time_for_Exufflation) {
    //prepare
    mech_vent.config.respiratory_rate = 20;
    mech_vent.config.perc_IE = 2;
    start(mech_vent);
    mech_vent.status.current_state = State_Insufflation;

    // Assume millis() in the update function will return almost the same value as here.
    mech_vent.status.start_cycle_time_ms = millis() - mech_vent.status.time_ins + 10;

    //actual call
    update(mech_vent, sensorData);

    //verify
    assertEqual(mech_vent.status.current_state, State_Insufflation);
}

test(update_vent_when_status_is_Init_Exufflation) {
    //prepare
    mech_vent.config.respiratory_rate = 20;
    mech_vent.config.perc_IE = 2;
    start(mech_vent);
    mech_vent.status.current_state = Init_Exufflation;

    //actual call
    update(mech_vent, sensorData);

    //verify
    assertEqual(mech_vent.stepper->speed(), (float) STEPPER_SPEED_EXSUFF);
    assertEqual(mech_vent.stepper->targetPosition(), (long) STEPPER_LOWEST_POSITION);
    assertEqual(mech_vent.status.current_state, State_Exufflation);
}

test(update_vent_when_status_is_State_Exufflation_and_is_time_for_Insufflation) {
    //prepare
    mech_vent.config.respiratory_rate = 20;
    mech_vent.config.perc_IE = 2;
    start(mech_vent);
    mech_vent.status.current_state = State_Exufflation;
    unsigned long cycle = 1000;
    mech_vent.status.cycle = cycle;

    // Assume millis() in the update function will return almost the same value as here.
    mech_vent.status.start_cycle_time_ms = millis() - mech_vent.status.time_exp - 10;
    mech_vent.stepper->setCurrentPosition(STEPPER_LOWEST_POSITION);

    //actual call
    update(mech_vent, sensorData);

    //verify
    assertEqual(mech_vent.status.current_state, Init_Insufflation);
    assertEqual(mech_vent.status.cycle, cycle + 1);
}

test(update_vent_when_status_is_State_Exufflation_and_position_is_not_the_lowest) {
    //prepare
    mech_vent.config.respiratory_rate = 20;
    mech_vent.config.perc_IE = 2;
    start(mech_vent);
    mech_vent.status.current_state = State_Exufflation;
    unsigned long cycle = 1000;
    mech_vent.status.cycle = cycle;

    // Assume millis() in the update function will return almost the same value as here.
    mech_vent.status.start_cycle_time_ms = millis() - mech_vent.status.time_exp + 10;
    mech_vent.stepper->setCurrentPosition(0);

    //actual call
    update(mech_vent, sensorData);

    //verify
    assertEqual(mech_vent.status.current_state, State_Exufflation);
    assertEqual(mech_vent.status.cycle, cycle);
}

void setup() {
    delay(1000); // wait for stability on some boards to prevent garbage Serial
    Serial.begin(115200); // ESP8266 default of 74880 not supported on Linux
    while(!Serial); // for the Arduino Leonardo/Micro only

    Serial.println(F("MechanicalVentilation tests:"));
    Serial.println(F("----"));
    accelStepper = new AccelStepper(AccelStepper::DRIVER,
                                     PIN_STEPPER_STEP,
                                     PIN_STEPPER_DIRECTION);
    mech_vent.stepper = accelStepper;
}

void loop() {
    aunit::TestRunner::run();
}

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
bool put_to_sleep;
bool sleep_mode;
unsigned long time2;
bool wake_up;
EpoxyEepromAvr EEPROM;