#include "MechanicalVentilation.h"

void initCycleTimes(Mechanical_Ventilation_t& mech_vent) {
    Ventilation_Status_t* status = &mech_vent.status;
    Ventilation_Config_t* config = &mech_vent.config;

    status->timeout_cycle = ((float) 60) * 1000.0f / (float) config->respiratory_rate;
    status->time_ins = status->timeout_cycle / (float(config->perc_IE + 1));
    status->time_exp = status->timeout_cycle - status->time_ins;
    Serial.println(String(status->timeout_cycle) + " " + String(status->time_ins) + " " + String(status->time_exp));
}

void start(Mechanical_Ventilation_t& mech_vent) {
    mech_vent.status.running = true;
    initCycleTimes(mech_vent);
}

void stop(Mechanical_Ventilation_t& mech_vent) {
    mech_vent.status.running = false;
    digitalWrite(PIN_STEPPER, LOW);
}

void newInsufflationActions(Ventilation_Status_t& status, SensorData& sensor) {
    status.last_max_pressure = sensor.pressure_max;
    status.last_min_pressure = sensor.pressure_min;
    sensor.pressure_max = 0;
    sensor.pressure_min = 60;

    status.c_dyn_pass[0] = status.c_dyn_pass[1];
    status.c_dyn_pass[1] = status.c_dyn_pass[2];
    status.c_dyn_pass[2] = status.ml_last_ins_vol/(status.last_max_pressure - status.last_min_pressure);

    status.c_dyn = (status.c_dyn_pass[0] + status.c_dyn_pass[1] + status.c_dyn_pass[2]) / 3.;
    status.ml_last_ins_vol = int(sensor.ml_ins_vol);
    status.ml_last_exp_vol = int(fabs(sensor.ml_exs_vol));
    Serial.println("New Insufflation - Max Pressure " + String(status.last_max_pressure) + ", LastMlInsVols " + String(status.ml_last_ins_vol));

    sensor.ml_ins_vol=0.;
    sensor.ml_exs_vol=0.;
}

void update(Mechanical_Ventilation_t& mech_vent, SensorData& sensor) {

    Ventilation_Status_t* status = &mech_vent.status;
    Ventilation_Config_t* config = &mech_vent.config;

    status->ms_timer_cnt = millis() - status->start_cycle_time_ms;
  
    float extra_time = 0.;
    if (status->current_state == State_Exufflation) {
        extra_time = status->time_ins;
    }
    status->cycle_pos = byte( (float) ( (status->ms_timer_cnt + extra_time) / (float) status->timeout_cycle * 127.0f) );

    switch (status->current_state) {
        case Init_Insufflation: {
            if ( !status->running ) {
                return;
            }
            newInsufflationActions(mech_vent.status, sensor);
            status->start_cycle_time_ms = millis();

            mech_vent.stepper->setSpeed(config->stepper_speed_max);
            mech_vent.stepper->moveTo(-STEPPER_HIGHEST_POSITION);
            mech_vent.stepper->setAcceleration(config->stepper_accel_max);
            status->current_state = State_Insufflation;
            status->update_display = true;
            status->ending_while_moving = false;

        }
        break;
        case State_Insufflation: {
            if (status->ms_timer_cnt > status->time_ins) {
                if (mech_vent.stepper->distanceToGo() != 0) {
                    status->ending_while_moving = true;
                }
                status->current_state = Init_Exufflation;
            }
        }
        break;
        case Init_Exufflation: {
            status->ended_while_moving = status->ending_while_moving;
            status->start_cycle_time_ms = millis();
            mech_vent.stepper->setAcceleration(config->stepper_accel_max);
            mech_vent.stepper->setSpeed(STEPPER_SPEED_EXSUFF);
            mech_vent.stepper->moveTo(STEPPER_LOWEST_POSITION);
            status->current_state = State_Exufflation;
        }
        break;
        case State_Exufflation: {
            if (status->ms_timer_cnt > status->time_exp) {
                if (mech_vent.stepper->currentPosition() == STEPPER_LOWEST_POSITION) {
                    status->current_state = Init_Insufflation;
                    status->start_cycle_time_ms = millis();
                    status->cycle += 1;
                }
            }
        }
        break;
        case State_Homing: {
             status->current_state = Init_Insufflation;
        }
        break;
        case State_Error: break;
    }

}

void update_config(Mechanical_Ventilation_t& mech_vent) {
    initCycleTimes(mech_vent);
}

void search_home_position(AccelStepper* stepper) {
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