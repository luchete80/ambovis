#include "MechanicalVentilation.h"

void initCycleTimes(MechanicalVentilation& mechanicalVentilation) {
    VentilationStatus* status = &mechanicalVentilation.ventilationStatus;
    VentilationConfig* config = &mechanicalVentilation.ventilationConfig;

    status->timeoutCycle = ((float) 60) * 1000.0f / (float) config->respiratoryRate;
    status->timeIns = status->timeoutCycle / (float(config->percIE + 1));
    status->timeExp = status->timeoutCycle - status->timeIns;
    Serial.println(String(status->timeoutCycle) + " " + String(status->timeIns) + " " + String(status->timeExp));
}

void start(MechanicalVentilation& mechanicalVentilation) {
    mechanicalVentilation.ventilationStatus.running = true;
    initCycleTimes(mechanicalVentilation);
}

void stop(MechanicalVentilation& mechanicalVentilation) {
    Serial.println("Vent stop called ");
    mechanicalVentilation.ventilationStatus.running = false;
    digitalWrite(PIN_STEPPER, LOW);
}

void newInsufflationActions(VentilationStatus& status, SensorData& sensor) {
    status.lastMaxPressure = status.pressure_max;
    status.lastMinPressure = status.pressure_min;
    status.pressure_max = 0;
    status.pressure_min = 60;

    status.cDynPass[0] = status.cDynPass[1];
    status.cDynPass[1] = status.cDynPass[2];
    status.cDynPass[2] = status.mlLastInsVol/(status.lastMaxPressure - status.lastMinPressure);

    status.cDyn = (status.cDynPass[0] + status.cDynPass[1] + status.cDynPass[2]) / 3.;
    status.mlLastInsVol = int(sensor.ml_ins_vol);
    status.mlLastExpVol = int(fabs(sensor.ml_exs_vol));
    Serial.println("New Insufflation - Max Press " + String(status.lastMaxPressure) + " LastMlInsVols " + String(status.mlLastInsVol) + " ml ins vol " + String(sensor.ml_ins_vol));

    sensor.ml_ins_vol=0.;
    sensor.ml_exs_vol=0.;
}

void update(MechanicalVentilation& mechanicalVentilation, SensorData& sensor) {

    VentilationStatus* status = &mechanicalVentilation.ventilationStatus;
    VentilationConfig* config = &mechanicalVentilation.ventilationConfig;

    status->msTimerCnt = millis() - status->startCycleTimeInMs;
  
    float extra_time = 0.;
    if (status->currentState == State_Exufflation) {
        extra_time = status->timeIns;
    }
    status->cyclePosition = byte( (float) ( (status->msTimerCnt + extra_time) / (float) status->timeoutCycle * 127.0f) );

    switch (status->currentState) {
        case Init_Insufflation: {
            if ( !status->running ) {
                return;
            }
            newInsufflationActions(mechanicalVentilation.ventilationStatus, sensor);
            status->startCycleTimeInMs = millis();

            mechanicalVentilation.stepper->setSpeed(config->stepper_speed_max);
            mechanicalVentilation.stepper->moveTo(-STEPPER_HIGHEST_POSITION);
            mechanicalVentilation.stepper->setAcceleration(config->stepper_accel_max);
            status->currentState = State_Insufflation;
            status->updateDisplay = true;
            status->endingWhileMoving = false;

        }
        break;
        case State_Insufflation: {
            if (status->msTimerCnt > status->timeIns) {
                if (mechanicalVentilation.stepper->distanceToGo() != 0) {
                    status->endingWhileMoving = true;
                }
                status->currentState = Init_Exufflation;
            }
        }
        break;
        case Init_Exufflation: {
            status->endedWhileMoving = status->endingWhileMoving;
            status->startCycleTimeInMs = millis();
            mechanicalVentilation.stepper->setAcceleration(config->stepper_accel_max);
            mechanicalVentilation.stepper->setSpeed(STEPPER_SPEED_EXSUFF);
            mechanicalVentilation.stepper->moveTo(STEPPER_LOWEST_POSITION);
            status->currentState = State_Exufflation;
        }
        break;
        case State_Exufflation: {
            if (status->msTimerCnt > status->timeExp) {
                if (mechanicalVentilation.stepper->currentPosition() == STEPPER_LOWEST_POSITION) {
                    status->currentState = Init_Insufflation;
                    status->startCycleTimeInMs = millis();
                    status->cycleNum += 1;
                }
            }
        }
        break;
        case State_Homing: {
             status->currentState = Init_Insufflation;
        }
        break;
        case State_Error: break;
    }

}

void update_config(MechanicalVentilation& mechanicalVentilation) {
    Serial.println("Vent update called ");
    initCycleTimes(mechanicalVentilation);
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
        Serial.println("cw");
    }

    stepper->setCurrentPosition(STEPPER_LOWEST_POSITION);
}