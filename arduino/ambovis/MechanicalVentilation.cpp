#include "MechanicalVentilation.h"

void initCycleTimes(MechanicalVentilation& mechanicalVentilation) {
    VentilationStatus status = mechanicalVentilation.ventilationStatus;
    VentilationConfig config = mechanicalVentilation.ventilationConfig;

    status.timeoutCycle = ((float) 60) * 1000.0f / (float) config.respiratoryRate;
    status.timeIns = status.timeoutCycle / (float(config.percIE + 1));
    status.timeExp = status.timeoutCycle - status.timeIns;
}

void start(MechanicalVentilation& mechanicalVentilation) {
    digitalWrite(PIN_STEPPER, HIGH);
    mechanicalVentilation.ventilationStatus.running = true;
    initCycleTimes(mechanicalVentilation);
}

void stop(MechanicalVentilation& mechanicalVentilation) {
    mechanicalVentilation.ventilationStatus.running = false;
    digitalWrite(PIN_STEPPER, LOW);
}

void newInsufflationActions(VentilationStatus& status, SensorData& sensorData) {
    float last_pressure_max = sensorData.max_pressure;
    float last_pressure_min = sensorData.min_pressure;
    sensorData.max_pressure = 0;
    sensorData.min_pressure = 60;

    status.cDynPass[0] = status.cDynPass[1];
    status.cDynPass[1] = status.cDynPass[2];
    status.cDynPass[2] = status.mlLastInsVol/(last_pressure_max - last_pressure_min);

    status.cDyn = (status.cDynPass[0] + status.cDynPass[1] + status.cDynPass[2]) / 3.;
    status.mlLastInsVol = int(sensorData.mlInsVol);
    status.mlLastExpVol = int(fabs(sensorData.mlExsVol));

    sensorData.mlInsVol=0.;
    sensorData.mlExsVol=0.;
}

void update(MechanicalVentilation& mechanicalVentilation) {

    VentilationStatus status = mechanicalVentilation.ventilationStatus;

    if ( !status.running ) {
        return;
    }

    bool endedWhileMoving = false;
    status.msTimerCnt = millis() - status.startCycleTimeInMs;
  
    float extra_time = 0.;
    if (status.currentState == State_Exufflation) {
        extra_time = status.timeIns;
    }
    status.cyclePosition = byte( (float) ( (status.msTimerCnt + extra_time) / (float) status.timeoutCycle * 127.0f) );

    status.newInsufflation = false;
    switch (status.currentState) {
        case Init_Insufflation: {

            status.newInsufflation = true;
            status.startCycleTimeInMs = millis();

            mechanicalVentilation.stepper->setSpeed(STEPPER_SPEED_MAX);
            mechanicalVentilation.stepper->moveTo(-STEPPER_HIGHEST_POSITION);
            mechanicalVentilation.stepper->setAcceleration(STEPPER_ACCEL_MAX);

            status.currentState = State_Insufflation;
            status.updateDisplay = true;

        }
        break;
        case State_Insufflation: {
            if (status.msTimerCnt > status.timeIns) {
                if (mechanicalVentilation.stepper->distanceToGo() != 0 ) {
                    endedWhileMoving = true;
                }
                status.currentState = Init_Exufflation;
            }
        }
        break;
        case Init_Exufflation: {
            status.endedWhileMoving = endedWhileMoving;
            status.startCycleTimeInMs = millis();

            mechanicalVentilation.stepper->setAcceleration(STEPPER_ACCEL_MAX);
            mechanicalVentilation.stepper->setSpeed(STEPPER_SPEED_EXSUFF);
            mechanicalVentilation.stepper->moveTo(STEPPER_LOWEST_POSITION);
            status.currentState = State_Exufflation;
        }
        break;
        case State_Exufflation: {
            if (status.msTimerCnt > status.timeExp) {
                if (mechanicalVentilation.stepper->currentPosition() == STEPPER_LOWEST_POSITION) {
                    status.currentState = Init_Insufflation;
                    status.startCycleTimeInMs = millis();
                    status.cycleNum += 1;
                }
            }
        }
        break;
        case State_Homing: {
             status.currentState = Init_Insufflation;
        }
        break;
        case State_Error: break;
    }

}

void update_config(MechanicalVentilation& mechanicalVentilation) {
    initCycleTimes(mechanicalVentilation);
}
