/** Calculation functions
 *
 * @file calc.h
 *
 *
 */
#ifndef CALC_H
#define CALC_H

#include "defaults.h"
#include "math.h"

 //constants - these will not change  
 const float tubeArea1 = 4.154e-4; // area of pneumotach first section  
 const float tubeArea2 = 1.5904e-5; // area of pneumotach second section  
 const float airDensity = 1.225;  

/**
 * @brief estima el volumen tidal en función de estatura y sexo, en ml.
 *
 * @param estatura en cm, del paciente
 * @param sexo 0: varón, 1: mujer, sexo del paciente
 * @return *volumenTidal volumen tidal estimado, en mililitros
 */
void calcularVolumenTidal(int* volumenTidal, int estatura, int sexo);

/**
 * @brief calcula los tiempos de ciclo, inspiratorio y espiratorio, en seg.
 *
 * Calcula a partir de las respiraciones por minuto, los tiempos de ciclo,
 * inspiratorio y espiratorio, y las velocidades uno y dos.
 * @param speedIns TODO: explicación?
 * @param speedEsp TODO: explicación?
 * @param tIns tiempo de inspiracion, en segundos
 * @param tEsp tiempo de espiracion, en segundos
 * @param tCiclo tiempo de ciclo, en segundos
 * @param pasosPorRevolucion TODO: explicación?
 * @param microStepper TODO: explicación?
 * @param porcentajeInspiratorio fraccion del ciclo en la que se inspira, tIns/tCiclo*100
 * @param rpm respiraciones por minuto
 */
void calcularCicloInspiratorio(float* speedIns, float* speedEsp,
                               float* tIns, float* tEsp, float* tCiclo,
                               int pasosPorRevolucion, int microStepper,
                               int porcentajeInspiratorio, int rpm);

/**
 * @brief estima el caudal a partir de la diferencia de presión
 *
 * @param pressure1 presión a un lado
 * @param pressure2 presión a otro lado
 * @param flux caudal resultante
 */
void calcularCaudal(float* pressure1, float* pressure2, float* flux);

void calcularCaudalVenturi(float dp, float* flux);

void refreshWatchDogTimer();

#endif // CALC_H
