#ifndef _CALC_SOURCE_
#define _CALC_SOURCE_

#include "calc.h"


/**
 * @brief estima el volumen tidal en función de estatura y sexo, en ml.
 *
 * @param estatura en cm, del paciente
 * @param sexo 0: varón, 1: mujer, sexo del paciente
 * @return *volumenTidal volumen tidal estimado, en mililitros
 */
 
void calcularVolumenTidal(int* volumenTidal, int estatura, int sexo) {
  float peso0, pesoIdeal, volumenEstimado;
  if (sexo == 0) { // Varón
    peso0 = 50.0;
  } else if (sexo == 1) { // Mujer
    peso0 = 45.5;
  }
  pesoIdeal = peso0 + 0.91 * (estatura - 152.4); // en kg

  *volumenTidal = int(round(pesoIdeal * DEFAULT_ML_POR_KG_DE_PESO_IDEAL));
}

void calcularCicloInspiratorio(float* speedIns, float* speedEsp,
                               float* tIns, float* tEsp, float* tCiclo,
                               int pasosPorRevolucion, int microStepper,
                               int porcentajeInspiratorio, int rpm) {
  *tCiclo = 60 / rpm; // Tiempo de ciclo en segundos
  *tIns = *tCiclo * porcentajeInspiratorio/100;
  *tEsp = *tCiclo - *tIns;

  *speedIns = (pasosPorRevolucion * microStepper / 2) / *tIns; // TODO: unidades?
  *speedEsp = (pasosPorRevolucion * microStepper / 2) / *tEsp; // TODO: unidades?
}


/**
 * @brief estima el caudal a partir de la diferencia de presión
 *
 * @param pressure1 presión a un lado
 * @param pressure2 presión a otro lado
 * @param flux caudal resultante
 */
void calcularCaudal(float* pressure1, float* pressure2, float* flux) {
  *flux = (*pressure1 - *pressure2) * DEFAULT_PRESSURE_V_FLUX_K1;
}

void calcularCaudalVenturi(float diffPressure, float* flux) {
  //A1>A2
  //Q=A1A2strt(2(p2-p1)/rho(A2^2-A1^2))
  *flux=tubeArea1*tubeArea2*sqrt(2.*fabs(diffPressure)/(airDensity*(tubeArea1*tubeArea1-tubeArea2*tubeArea2) ))*1000;
  if (diffPressure < 0) {  
   diffPressure = diffPressure * (-1.);  
   *flux=-*flux;
   
//   //calculate volumetric flow rate for Inhalation  
//   *flux = tubeArea2 * (sqrt((2 / airDensity) * (diffPressure / (1 - sqrt(tubeArea2 / tubeArea1)))));  
//   
//   //calculate velocity of flow   
//   *flux = *flux / tubeArea2;  
 } else {  
//   //calculate volumetric flow rate for Exhalation  
//   *flux = tubeArea1 * (sqrt((2 / airDensity) * (diffPressure / (sqrt(tubeArea1 / tubeArea2) - 1))));  
//   
//   //calculate velocity of flow   
//   *flux = *flux / tubeArea1;  
  }  
}

/**
 * @brief Refresca el WDT (Watch Dog Timer)
 */
void refreshWatchDogTimer() {
  //TODO implementar
}

#endif
