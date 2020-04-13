#include "Pressure_Sensor.h"

Pressure_Sensor::Pressure_Sensor(int pin)
{
	sensorPin = pin;
	pinMode(sensorPin, INPUT);
}

void Pressure_Sensor::read()
{
	sensorValue = analogRead(sensorPin);  

  // initial value   
  sensorValue = sensorValue - (48);  
   
  // increment sample counter   
  sampleNumber++;  
	
}

float Pressure_Sensor::get_dV()
{

	
}

float Pressure_Sensor::get_dp()
{

  	sensorValue = analogRead(sensorPin);  

  // initial value   
  //sensorValue = sensorValue - 48;  
   
  // increment sample counter   
  sampleNumber++;  
  
  // map the Raw data to kPa  
  //diffPressure = map(sensorValue, 0, 1023, -2000, 2000); 
  //Vout = VS*(0.018*P+0.04) ± ERROR
  //VS = 5.0 Vdc
  //EL 0.04 es el 48que aparece en 0
  //Por eso como yaesta restado no se tiene en cuenta 
  //0.04=48*/1024
  diffPressure=(sensorValue/1024.-0.0425)/0.18*1000;
  //Acc to datasheet
  //diffPressure=(sensorValue/1024.-0.04)/0.18*1000;
	//return sensorValue;
	return   diffPressure;
	
}