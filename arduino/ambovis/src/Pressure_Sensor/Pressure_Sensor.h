#ifndef _PRESSURE_SENSOR_H
#define _PRESSURE_SENSOR_H

//@TODO: HEREDAR ELMPX DE ESTE
#include "Arduino.h"

class Pressure_Sensor
{
	int sensorpin;
	
	int sampleNumber; // variable to store the sample number   
	int sensorPin; // select the input pin for the Pressure Sensor  
 int sensorValue = 0; // variable to store the Raw Data value coming from the sensor  
 float diffPressure = 0; // variable to store converted kPa value   
 //float dV;
 
	public:
	
	Pressure_Sensor(){}
	Pressure_Sensor (int);
	void read();
	float get_dp();
	float get_dV();
	~Pressure_Sensor(){}	
};

#endif