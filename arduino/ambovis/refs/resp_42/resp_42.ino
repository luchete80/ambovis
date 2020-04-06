 // MPX7002DP Test Code with conversion   
 // to volumetric flow rate and velocity   
 // of flow  
 //  
 // A.Lang - 2017  
   
 // This code exercises the MPX7002DP  
 // Pressure sensor connected to A0  
   
 //variables  
   
 int sampleNumber = 0; // variable to store the sample number   
 int sensorPin = A0; // select the input pin for the Pressure Sensor  
 int sensorValue = 0; // variable to store the Raw Data value coming from the sensor  
 float diffPressure = 0; // variable to store converted kPa value   
 float volumetricFlowExhale = 0; // variable to store volumetric flow rate value when Exhaling  
 float volumetricFlowInhale = 0; // variable to store volumetric flow rate value when Inhaling  
 float velocityFlowExhale = 0; // variable to store velocity of flow value when Exhaling  
 float velocityFlowInhale = 0; // variable to store velocity of flow value when Inhaling  
 float offset = 0; // variable to store offset differential pressure  
   
 //constants - these will not change  
 const float tubeArea1 = 0.003095; // area of pneumotach first section  
 const float tubeArea2 = 0.003094; // area of pneumotach second section  
 const float airDensity = 1.225;  
   
 void setup() {  
  // start serial port at 9600 bps and wait for port to open:  
  Serial.begin(9600);  
   
  pinMode(sensorPin, INPUT); // Pressure sensor is on Analogue pin 0  
   
  Serial.flush();  
  Serial.println();  
   
  //Header for CSV data  
   
  Serial.print("Sample Number,Differential Pressure, Volumetric Flow Rate (Exhale), Volumetric Flow Rate (Inhale), Velocity of Flow Exhale, Velocity of Flow Inhale,");  
  Serial.println();  
  Serial.print("       ,     Pa     ,   m^3/second       ,     m^3/second      ,      m/s     ,      m/s     ,");  
  Serial.println();  
   
 }  
   
 void loop() {  
   
  // read the value from the sensor:   
  sensorValue = analogRead(sensorPin);  
   
  // initial value   
  sensorValue = sensorValue - 48;  
   
  // increment sample counter   
  sampleNumber++;  
   
  // map the Raw data to kPa  
  diffPressure = map(sensorValue, 0, 1023, -2000, 2000);  
   
  // convert reading to a positive value  
  if (diffPressure < 0) {  
   diffPressure = diffPressure * -1;  
   
   //calculate volumetric flow rate for Inhalation  
   volumetricFlowInhale = tubeArea2 * (sqrt((2 / airDensity) * (diffPressure / (1 - sq(tubeArea2 / tubeArea1)))));  
   
   //calculate velocity of flow   
   velocityFlowInhale = volumetricFlowInhale / tubeArea2;  
  } else {  
   //calculate volumetric flow rate for Exhalation  
   volumetricFlowExhale = tubeArea1 * (sqrt((2 / airDensity) * (diffPressure / (sq(tubeArea1 / tubeArea2) - 1))));  
   
   //calculate velocity of flow   
   velocityFlowExhale = volumetricFlowExhale / tubeArea1;  
  }  
   
  // Print the results as comma separated values for easier processing  
  // in a spreadsheet program  
   
  Serial.print(sampleNumber);  
  Serial.print(",");  
  Serial.print(diffPressure);  
  Serial.print(",");  
  Serial.print(volumetricFlowExhale);  
  Serial.print(",");  
  Serial.print(volumetricFlowInhale);  
  Serial.print(",");  
  Serial.print(velocityFlowExhale);  
  Serial.print(",");  
  Serial.print(velocityFlowInhale);  
  Serial.print(",");  
  Serial.println();  
   
  // wait 100 milliseconds before the next loop  
  // for the analog-to-digital converter and  
  // pressure sensor to settle after the last reading:  
  delay(100);  
   
 }  