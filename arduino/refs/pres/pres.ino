#include "src/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#include "src/Pressure_Sensor/Pressure_Sensor.h"  //LUCIANO: MPX5050DP

Pressure_Sensor psen;
LiquidCrystal_I2C lcd(0x3F, 20, 4);
float flux;
 //constants - these will not change  
 const float tubeArea1 = 4.154e-4; // area of pneumotach first section  
 const float tubeArea2 = 1.5904e-5; // area of pneumotach second section  
 const float airDensity = 1.225;  
 
void calcularCaudalVenturi(float diffPressure, float* flux) {
  //A1>A2
  //Q=A1A2strt(2(p2-p1)/rho(A2^2-A1^2))
  *flux=tubeArea1*tubeArea2*sqrt(2.*fabs(diffPressure)/(airDensity*(tubeArea1*tubeArea1-tubeArea2*tubeArea2) ))*1000;
  if (diffPressure < 0) {  
   diffPressure = diffPressure * (-1.);  
   *flux=-*flux;
//   *flux = *flux / tubeArea2;  
 } else {  
  }  
}
void writeLine(int line, String message = "", int offsetLeft = 0)
{
  lcd.setCursor(0, line);
  lcd.print("");
  lcd.setCursor(offsetLeft, line);
  lcd.print(message);
}


void setup()
{
  Serial.begin(9600);
	  lcd.begin();
  //lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  
	psen=Pressure_Sensor(A0);
}

float dv;

void loop()
{
  dv=analogRead(A0) * 5. / 1024.;
	calcularCaudalVenturi(psen.get_dp(), &flux);
	writeLine(0, "dV: " + String(dv) + " V");
	writeLine(1, "flow: "+String(flux*1000));
 Serial.print(dv);Serial.print(" ");Serial.println(flux);
}
