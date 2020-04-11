#include "src/Adafruit_BMP280/Adafruit_BMP280.h"
//#include "LiquidCrystal_I2C.h"
#include <LiquidCrystal.h>
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

Adafruit_BMP280 bmp; // I2C
//     Adafruit_BMP280 bmp(
//    PIN_BME_CS1,
//    PIN_BME_MOSI,
//    PIN_BME_MISO,
//    PIN_BME_SCK
//    );
//
float presion,presion_0; // Almacena la presion atmosferica (Pa)
float temperatura; // Almacena la temperatura (oC)
int altitud; // Almacena la altitud (m) (se puede usar variable float)

float presion_mpx;
#define DEFAULT_PA_TO_CM_H20 0.0102F

//LiquidCrystal_I2C lcd(0x3F, 20, 4);
void setup() {
  Serial.begin(9600); // Inicia comunicacion serie

  //LUCIANO-------------
  lcd.begin(20, 4);
  //  //lcd.backlight();
  lcd.clear();


  if (!bmp.begin(0x76))//I2C
    //if (!bmp.begin()) //SP
    Serial.println("Could not find a valid BMP280 sensor, check wiring!"); // Inicia el sensor
  presion_0 = bmp.readPressure();
}


void loop() {

  // Lee valores del sensor:
  presion = (bmp.readPressure()-presion_0)*DEFAULT_PA_TO_CM_H20;
  temperatura = bmp.readTemperature();
  altitud = bmp.readAltitude (1015); // Ajustar con el valor local

  // Imprime valores por el serial monitor:
  Serial.print(F("Presion: "));
  Serial.print(presion);
  Serial.print(" hPa");
  Serial.print("\t");
  Serial.print(("Temp: "));
  Serial.print(temperatura);
  Serial.print(" *C");
  Serial.print("\t");
  Serial.print("Altitud (aprox): ");
  Serial.print(altitud);
  Serial.println(" m");

  presion_mpx = (analogRead(A1) / 1024. - 0.04) / 0.18 * 1000 * DEFAULT_PA_TO_CM_H20;
  
  
  
  lcd.setCursor(0, 0);
  lcd.print("dp [cmH20]");
  Serial.print("Analogico");Serial.println(analogRead(A1));
  lcd.setCursor(0, 1);
  lcd.print("BMP280 : "); lcd.print(presion);
  lcd.setCursor(0, 2);
  lcd.print("MPX5050: "); lcd.print(presion_mpx);

  delay(50);
}
