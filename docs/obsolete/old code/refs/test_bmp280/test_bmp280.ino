//http://www.playbyte.es/electronica/arduino/sensor-presion-atmosferica-gy-bmp280/

/*################## modulo BMP280 ########################
  Filename: BMP280_Ej1.ino
  Descripción: Modulo Presion y Temperatura
  Autor: Jose Mª Morales
  Revisión: 3-03-2017
  Probado: ARDUINO UNO r3 - IDE 1.8.2 (Windows7)
  Web: www.playbyte.es/electronica/
  Licencia: Creative Commons Share-Alike 3.0
  http://creativecommons.org/licenses/by-sa/3.0/deed.es_ES
  ##############################################################
*/

// Las conexiones son simples pero hay que tener en cuenta un pequeño detalle, la dirección del bus cambia según el estado lógico del pin SDO, y si se deja desconectado la dirección queda indeterminada, por lo que puede parecer que no funciona correctamente.

// SDO=GND -> I2C Address (0x76)
// SDO=3.3V -> I2C Address (0x77)

// En la librería desarrollada por Adafruit, el bus I2C utiliza por defecto la dirección (0x77), para modificarlo hay que editar el fichero “Adafruit_BMP280.h”
//y en la linea 37  #define BMP280_ADDRESS  (0x77) cambiamos la dirección (0x
// BME280 SPI for Arduino Mega 256
#define PIN_BME_SCK  3
#define PIN_BME_MISO 2
#define PIN_BME_MOSI 1
#define PIN_BME_CS1  0 // sensor de presion 1

//Connect the SCK pin to Digital #13 but any pin can be used later
//Connect the SDO MISO pin to Digital #12 but any pin can be used later
//Connect the SDI (o SDA) pin to Digital #11 but any pin can be used later
//Connect the CS pin Digital #10 but any pin can be used later
// https://learn.adafruit.com/adafruit-bmp280-barometric-pressure-plus-temperature-sensor-breakout/arduino-test

//#define PIN_BME_SCK  13  //ES IGUAL A SCL
//#define PIN_BME_MISO 12  //SDO pin to Digital #12
//#define PIN_BME_MOSI 11   //SDA
//#define PIN_BME_CS1  10 // sensor de presion 1

//D13 (SCLK). . . . . .SCL. . . . . . . . . . .SCL. . . . . . . . . .SCL
//D12 (MISO). . . . . .SDO. . . . . . . . . .SDO. . . . . . . . . .SDO
//D11 (MOSI). . . . . .SDA. . . . . . . . . . .SDA. . . . . . . . . .SDA
//D10 (ss1). . . . . . . .CSB
#define PIN_LCD_RS  A8
#define PIN_LCD_EN  A9
#define PIN_LCD_D4  A10
#define PIN_LCD_D5  A11
#define PIN_LCD_D6  A12
#define PIN_LCD_D7  A13

#ifdef MPX5050dp

#endif

#include "src/Adafruit_BMP280/Adafruit_BMP280.h"
//#include "LiquidCrystal_I2C.h"
#include <LiquidCrystal.h>
//LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

//Adafruit_BMP280 bmp; // I2C
     Adafruit_BMP280 bmp(
    PIN_BME_CS1,
    PIN_BME_MOSI,
    PIN_BME_MISO,
    PIN_BME_SCK
    );
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
//  lcd.begin(20, 4);
//  //  //lcd.backlight();
//  lcd.clear();


//  if (!bmp.begin(0x76))//I2C
    if (!bmp.begin()) //SP
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
  
  
  
//  lcd.setCursor(0, 0);
//  lcd.print("dp [cmH20]");
  Serial.print("Analogico");Serial.println(analogRead(A1));
//  lcd.setCursor(0, 1);
//  lcd.print("BMP280 : "); lcd.print(presion);
//  lcd.setCursor(0, 2);
//  lcd.print("MPX5050: "); lcd.print(presion_mpx);

  delay(50);
}
