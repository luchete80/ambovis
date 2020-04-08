//https://diyi0t.com/lcd-display-tutorial-for-arduino-and-esp8266/

// include the library code:
#include <LiquidCrystal.h>
#include "Arduino.h"

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
//const int RS = D2, EN = D3, d4 = D5, d5 = D6, d6 = D7, d7 = D8;
//#define PIN_LCD_RS A0
//#define PIN_LCD_RW A1
//#define PIN_LCD_EN  A2
//#define PIN_LCD_D4 A3
//#define PIN_LCD_D5 A4
//#define PIN_LCD_D6 A5
//#define PIN_LCD_D7 A6

#define PIN_LCD_RS  2
#define PIN_LCD_EN  3
#define PIN_LCD_D4  4
#define PIN_LCD_D5  5
#define PIN_LCD_D6  6
#define PIN_LCD_D7  7

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
 
void setup() {
  Serial.begin(9600);
  // set up the LCD's number of columns and rows:
  lcd.begin(20, 4);
//  lcd.backlight();
  // Print a message to the LCD.
  lcd.print("hello, world!");
}
 
void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  Serial.println("Aver");
  // print the number of seconds since reset:
  lcd.print(millis() / 1000);
}
