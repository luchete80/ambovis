#define PIN_LCD_RS 8
#define PIN_LCD_EN 9
#define PIN_LCD_D4 10
#define PIN_LCD_D5 11
#define PIN_LCD_D6 12
#define PIN_LCD_D7 13

#define PIN_ONOFF 5

#include <LiquidCrystal.h>

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
unsigned long onoff_time;
bool on_off;

void setup() {
  Serial.begin(9600);
    lcd.begin(20, 4);  //I2C
    onoff_time=millis();
    pinMode(PIN_ONOFF, OUTPUT);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Test 2N2222!!");
    on_off=true;
}

void loop()
{
  if (millis() > onoff_time + 1000){
    Serial.print(on_off);
    digitalWrite(PIN_ONOFF,on_off);
    onoff_time=millis();
    on_off=!on_off;}
  
}
