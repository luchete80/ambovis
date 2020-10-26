

#define PIN_ONOFF 5

//#include <LiquidCrystal.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

//LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
#define TFT_CS     53
#define TFT_DC    49
#define TFT_RST   48

//#define TFT_CLK   13 FIXED (HW)
//#define TFT_MISO  12 FIXED (HW)
//#define TFT_MOSI  11
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);
//Adafruit_ILI9341 tft(TFT_DC, TFT_RST);
unsigned long onoff_time;
bool on_off;

void setup() {
  Serial.begin(9600);
    //lcd.begin(20, 4);  //I2C
    onoff_time=millis();
    pinMode(PIN_ONOFF, OUTPUT);
    //lcd.clear();
    //lcd.setCursor(0, 0);
    //lcd.print("Test 2N2222!!");
    on_off=true;
    tft.begin();
    tft.fillScreen(ILI9341_RED);
            tft.setTextColor(ILI9341_BLACK); tft.setTextSize(2); 
        tft.setCursor(0, 40);   
        tft.println("TEST SLEEP");
    //tft.fillRect(0, 240 , 320, 10, ILI9341_RED);//CLEAN PREVIOUS CURVE x,y,lengthx,lentgthy
}

void loop()
{
  if (millis() > onoff_time + 1000){
    Serial.print(on_off);
    digitalWrite(PIN_ONOFF,on_off);
    onoff_time=millis();
    on_off=!on_off;
    tft.begin();
    tft.fillScreen(ILI9341_RED);
                tft.setTextColor(ILI9341_BLACK); tft.setTextSize(2); 
        tft.setCursor(0, 40);   
        tft.println("TEST SLEEP");
        }
  
}
