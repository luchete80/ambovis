//VERY GOOD REF HERE: http://arduino-er.blogspot.com/2015/04/display-waveform-on-mini-oled-with.html

#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>

#define DT 50 // Loop sleep time (ms)
#define CYCLE 127

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

int count=0;
byte x;
byte escala=32;

byte y[128];

void setup(void) {
  Serial.begin(115200);
  u8g2.begin();
}

// void loop(void) {
  // u8g2.firstPage();
  // do {
    // //u8g2.setFont(u8g2_font_ncenB14_tr);
    // u8g2.setFont(u8g2_font_7x14B_tf);
    // u8g2.drawStr(0,10,"Hello!");
  // } while ( u8g2.nextPage() );
  // delay(1000);
// }

void drawY(){
  u8g2.drawPixel(0, y[0]);
  for(int i=1; i<CYCLE+1; i++){
      u8g2.drawPixel(i, y[i]);
      //u8g.drawLine(i-1, y[i-1], i, y[i]);
  }  
}
unsigned long time;
void loop(void) {
    float flux = fabs(cos(2.0 * 3.1416 * ((float)(count) / ((float) CYCLE) ))) * 2* escala;
	count +=2;
	
  if (count >127){
    count=0;
   escala-=4;
  }

  y[count  ]=byte(flux);
  y[count+1]=byte(flux);
  
  time=millis();
  u8g2.firstPage();
  do {
    //u8g2.drawPixel(126,20);
    u8g2.drawPixel(byte (count), byte(flux) );
    drawY();
  } while ( u8g2.nextPage() );
  Serial.println("elaspsed time: ");Serial.println(millis()-time);
//  /delay(30);
  
  
}
