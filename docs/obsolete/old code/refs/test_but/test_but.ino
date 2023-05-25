#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define ILI9341_LIGHTGREY 0xC618 /* 192, 192, 192 */
#define ILI9341_DARKGREY 0x7BEF /* 128, 128, 128 */

#define TIME_MUTE 5
#define DEBUG 1
//enum serialpos={ALARM_=0,TIME_,PRESSURE_,FLUX_,VT_};
//enum serialpos {TIME_=0,PRESSURE_,FLUX_,VT_,ALARM_};//ORIGINAL
#define TIME_     0
#define P_        1
#define FLUX_     2
#define ALARM_    3
#define VT_       4
#define VI_       5
#define VE_       6

bool ve_readed,vi_readed;

float diff_var[]={10., 10., 800.,0.,200.}; 
int vi,ve,vt;
unsigned long cyclenum;
bool vols_drawn;

#define TFT_CLK 13
#define TFT_MISO 12
#define TFT_MOSI 11
#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8

//SPI HARDWARE SHOULD BE PINS CLK #13 AND MISO #11

//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO); //THIS SHOULD NEVER BE USED!
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

bool lcd_cleaned=false;

unsigned long time_last_show=0;

char a[10],b[10];

const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
int last_t;
int integerFromPC [7];
float floatFromPC = 0.0;
int last_vals[7][2];

int axispos[3]; //from each graph

int buzzer=3; //pin
unsigned long timebuzz=0;
bool isbuzzeron=false;
#define TIME_BUZZER 500

#define GREEN_LED   4
#define YELLOW_LED  5
#define RED_LED     6

#define PIN_MUTE    2

enum _state {NO_ALARM=0,PEEP_ALARM=1,PIP_ALARM=2,PEEP_PIP_ALARM=3};
bool wait4statechg=false;

_state state;

char recvChar;
char endMarker = '>';
boolean newData = false;
byte valsreaded=0;
int valsreaded_[3];
byte last_x=0;


int count=0;
byte escala=32;
byte x[128],y[64];

byte rx[128],ry[128];
int  ry2[128];
int yflux[2];
int yvt[2];
int xgra[3][2];
char buffer[10];

//MUTE
boolean last_mute,curr_mute;
unsigned long time_mute;
boolean buzzmuted;

boolean debounce(boolean last, int pin)
{
    boolean current = digitalRead(pin);
    if (last != current)
    {
        delay(10);
        current = digitalRead(pin);
    }
    return current;
}

void setup() {
    Serial.begin(250000);
    Serial.println("ILI9341 Test!"); 
  
    pinMode(PIN_MUTE, INPUT);
    axispos[0]=63;  axispos[1]=160;  axispos[2]=260;
    cyclenum=0;
    buzzmuted=false;
    last_mute=LOW;
}


void loop(void) {

    curr_mute = debounce ( last_mute, PIN_MUTE );         //Debounce for Up button
    if (last_mute== LOW && curr_mute == HIGH && !buzzmuted){
        time_mute=millis();
        buzzmuted=true;
    }
    last_mute = curr_mute;

//    if(buzzmuted) {
//        if (millis() > time_mute + TIME_MUTE )  
//        buzzmuted=false;
//    }

    if (buzzmuted) {
      Serial.println("MUTED");
    } else {
      Serial.println("NO MUTED");
    }
    
    
    
    
}
