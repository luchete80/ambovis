// BMP280_Uno_I2C_LCD_nn
// BMP 280 temperature - barometric pressure sensor on I2C bus
// RTC-SD shield on Uno 
// LCD display on I2C bus (this LCD has address 35F)
// Notice BMP280 is a 3.3 V device - pin layout:
//    VCC --> 3.3 V
//    GND --> GND
//    SCL --> A5
//    SDA --> A4
// 
// LCD I2C piggyback has 4 pins to connect: 
//    VCC --> 5V
//    GND --> GND
//    SCL --> A5
//    SDA --> A4
//
// public domain
// Floris Wouterlood - June 10, 2017
//
// important parameter: SYNC_INTERVAL = millis between SD write action 
//
//
// ============================== meaning of the leds =================================================================
// yellow led on pin 4 - synching to card abd error-no card
// green  led on pin 3 - write to card

  #include <SPI.h>
  #include <SD.h>
  #include <Wire.h>
  #include "RTClib.h"

// =============================== BMP280 =============================================================================
  #include "BMP280.h"  
  #define P0 1025.45 // calibratie Thorbeckestraat 33, Leiden, zolder
  BMP280 bmp;

// =============================== set the lcd I2C address ============================================================  
  #include <LiquidCrystal_I2C.h>  
  LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  

// =============================== define the RTC =====================================================================
  RTC_DS1307 RTC;                               

// =============================== SD logging shield ==================================================================
  long sync_interval = 120000;                  // milliseconds between calls to flush() - to write data to the card = 2 minutes
  long lastMillis=0;
  uint32_t syncTime = 0;                        // time of last sync()
  #define yellowLEDpin 4                        // control LED pin
  #define greenLEDpin 3                         // control LED pin
  const int chipSelect = 10;                    // for the data logging shield, we use digital pin 10 for the SD cs line
  File logfile;                                 // filename of logging file

// =============================== initialize the BMP280 ==============================================================  

void setup(void)
{
  lcd.begin(20,4); 
  lcd.setCursor(0,0); 
  lcd.print ("barometer &");
  lcd.setCursor(0,1); 
  lcd.print ("temperature");  
  lcd.setCursor(0,2); 
  lcd.print ("baro = ");
  lcd.setCursor(18,2); 
  lcd.print ("mb");  
  lcd.setCursor(0,3); 
  lcd.print ("temp = ");
  lcd.setCursor(18,3); 
  lcd.print ("*C");
 
  pinMode(yellowLEDpin, OUTPUT);                  
  pinMode(greenLEDpin, OUTPUT);
    
  Serial.begin(9600);
  Serial.println("");
  Serial.print("Barometric pressure & temperature logger ");
  Serial.println("");  
  
// =============================== logging shield section =============================================================
  Serial.print("Initializing SD card ... ");    // initialize the SD card  
  pinMode(10, OUTPUT); // default chip select pin
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);
  Serial.println("logging temperature and millibars");

// =============================== RTC section ========================================================================

// =============================== connect to RTC =====================================================================
  Wire.begin();  
  if (!RTC.begin()) {
    logfile.println("RTC failed");
  #if ECHO_TO_SERIAL
    Serial.println("RTC failed");
  #endif  //ECHO_TO_SERIAL
  }

  logfile.println("millis,stamp,date/time,,barometer,,temp");    

  if(!bmp.begin()){
    Serial.println("BMP280 init failed!");
    while(1);
  }
  else Serial.println("BMP280 init .......... sensor initialized."); 
       Serial.println ("");  
       bmp.setOversampling(4); 
}

void loop(void)
{

  double T,P;
  char result = bmp.startMeasurment();
 
  if(result!=0){
    delay(result);
    result = bmp.getTemperatureAndPressure(T,P);
    
      if(result!=0)
      {
        Serial.print("T = \t");Serial.print(T,2); Serial.print(" *C\t");
        Serial.print("baro = \t");Serial.print(P,2); Serial.println(" mBar\t");
        lcd.setCursor(10,2);                    // barometer to lcd display
        lcd.print (P,2);
        lcd.setCursor(12,3);                    // temp to lcd display
        lcd.print (T,2); 
        delay(5000);                            // one minute between samples
      }
      else {
        Serial.println("Error");
      }
  }
  else {
    Serial.println("Error");
  }
  
  DateTime now;


  digitalWrite(greenLEDpin, HIGH);

  uint32_t m = millis();                        // log milliseconds since starting
  logfile.print(m);                             // milliseconds since start
  logfile.print(", ");    

  now = RTC.now();                              // fetch the time
  // log time
  logfile.print(now.unixtime());                // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
  logfile.print(", ");    
  logfile.print(", ");    
  logfile.print(P,2);
  logfile.print(", ");
  logfile.print(", ");      
  logfile.print(T,2);
  logfile.println();

  digitalWrite(greenLEDpin, LOW);
  delay(60000);
// =============================== refresh logfile ===================================================================

    if (millis() - lastMillis > sync_interval) 
               {            

               // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
               // which uses a bunch of power and takes time
               // blink yellow LED to show we are syncing data to the card & updating FAT!
               
               digitalWrite(yellowLEDpin, HIGH);
               logfile.flush();
               digitalWrite(yellowLEDpin, LOW);
               lastMillis = millis();
    }
}

// =============================== subroutines ========================================================================

// =============================== error routine ======================================================================

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // yellow LED indicates error
  digitalWrite(yellowLEDpin, HIGH);

  while(1);
}



