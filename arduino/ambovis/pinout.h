#ifndef PINOUT_H
#define PINOUT_H

// Stepper driver (FlexyStepper)
#define PIN_STEPPER_STEP 6
#define PIN_STEPPER_DIRECTION 7
#define PIN_EN 99
// Buzzer
#define PIN_BUZZ 100

// Stepper homing
#define PIN_ENDSTOP 5

// Solenoid pine
#define PIN_SOLENOID 39

// Display

#ifdef LCD_I2C
#define I2C_DIR 0x3F
#else
#define PIN_LCD_RS 8
#define PIN_LCD_EN 9
#define PIN_LCD_D4 10
#define PIN_LCD_D5 11
#define PIN_LCD_D6 12
#define PIN_LCD_D7 13
#endif

// BME280 SPI for Arduino Nano or Mega 128
#define PIN_BME_SCK  99 //SCL
#define PIN_BME_MISO 99 //SDO
#define PIN_BME_MOSI 99 //SDA-SDI
#define PIN_BME_CS1  99 // sensor de presion 1
#define PIN_BME_CS2  99 // sensor de presion 1


// #define PIN_BME_CS2  4  // sensor de presion 2
#define PIN_ENC_SW  4
#define PIN_ENC_CL  2
#define PIN_ENC_DIR 3

// IF ARDUINO UNO
//#define PIN_ENC_CL  2
//#define PIN_ENC_DIR 3



#endif // ENCODER_H
