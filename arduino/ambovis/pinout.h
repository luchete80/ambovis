#ifndef PINOUT_H
#define PINOUT_H

// Stepper driver (FlexyStepper)
#define PIN_STEPPER_STEP 6
#define PIN_STEPPER_DIRECTION 7
#define PIN_EN 99
// Buzzer
#define PIN_BUZZ 1

// Stepper homing
#define PIN_ENDSTOP 5

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

// #define PIN_BME_CS2  4  // sensor de presion 2
#define PIN_ENC_CL  19
#define PIN_ENC_DIR 18
#define PIN_ENC_SW  4

//#define TFT_CLK 	13 FIXED (HW)
//#define TFT_MISO 	12 FIXED (HW)
//#define TFT_MOSI 	11
#define TFT_CS 		53
#define TFT_DC 		49
#define TFT_RST 	48


#define GREEN_LED       22
#define YELLOW_LED      23
#define RED_LED         24

#define PIN_BUZZER      3
#define PIN_MUTE        2

//4 leds de alarmas en los pines 44, 45, 46, y 47 
//pin 43 recibiría una señal de cuando se corte la luz
#define PIN_POWEROFF    43
#define PIN_NO_ALARM    44
#define PIN_PIP_ALARM   45
#define PIN_PEEP_ALARM  46
#define PIN_PIP_ALARM   47



// IF ARDUINO UNO
//#define PIN_ENC_CL  2
//#define PIN_ENC_DIR 3



#endif // ENCODER_H
