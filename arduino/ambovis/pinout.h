#ifndef PINOUT_H
#define PINOUT_H

// Stepper driver (FlexyStepper)
#define PIN_STEPPER_STEP 6
#define PIN_STEPPER_DIRECTION 7
#define PIN_EN 99
// Stepper homing
#define PIN_ENDSTOP 5

// Display
#define PIN_LCD_RS 8
#define PIN_LCD_EN 9
#define PIN_LCD_D4 10
#define PIN_LCD_D5 11
#define PIN_LCD_D6 12
#define PIN_LCD_D7 13

// #define PIN_BME_CS2  4  // sensor de presion 2
#define PIN_ENC_CL  18
#define PIN_ENC_DIR 19
#define PIN_ENC_SW  4

//#define TFT_CLK 	13 FIXED (HW)
//#define TFT_MISO 	12 FIXED (HW)
//#define TFT_MOSI 	11
#define TFT_CS 		53
#define TFT_DC 		49
#define TFT_RST 	48
#define TFT_SLEEP 40
#define LCD_SLEEP 38

#define PIN_POWEROFF    43
#define BCK_LED         44
#define GREEN_LED       45
#define YELLOW_LED      46
#define RED_LED         47

#define PIN_BUZZER      3
#define PIN_MUTE        2

#define PIN_MENU_UP     30
#define PIN_MENU_DN     34
#define PIN_MENU_EN     28
#define PIN_MENU_BCK    32

#define PIN_STEPPER     36

#define PIN_BAT_LEV     A4
#define PIN_MPX_LEV     A2
#define PIN_PRESSURE    A3

#define PIN_TEMP        37

#endif // ENCODER_H
