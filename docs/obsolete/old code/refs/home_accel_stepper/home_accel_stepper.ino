//https://forum.arduino.cc/index.php?topic=499412.0

/*  Motor Homing code using AccelStepper and the Serial Monitor

  Created by Yvan / https://Brainy-Bits.com
  This code is in the public domain...
*/

#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMSmid(0x61); // I2C address of MEGA shield.
Adafruit_StepperMotor *mystepper1 = AFMSmid.getStepper(200, 1);  // 200step 1.8degree Motor on port #1 (M1 and M2).

// Define the Pins used
#define home_switch 18 // Pin 18 connected to Home Switch (MicroSwitch)

// Stepper Travel Variables
long TravelX;  // Used to store the X value entered in the Serial Monitor
int move_finished = 1; // Used to check if move is completed
long initial_homing = -1; // Used to Home Stepper at startup

void forwardstep1() {
  mystepper1->onestep(FORWARD, SINGLE);
}
void backwardstep1() {
  mystepper1->onestep(BACKWARD, SINGLE);
}

AccelStepper Astepper1(forwardstep1, backwardstep1); // use functions to step

void setup() {
  Serial.begin(9600);  // Start the Serial monitor with speed of 9600 Bauds
  AFMSmid.begin();  // create with the default frequency 1.6KHz
  pinMode(home_switch, INPUT_PULLUP);

  delay(5);  // Wait for EasyDriver wake up

  //  Set Max Speed and Acceleration of each Steppers at startup for homing
  Astepper1.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  Astepper1.setAcceleration(100.0);  // Set Acceleration of Stepper


  // Start Homing procedure of Stepper Motor at startup

  Serial.print("Stepper is Homing . . . . . . . . . . . ");

  while (digitalRead(home_switch)) {  // Make the Stepper move CCW until the switch is activated
    Astepper1.moveTo(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    Astepper1.run();  // Start moving the stepper
    delay(5);
  }

  Astepper1.setCurrentPosition(0);  // Set the current position as zero for now
  Astepper1.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  Astepper1.setAcceleration(100.0);  // Set Acceleration of Stepper
  initial_homing = 1;

  while (!digitalRead(home_switch)) { // Make the Stepper move CW until the switch is deactivated
    Astepper1.moveTo(initial_homing);
    Astepper1.run();
    initial_homing++;
    delay(5);
  }

  Astepper1.setCurrentPosition(0);
  Serial.println("Homing Completed");
  Serial.println("");
  Astepper1.setMaxSpeed(1000.0);      // Set Max Speed of Stepper (Faster for regular movements)
  Astepper1.setAcceleration(1000.0);  // Set Acceleration of Stepper

  // Print out Instructions on the Serial Monitor at Start
  Serial.println("Enter Travel distance (Positive for CW / Negative for CCW and Zero for back to Home): ");
}
/* 
void loop() {

  while (Serial.available() > 0)  { // Check if values are available in the Serial Buffer

    move_finished = 0; // Set variable for checking move of the Stepper

    TravelX = Serial.parseInt(); // Put numeric value from buffer in TravelX variable
    if (TravelX < 0 || TravelX > 1350) {  // Make sure the position entered is not beyond the HOME or MAX position
      Serial.println("");
      Serial.println("Please enter a value greater than zero and smaller or equal to 1350.....");
      Serial.println("");
    } else {
      Serial.print("Moving stepper into position: ");
      Serial.println(TravelX);

      Astepper1.moveTo(TravelX);  // Set new moveto position of Stepper

      delay(1000);  // Wait 1 seconds before moving the Stepper
    }
  }

  if (TravelX >= 0 && TravelX <= 1350) {

    // Check if the Stepper has reached desired position
    if ((Astepper1.distanceToGo() != 0)) {

      Astepper1.run();  // Move Stepper into position

    }

    // If move is completed display message on Serial Monitor
    if ((move_finished == 0) && (Astepper1.distanceToGo() == 0)) {
      Serial.println("COMPLETED!");
      Serial.println("");
      Serial.println("Enter Travel distance (Positive for CW / Negative for CCW and Zero for back to Home): ");
      move_finished = 1; // Reset move variable
    }
  }
} */

void loop() {
///////////////////////////////////////////////////////////////////////////
  recvWithEndMarker();
  parseData();
  showNewData();
  Astepper1.moveTo(TravelX);  // Set new moveto position of Stepper
  Astepper1.run();
///////////////////////////////////////////////////////////////////////////
}