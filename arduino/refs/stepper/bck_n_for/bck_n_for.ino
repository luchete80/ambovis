#include "src/TimerOne/TimerOne.h"

#define STEPPER_MICROSTEPS 8
#define STEPPER_STEPS_PER_REVOLUTION 200

#define STEPPER_MICROSTEPS_PER_REVOLUTION (STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS)
#define STEPPER_DIR 1
#define STEPPER_HOMING_DIRECTION    (-1)
#define STEPPER_HOMING_SPEED        (STEPPER_MICROSTEPS * 1000)   // Steps/s
// #define STEPPER_LOWEST_POSITION     (STEPPER_MICROSTEPS * -100)   // Steps
// #define STEPPER_HIGHEST_POSITION    (STEPPER_MICROSTEPS *   50)   // Steps
#define STEPPER_LOWEST_POSITION     (-10)   // Steps
#define STEPPER_HIGHEST_POSITION    (STEPPER_MICROSTEPS * 50)   // Steps
#define STEPPER_SPEED_DEFAULT       (STEPPER_MICROSTEPS *  200)   // Steps/s
#define STEPPER_ACC_EXSUFFLATION    (STEPPER_MICROSTEPS *  200)   // Steps/s2
#define STEPPER_ACC_INSUFFLATION    (STEPPER_MICROSTEPS *  200)   // Steps/s2


#define PIN_STEPPER_STEP 6
#define PIN_STEPPER_DIRECTION 7
#define PIN_EN 8

#define PIN_ENDSTOP 5
//
//#define ACCEL 1

int pos = 1;

//Accel
//From https://www.schmalzhaus.com/EasyDriver/Examples/EasyDriverExamples.html
//FlexyStepper
//https://github.com/Stan-Reifel/FlexyStepper/blob/master/Documentation.md
  
#ifdef ACCEL
 #include <AccelStepper.h>
 AccelStepper stepper(
  AccelStepper::DRIVER,
  PIN_STEPPER_DIRECTION,
  PIN_STEPPER_STEP);
#else
 #include <FlexyStepper.h>
 FlexyStepper stepper;
#endif


unsigned long cycle_time;
unsigned long change_time;

float _stepperSpeed;


#define PIN_BUZZER      3

void goHome(){

  #ifdef ACCEL_STEPPER
    Serial.print("Stepper is Homing . . . . . . . . . . . ");

  while (digitalRead(PIN_ENDSTOP)) {  // Make the Stepper move CCW until the switch is activated
    stepper->moveTo(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    stepper->run();  // Start moving the stepper
    delay(5);
  }

  stepper->setCurrentPosition(0);  // Set the current position as zero for now
  stepper->setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper->setAcceleration(100.0);  // Set Acceleration of Stepper
  initial_homing = 1;

  while (!digitalRead(PIN_ENDSTOP)) { // Make the Stepper move CW until the switch is deactivated
    stepper->moveTo(initial_homing);
    stepper->run();
    initial_homing++;
    delay(5);
  }

  stepper->setCurrentPosition(0);
  Serial.println("Homing Completed");
  #endif
}

bool reach_home=false;

void setup()
{
      Timer1.initialize(50);
      Timer1.attachInterrupt(timer1Isr);
      
        pinMode(PIN_BUZZER,OUTPUT);
        pinMode(PIN_ENDSTOP,INPUT_PULLUP);
      digitalWrite(PIN_BUZZER,HIGH);

  Serial.begin(250000);
  cycle_time=1000;
  
  _stepperSpeed = STEPPER_SPEED_DEFAULT;


	#ifdef ACCEL
  goHome();
  

    stepper.setMaxSpeed(STEPPER_SPEED_DEFAULT);
 stepper.setSpeed(STEPPER_SPEED_DEFAULT);
 stepper.moveTo(pos);
  stepper.setAcceleration(1000);
#else

     
	stepper.connectToPins(PIN_STEPPER_STEP, PIN_STEPPER_DIRECTION);
  stepper.setStepsPerRevolution(STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS);
  stepper.setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_EXSUFFLATION);

    while(!reach_home){
    if (digitalRead(PIN_ENDSTOP)){
          stepper.moveToHomeInSteps(
              STEPPER_HOMING_DIRECTION,
              STEPPER_HOMING_SPEED,
              STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS, //ATTENTION
              PIN_ENDSTOP);
              Serial.println("Homing alto");
              stepper.processMovement();   
        } else {
        reach_home=true;}
    }
        
      
#endif
change_time=millis();
}

  // Note 2: It is also assumed that your stepper driver board is  
  // configured for 1x microstepping.
  
void loop()
{


//  if ( (millis ()-change_time) > cycle_time)
//    pos=-pos;
#ifdef ACCEL
//stepper1.distanceToGo() == 0){	
if ( (millis ()-change_time) > cycle_time){
  change_time=millis();
	stepper.setMaxSpeed(STEPPER_SPEED_DEFAULT);
	pos=-pos;
  stepper.setSpeed(STEPPER_SPEED_DEFAULT);
	stepper.moveTo(pos);		
}
	//stepper.move(200);
	stepper.run();   
 //stepper1.currentPosition
 
#else
        
	/* Stepper control: set acceleration and end-position */
	//stepper.setSpeedInStepsPerSecond(_stepperSpeed);
	//stepper.setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_INSUFFLATION);
 
  if ( (millis ()-change_time) > cycle_time){
    pos=-pos;
      if (pos>0)
      stepper.setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
      else
      stepper.setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
      
      
      stepper.setSpeedInStepsPerSecond(STEPPER_SPEED_DEFAULT);
      Serial.println("Insuflando");
      change_time=millis();

      //stepper.moveRelativeInSteps(pos*STEPPER_MICROSTEPS_PER_REVOLUTION/2);
      Serial.println("Cambio..");
      }


	
	    //stepper.processMovement();
#endif 
	
}


//#include <AccelStepper.h>
//
//// Define a stepper and the pins it will use
//AccelStepper stepper(AccelStepper::DRIVER, 7, 6);
//
//int pos = 500;
//
//void setup()
//{ 
//  stepper.setMaxSpeed(3000);
//  stepper.setAcceleration(1000);
//}
//
//void loop()
//{
//  if (stepper.distanceToGo() == 0)
//  {
//    delay(500);
//    pos = -pos;
//    stepper.moveTo(pos);\
//  }
//  stepper.run();
//}

void timer1Isr(void)
{
  #ifdef ACCEL
    stepper.run();
  #else
    stepper.processMovement(); //LUCIANO
    //Serial.print("Speed");Serial.println(_stepperSpeed);
  #endif
}
