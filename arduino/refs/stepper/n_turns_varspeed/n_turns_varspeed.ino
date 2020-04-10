#define STEPPER_MICROSTEPS 4
#define STEPPER_STEPS_PER_REVOLUTION 200

#define STEPPER_MICROSTEPS_PER_REVOLUTION (STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS)
#define STEPPER_DIR 1
#define STEPPER_HOMING_DIRECTION    (-1)
#define STEPPER_HOMING_SPEED        (STEPPER_MICROSTEPS * 1000)   // Steps/s
// #define STEPPER_LOWEST_POSITION     (STEPPER_MICROSTEPS * -100)   // Steps
// #define STEPPER_HIGHEST_POSITION    (STEPPER_MICROSTEPS *   50)   // Steps
#define STEPPER_LOWEST_POSITION     (STEPPER_MICROSTEPS *  85)   // Steps
#define STEPPER_HIGHEST_POSITION    (STEPPER_MICROSTEPS * -100)   // Steps
#define STEPPER_SPEED_DEFAULT       (STEPPER_MICROSTEPS *  1600)   // Steps/s
#define STEPPER_ACC_EXSUFFLATION    (STEPPER_MICROSTEPS *  600)   // Steps/s2
#define STEPPER_ACC_INSUFFLATION    (STEPPER_MICROSTEPS *  450)   // Steps/s2


#define PIN_STEPPER_STEP 6
#define PIN_STEPPER_DIRECTION 7
#define PIN_EN 8
//
//#define ACCEL 0

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
int n=1;

void setup()
{
  Serial.begin(9600);
  cycle_time=1000;
  
  _stepperSpeed = STEPPER_SPEED_DEFAULT;
 
 
	stepper.connectToPins(PIN_STEPPER_STEP, PIN_STEPPER_DIRECTION);
  stepper.setStepsPerRevolution(1600);
  stepper.setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_EXSUFFLATION);
  stepper.setTargetPositionInSteps(4800);
	stepper.setSpeedInStepsPerSecond(50);
	change_time=millis();
	
}

  // Note 2: It is also assumed that your stepper driver board is  
  // configured for 1x microstepping.
  
void loop()
{
	if ( (millis ()-change_time) > cycle_time){
			
  //stepper.moveRelativeInSteps(10*1600);
  //stepper.setTargetPositionInSteps(4800);
	stepper.setSpeedInStepsPerSecond(n*200);
	change_time=millis();
	n++;
}

	stepper.processMovement();

}
