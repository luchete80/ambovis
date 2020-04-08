#define STEPPER_SPEED_DEFAULT 200

 #include <AccelStepper.h>
 AccelStepper stepper(
  1,
  6,
  7);


unsigned long cycle_time;
unsigned long change_time;
float _stepperSpeed;

 int pos=400;
 
void setup()
{
  Serial.begin(9600);
  cycle_time=2500;
  
    stepper.setMaxSpeed(STEPPER_SPEED_DEFAULT);
 stepper.setSpeed(STEPPER_SPEED_DEFAULT);
 stepper.moveTo(pos);
  stepper.setAcceleration(1000);

change_time=millis();
}

  // Note 2: It is also assumed that your stepper driver board is  
  // configured for 1x microstepping.
 
void loop()
{

//stepper1.distanceToGo() == 0){	
if ( (millis ()-change_time) > cycle_time){
  Serial.println("yendo...");
  change_time=millis();
	stepper.setMaxSpeed(200);
	pos=-pos;
  stepper.setSpeed(200);
	stepper.moveTo(pos);		
}
	//stepper.move(200);
	stepper.run();   
 //stepper1.currentPosition


	
}
