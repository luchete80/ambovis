//https://sites.google.com/site/angmuz/home/proyecto-10-arduino-basics-steppers-motors
#include <AccelStepper.h>
#include <AFMotor.h>
// two stepper motors one on each port
AF_Stepper motor1(64, 1);
AF_Stepper motor2(64, 2);
// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {  
  motor1.onestep(FORWARD, SINGLE);
}
void backwardstep1() {  
  motor1.onestep(BACKWARD, SINGLE);
}
// wrappers for the second motor!
void forwardstep2() {  
  motor2.onestep(FORWARD, SINGLE);
}
void backwardstep2() {  
  motor2.onestep(BACKWARD, SINGLE);
}
// Motor shield has two motor ports, now we'll wrap them in an AccelStepper object
AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);
int pos1 = -4096;
int pos2 = 4096;
void setup()
{  
  stepper1.setMaxSpeed(400);
  stepper1.setAcceleration(200);
  stepper2.setMaxSpeed(400);
  stepper2.setAcceleration(200);
}
void loop()
{
  if (stepper1.distanceToGo() == 0)
  {
    pos1 = -pos1;
    stepper1.moveTo(pos1);
  }
  if (stepper2.distanceToGo() == 0)
  {
    pos2 = -pos2;
    stepper2.moveTo(pos2);
  }
  stepper1.run();
  stepper2.run();
}