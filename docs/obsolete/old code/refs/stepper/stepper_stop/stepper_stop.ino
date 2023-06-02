//FROM:https://groups.google.com/forum/#!topic/accelstepper/V3cGYPpYujc
// Hi,

// On Friday, April 22, 2016 at 104533 PM UTC+2, Joe Varghese John wrote
// 2) what is the best way to stop at the maximum deceleration speed but i dont want to come back.
// this is exactly what the stop() command does - it calculates a new target position for the stepper taking into account the acceleration.
// You need to call stop() once and then use run() or runToPosition().

// the example below shows how to use stop() correctly.

#include AccelStepper.h

 Define some steppers and the pins the will use
AccelStepper stepper1;  Defaults to AccelStepperFULL4WIRE (4 pins) on 2, 3, 4, 5

#define STOP_BTN A0    input pin for stop button

bool curr_state, last_state;

void setup()
{  
    stepper1.setMaxSpeed(200.0);
    stepper1.setAcceleration(100.0);
    stepper1.moveTo(2400000);

    pinMode(STOP_BTN, INPUT_PULLUP);     connect a normally open switch between A0 and Ground
    curr_state = digitalRead(A0);
    last_state = curr_state;
}

void loop()
{
  last_state = curr_state;
 
  curr_state = digitalRead(STOP_BTN);
  if ((curr_state != last_state) && (curr_state == LOW))    call stop() once when the button was pressed
    stepper1.stop();
     
  stepper1.run();
}