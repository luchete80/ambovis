#define STEPPER_SPEED_DEFAULT 200

 #include <AccelStepper.h>
 #include <LiquidCrystal_I2C.h>
 AccelStepper stepper(
  1,
  6,
  7);


unsigned long cycle_time;
unsigned long change_time;
float _stepperSpeed;

 int pos=1000;

 LiquidCrystal_I2C lcd(0x3F, 20, 4);
 
void setup()
{
  
  //LUCIANO-------------
  lcd.begin();
  //lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  
  Serial.begin(9600);
  cycle_time=1000;
  
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
    lcd.setCursor(0,0);
    lcd.print("Hola");

//stepper1.distanceToGo() == 0){	
if ( (millis ()-change_time) > cycle_time){
  change_time=millis();
	stepper.setMaxSpeed(1000);
	pos=-pos;
  stepper.setSpeed(pos*1);
	stepper.moveTo(pos);		
}
	//stepper.move(200);
	stepper.run();   
 //stepper1.currentPosition


	
}
