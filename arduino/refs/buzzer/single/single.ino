int buzzer = 3; //Define buzzerPin

void setup() {
  pinMode(buzzer, OUTPUT); //Set buzzerPin as output
  beep(50); //Beep
  beep(50); //Beep
  delay(1000); //Add a little delay

}

void loop() {
  beep(500);
  //tone 
}

void beep(unsigned char delayms) { //creating function
  digitalWrite(buzzer, 0); //Setting pin to high
  delay(delayms); //Delaying
  digitalWrite(buzzer,1); //Setting pin to LOW
  delay(delayms); //Delaying
  
}
