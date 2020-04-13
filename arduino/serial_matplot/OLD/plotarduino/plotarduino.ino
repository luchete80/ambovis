const int analogIn = A0;
int analogVal = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {

  //analogVal = analogRead(analogIn);
  analogVal = random(0,1024);
  Serial.println(analogVal);
  delay(250);
}
