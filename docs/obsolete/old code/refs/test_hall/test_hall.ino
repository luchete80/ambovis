int LED = 13;
int SENSOR = 0;

void setup()
{
	pinMode(LED,OUTPUT);
	pinMode(SENSOR,INPUT);
	digitalWrite(LED,LOW);
}

void loop()
{
	digitalWrite(LED,!digitalRead(SENSOR));
  if (digitalRead(SENSOR)==0)
    Serial.print("Bajo");
  else
    Serial.println("Alto");
}
