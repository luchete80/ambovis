//int analoginput = 0; // our analog pin

int analogamount = 0; // stores incoming value
float percentage = 0; // used to store our percentage value
float voltage =0; // used to store voltage value

#define analoginput A2

void setup()
{
  //lcd.begin(16, 2);
  //analogReference(EXTERNAL); // use AREF for reference voltage
  analogReference(INTERNAL1V1); // use AREF for reference voltage
}

void loop()
{
  Serial.begin(115200);
  //lcd.clear();
  analogamount=analogRead(analoginput);
  percentage=(analogamount/1024.00);
  voltage=analogamount/1024.*1.1; // in millivolts
  //lcd.setCursor(0,0);
  Serial.print("% of AREF: ");
  Serial.print(percentage,2);
  Serial.print(",");
  Serial.println(voltage,2);
  delay(250);
}
