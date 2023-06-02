unsigned long cycle;
#include <EEPROM.h>

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  EEPROM.get(0,cycle);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("ciclo:");Serial.println(cycle);
}
