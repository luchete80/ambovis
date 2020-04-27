#define TIMEOUT (10000UL)     // Maximum time to wait for serial activity to start

#define DT 50 // Loop sleep time (ms)
#define CYCLE 60

int count;
bool led_on;

void setup() {
  // put your setup code here, to run once:

  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output

Serial.detectBaudrate();
  Serial.begin(115200);

   Serial.detectBaudrate();// may also be called before Serial.begin()
  // There must be activity on the serial port for the baudrate to be detected

//  unsigned long detectedBaudrate = Serial.detectBaudrate(TIMEOUT);

//  if (detectedBaudrate) {
//    Serial.printf("\nDetected baudrate is %lu, switching to that baudrate now...\n", detectedBaudrate);
//
//    // Wait for printf to finish
//    while (Serial.availableForWrite() != UART_TX_FIFO_SIZE) {
//      yield();
//    }
//
//    // Clear Tx buffer to avoid extra characters being printed
//    Serial.flush();
//
//    // After this, any writing to Serial will print gibberish on the serial monitor if the baudrate doesn't match
//    Serial.begin(detectedBaudrate);
//  } else {
//    Serial.println("\nNothing detected");
//  }
//
//  led_on = false;
//  count = 0;
//  digitalWrite(LED_BUILTIN, HIGH); // Active is low in the ESP
}

void loop() {

  count++;
  int pressure1 = (int)(sin(2.0 * 3.1416 * ((float)(count) / ((float) CYCLE) )) * 30+2);
  int pressure2 = 0;
  int volume = 0;
  int flow = (int)(cos(2.0 * 3.1416 * ((float)(count) / ((float) CYCLE) )) * 20000 + 50000);
  if (count == CYCLE)
  {
    count = 0;
    if (led_on) {
      digitalWrite(LED_BUILTIN, HIGH);
      led_on = false;
    }
    else {
      digitalWrite(LED_BUILTIN, LOW);
      led_on = true; 
    }

  }
  
  char string[100];
  sprintf(string, "DT %05d %05d %05d %06d", pressure1, pressure2, volume, flow);
  //sprintf(string, "%05d %05d", pressure1, pressure2);
  Serial.println(string);
  
  delay(DT);                      // Wait for two seconds (to demonstrate the active low LED)

}
