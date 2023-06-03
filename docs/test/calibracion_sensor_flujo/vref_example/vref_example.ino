// Here is a trick that allows you to check the
// power rail voltage against the 1.1V (+/- 10%)
// internal reference.
//
// Run the sketch, send 'R', and use a good meter
// to measure the voltage from the AREF pin to
// Ground.  Put the measured reference voltage
// in millivolts (should be between 1000 and 
// 1200) into the sketch as 
// 'InternalReferenceMillivolts'.
//
// Upload the (now-calibrated) sketch again and
// send 'V' to read the current "+5V" voltage 
// (should be around 5000 mV).  Compare that to
// the voltage measured at the +5V pin.  If the 
// answer is not close enough, adjust the
// InternalReferenceMillivolts to get a closer
// result.
//
// Now you can use "(getVccMillivolts()/1000.0)" 
// in place of "5.0" in your sketch to get analog
// readings less dependent on fluctuations in the
// "5V" line.

void setup()
{
  Serial.begin(115200);
  while (!Serial); // Wait for USB connection on Leonardo/Micro

  delay(1000);
  Serial.println("Send 'R' to put reference voltage on AREF pin for measurement.");
  Serial.println("Send 'V' to measure supply voltage.");
}

void loop()
{
  switch (Serial.read())
  {
    case 'r':
    case 'R':
      analogReference(INTERNAL);
      delay(500);
      Serial.println("Now measure the voltage at the AREF pin.");
      Serial.println("Put that value in this sketch as InternalReferenceMillivolts.");
      break;

    case 'v':
    case 'V':
      analogReference(DEFAULT);
      Serial.print("Power rail (millivolts): ");
      Serial.println(getVccMillivolts());
      break;
  }
}

/*VVVVVVVVVVVVV  The part below here goes into your sketch  VVVVVVVVVVVV*/

// Adjust this value to your boards specific
// internal bandgap reference voltage in millivolts
const unsigned long InternalReferenceMillivolts = 1080UL;

// Returns actual value of Vcc in millivolts
int getVccMillivolts()
{
  // Set the analog reference to DEFAULT (AVcc == Vcc power rail)
  // REFS1 REFS0          --> 0b01   -Selects DEFAULT (AVcc) reference
  // Set the analog input to channel 14: the INTERNAL bandgap reference (1.1V +/- 10%)
  // MUX3 MUX2 MUX1 MUX0  --> 0b1110 -Selects channel 14, bandgap voltage, to measure
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

  delay(50);  // Let mux settle a little to get a more stable A/D conversion

  // Start a conversion to measure the INTERNAL reference relative to the DEFAULT (Vcc) reference.
  ADCSRA |= _BV( ADSC );
  // Wait for it to complete
  while (ADCSRA & (1 << ADSC));

  // Calculate the power rail voltage (reference voltage) relative to the known voltage
  return (InternalReferenceMillivolts * 1024UL) / ADC;
}
