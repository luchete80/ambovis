#include <LiquidCrystal.h>

#define PIN_LCD_RS 8
#define PIN_LCD_EN 9
#define PIN_LCD_D4 10
#define PIN_LCD_D5 11
#define PIN_LCD_D6 12
#define PIN_LCD_D7 13

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

//including numbers of interface bins
const int analogInPin = A1; //Analog input pin, connected to pressure sensor

//Initialize Variables
float inputVoltage = 0; //Voltage read from pressure sensor (in bits 0-1023)
float volt0 = 2.5; //Initial Voltage
float volt = 0; //Voltage (converted from 0-255 to 0-5)
float pressure_psi = 0; //Pressure value calculated from voltage--in Psi
float pressure_pa = 0; //Pressure value converted to Pa
float massFlow = 0;// Mass flow rate calculated from pressure
float volFlow = 0;// Calculated from mass flow rate
float volume = 0;// Integral of flow rate over time
//Constants
float vs = 5; //Voltage powering pressure sensor
float rho = 1.225; //Density of air in kgm3
float area_1 = 0.0087969; //surface area in m2
float area_2 = 0.0039985; //surface area in m2
float dt = 0;
int button = 0;//Value of button

#define PSI_2_CMH2O   70.306957964239

void setup() {
//   put your setup code here, to run once
//  Set up the LCD's number of columns and rows
  lcd.begin(20,4);
  lcd.print("Volume = ");
  Serial.begin(9600);
}

void loop() {
   // put your main code here, to run repeatedly
  // Check if button is pressed, if so enter program condition
  lcd.setCursor(0,1);
//  button = analogRead(analogButton);

    inputVoltage = analogRead(analogInPin); //Voltage read in (0 to 1023)
    volt = inputVoltage*5./1023.0;
    pressure_psi = (15/2)*(volt-2.09);// Pressure in Psi
    pressure_pa = pressure_psi*6894.75729; //Pressure in Pascals
    Serial.print("Read: ");Serial.print(inputVoltage);Serial.print("Volt");//ln(volt);
    //get number that isnt exactly 2.5 becuse of calibrations that gave exact voltagepressure_pa = pressure_psi6894.75729; Pressure in Pascals
    //massFlow = 1000.*sqrt((abs(pressure_pa)*2*rho)((1/(pow(area_2,2)))-(1(pow(area_1,2))))); //Mass
    //flow of air
    //use factor of 1000 so you end with units of Ls
    //volFlow = massFlowrho; //Volumetric flow of air
    //volume = volFlowdt + volume; //Total volume
    //dt = 0.001;
    //delay(100);

  lcd.print(volume);
}
