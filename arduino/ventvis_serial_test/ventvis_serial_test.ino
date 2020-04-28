#define DT 50 // Loop sleep time (ms)
#define CYCLE 60

int count;

void setup() {
  Serial.begin(115200);
  count = 0;
}

void loop() {

  count++;
  float p_bmp = fabs(sin(2.0 * 3.1416 * ((float)(count) / ((float) CYCLE) ))) * 20+2;
  float p_honey = p_bmp + 5;
  float p_dpt = fabs(sin(2.0 * 3.1416 * ((float)(count) / ((float) CYCLE) ))) * 30+2;
  float flux = fabs(cos(2.0 * 3.1416 * ((float)(count) / ((float) CYCLE) ))) * 1000;
  float vol = fabs(cos(2.0 * 3.1416 * ((float)(count) / ((float) CYCLE) ))) * 700;
  if (count == CYCLE)
    count = 0;

  Serial.print(p_bmp);Serial.print(" "); //presure CMH2O 0 - 40
  Serial.print(p_honey);Serial.print(" ");
  Serial.print(p_dpt);Serial.print(" ");
  Serial.print(flux);Serial.print(" "); // Flux ml/s 0 - 1000
  Serial.println(vol); // Volume ml 0 - 700
  
  delay(DT); // Wait for two seconds

}
