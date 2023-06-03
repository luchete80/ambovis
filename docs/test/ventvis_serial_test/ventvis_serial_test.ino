#define DT 100 // Loop sleep time (ms)
#define CYCLE 127

int count;

void setup() {
  Serial.begin(115200);
  count = 0;
}

int scale=20;
void loop() {

  count++;
  float p_bmp = fabs(sin(2.0 * 3.1416 * ((float)(count) / ((float) CYCLE) ))) * scale+2.;
  float p_honey = p_bmp + 5;
  float p_dpt = fabs(sin(2.0 * 3.1416 * ((float)(count) / ((float) CYCLE) ))) * 30+2;
  float flux = fabs(cos(2.0 * 3.1416 * ((float)(count) / ((float) CYCLE) ))) * 1000;
  float vol = fabs(cos(2.0 * 3.1416 * ((float)(count) / ((float) CYCLE) ))) * 700;
  if (count == CYCLE){
    count = 0;
    scale++;
  }
  char buffer[50];
  //sprintf(buffer, "<%d,%d,%d>\n", count,int(p_bmp), int(flux), int(vol));
  //Serial.print(buffer);
//    
  Serial.print(count);Serial.print(",");Serial.print(int(p_bmp));Serial.print(",");Serial.println(int(vol));
//  
  delay(DT); // Wait for two seconds

}
