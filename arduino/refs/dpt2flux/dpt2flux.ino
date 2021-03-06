
float dp[]={-9.709861146,-8.828793439,-7.868898584,-7.118529669,-6.295077605,-5.530065589,-4.803049529,-4.196388314,-3.480106395,-2.991823428,-2.444452733,-2.030351958,-1.563385753,-1.207061607,-0.877207832,-0.606462279,-0.491216024,-0.377891785,-0.295221736,-0.216332764,-0.151339196,-0.096530072,-0.052868293,-0.047781395,-0.039664506,-0.03312327,-0.028644966,-0.023566372,-0.020045692,-0.014830113,-0.011688636,-0.008176254,-0.006117271,-0.003937171,-0.001999305,-0.00090924,-0.00030358,0,0.000242233,0.000837976,0.002664566,0.004602432,0.007024765,0.009325981,0.012111664,0.01441288,0.017561913,0.023012161,0.029794693,0.037061691,0.043771552,0.051474571,0.05874157,0.109004974,0.176879848,0.260808033,0.365700986,0.504544509,0.630753349,0.795599072,1.216013465,1.60054669,2.087678384,2.547210457,3.074176245,3.676588011,4.385391541,5.220403813,5.947168311,6.794489065,7.662011691,8.642594913,9.810447693,10.7793808,11.95257389};
int po_flux[]={-200,-190,-180,-170,-160,-150,-140,-130,-120,-110,-100,-90,-80,-70,-60,-50,-45,-40,-35,-30,-25,-20,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,0,0,0,0,0,3,4,5,6,7,8,9,10,11,12,13,14,15,20,25,30,35,40,45,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200};

float p_dpt;
#define DEFAULT_PA_TO_CM_H20 0.0102F

int findClosest2(float arr[], int n, float target) { 
    int i = 0, j = n-1, mid = 0; 
    while (i < j) { 
        mid = (i + j) / 2;  
        if (target < arr[mid]) { 
            j = mid - 1; 
        } else {       // If target is greater than mid 
            i = mid + 1;  } 
        Serial.print("i,j: ");Serial.print(i);Serial.print(",");Serial.println(j);
        } 
    return mid;
} 

int findClosest3(float arr[], int n, float target) { 
    int i = 0, j = n-1, mid = 0; 
    while ( j - i > 1 ) { 
        mid = (i + j) / 2;  
        if (target < arr[mid]) { 
            j = mid; 
        } else {       // If target is greater than mid 
            i = mid;  } 
        Serial.print("i,j: ");Serial.print(i);Serial.print(",");Serial.println(j);
        } 
    return i;
} 

int getClosest(int val1, int val2, int target) { 
    if (target - val1 >= val2 - target) 
        return val2; 
    else
        return val1; 
} 

void setup()
{
	Serial.begin(115200);

 Serial.print("dp length: ");Serial.println(sizeof(dp));
}

int pos;

float res;
void loop ()
{
	//p_dpt=200.*float(analogRead(A1))*DEFAULT_PA_TO_CM_H20*100./1024.; //From DPT, AS MAX RANGE (100 Pa or more) //PA TO CMH2O
	
	
	p_dpt=random(-9, 11);
  //p_dpt=-7;
	Serial.print("pdpt is ");Serial.println(p_dpt);
 
	//int findClosest(float arr[], int n, float target) 
	pos=findClosest3(dp,75,p_dpt);
	Serial.print("Position is ");Serial.println(pos);

  res=po_flux[pos]+(po_flux[pos+1]-po_flux[pos])*(p_dpt-dp[pos])/(dp[pos+1]-dp[pos]);
	
	Serial.print("Flux        is ");Serial.println(po_flux[pos]);
  Serial.print("InterpFlux  is ");Serial.println(res);
  
  Serial.println("***************************");
	delay(500);
}
