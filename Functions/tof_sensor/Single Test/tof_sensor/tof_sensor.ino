#include <SparkFun_RFD77402_Arduino_Library.h>

#define TOF_MUX 7

RFD77402 myDistance; //Hook object to the library


const int limit = 10;
float values[limit] = {0};
float sum = 0;
int counter = 0;

void setup()
{
  Serial.begin(9600);
  while (!Serial);

  init_tof();

  Serial.println("Sensor online!");
}

void init_tof()
{ 
  if (myDistance.begin() == false)
  {
    Serial.println("Sensor failed to initialize. Check wiring.");
    while (1); //Freeze!
  }
}

int getDistance_tof()
{
  myDistance.takeMeasurement(); //Tell sensor to take measurement
  return myDistance.getDistance();
}

void loop()
{ 
//  sum = sum - values[counter];
  int distance = getDistance_tof();

  
//  sum += distance;
//  values[counter] = distance;
//  counter = (counter + 1)%limit;

  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.print("mm");
    
//  Serial.print("sum: ");
//  Serial.print(sum);
//  Serial.print("  counter: ");
//  Serial.print(counter);
  Serial.println();
}
