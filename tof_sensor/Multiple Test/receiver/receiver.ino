#include <SparkFun_RFD77402_Arduino_Library.h>

RFD77402 myDistance; //Hook object to the library

int requestLine = 31;
int dataReceived = 33;

bool limitReached = false;
const int limit = 10;
float values[limit] = {0};
float sum = 0;
int counter = 0;

unsigned int requestData(){
  int last3 = 0;
  int first3 = 0;
  digitalWrite(requestLine, HIGH);  //I want data
  while(!digitalRead(dataReceived)){ // while I haven't received anything
    while(!Serial.available());
    last3 = Serial.read();
    while(!Serial.available());
    first3 = Serial.read();
  }
  
  digitalWrite(requestLine, LOW);
  return (last3 + (first3 << 8));
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

void setup() {
  Serial.begin(38400);

  pinMode(requestLine, OUTPUT);
  pinMode(dataReceived, INPUT);

  init_tof();
}

void loop() {
  sum = sum - values[counter];
  int distance = getDistance_tof();
  
  sum += distance;
  values[counter] = distance;
  counter = (counter + 1)%limit;

  float distance2 = requestData() / 10.0;

  if (counter == (limit -1)) limitReached = true;

  if (limitReached)
  {
    Serial.print("Distance (Rec): ");
    Serial.print(sum/limit);
    Serial.print("mm  Distance (send): ");
    Serial.print(distance2);
    Serial.println("mm");
  }
}
