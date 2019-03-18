#include <SparkFun_RFD77402_Arduino_Library.h>

RFD77402 myDistance; //Hook object to the library

int dataRequest = 13;
int dataSent = 12;


bool limitReached = false;
const int limit = 10;
float values[limit] = {0};
float sum = 0;
int counter = 0;

void sendData(int myint){
  byte data[2] = {0};
  data[0] = (byte) (myint & 0xFF);
  data[1] = (byte) ((myint >> 8) & 0xFF);
  
  Serial.write(data, 2);
  digitalWrite(dataSent, HIGH);
  while(digitalRead(dataRequest));
  digitalWrite(dataSent, LOW);
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
  
  pinMode(dataRequest, INPUT);
  pinMode(dataSent, OUTPUT);

  init_tof();  
}

void loop() {  
  sum = sum - values[counter];
  int distance = getDistance_tof();

  
  sum += distance;
  values[counter] = distance;
  counter = (counter + 1)%limit;

  if (counter == (limit -1)) limitReached = true;
  if(digitalRead(dataRequest) && limitReached)
  {
    sendData((int)((sum/(float)limit)*10)); 
  }
}
