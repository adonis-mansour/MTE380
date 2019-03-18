#define trigPin 12                                  
#define echoPin1 11                                  
#define echoPin2 13
//#define echoPin3 5

long duration, distance, UltraSensor1, UltraSensor2, UltraSensor3; 
int counter = 0, US = 0;

const int NumOfValues = 3;
int US1_values[NumOfValues]={0};
int US2_values[NumOfValues]={0};
int US3_values[NumOfValues]={0};
int US4_values[NumOfValues]={0};
int US5_values[NumOfValues]={0};
int US6_values[NumOfValues]={0};
int US7_values[NumOfValues]={0};
int US8_values[NumOfValues]={0};

void setup() {
  
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  
}


void loop() {
  Serial.print("Sensor 1: ");
  SonarSensor(trigPin, echoPin1, US1_values);              
  UltraSensor1 = distance; 
                     

  Serial.print("Sensor 2: ");
  SonarSensor(trigPin,echoPin2,  US2_values);              
  UltraSensor2 = distance; 
  
    
//  Serial.print("Sensor 3: ");
//  SonarSensor(trigPin,echoPin3);              
//  UltraSensor3 = distance; 
  
  counter = (counter + 1)%NumOfValues;

}

void SonarSensor(int trigPinSensor,int echoPinSensor, int values[]) {

  pinMode(trigPinSensor, OUTPUT);
  pinMode(echoPinSensor, INPUT);   
 
  digitalWrite(trigPinSensor, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinSensor, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinSensor, LOW);

  duration = pulseIn(echoPinSensor, HIGH);
  distance= (duration/2) / 29.1;  

  values[counter] = distance;
  US = 0;
  for (int i=0; i<NumOfValues; i++) {
    US = US+values[i];
  }
  US = (US)/NumOfValues;
  
  Serial.print(US);
  Serial.println(" cm");
  delay(250); 

}
