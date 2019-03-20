#define trigPin 7  //Trigger                                    

#define echoPin0 4  //Front_R                                     
#define echoPin1 11 //Front_L                                 
#define echoPin2 13 //Right_R
#define echoPin3 12 //Right_L
#define echoPin4 2  //Left_R
#define echoPin5 3  //Left_L
#define echoPin6 6  //Rear_R
#define echoPin7 8  //Rear_L

long duration, distance; 
int counter = 0, US = 0;
boolean sensors_detected[8] = {0};
boolean object_location[4] = {0};

const int NumOfValues = 1;
int US0_values[NumOfValues]={0};
int US1_values[NumOfValues]={0};
int US2_values[NumOfValues]={0};
int US3_values[NumOfValues]={0};
int US4_values[NumOfValues]={0};
int US5_values[NumOfValues]={0};
int US6_values[NumOfValues]={0};
int US7_values[NumOfValues]={0};

void setup() {
  Serial.begin(9600);
  
}


void loop() {
  //Serial.print("Sensor FR: ");
  SonarSensor(0, trigPin, echoPin0, US0_values);                                  

  //Serial.print("Sensor FL: ");
  SonarSensor(1, trigPin,echoPin1,  US1_values);   

  //Serial.print("Sensor RR: ");
  SonarSensor(2, trigPin,echoPin2,  US1_values);   
  
  //Serial.print("Sensor RL: ");
  SonarSensor(3, trigPin,echoPin3,  US1_values);   
  
  //Serial.print("Sensor LR: ");
  SonarSensor(4, trigPin,echoPin4,  US1_values);   
  
  //Serial.print("Sensor LL: ");
  SonarSensor(5, trigPin,echoPin5,  US1_values);   
  
  //Serial.print("Sensor BR: ");
  SonarSensor(6, trigPin,echoPin6,  US1_values);   
  
  //Serial.print("Sensor BL: ");
  SonarSensor(7, trigPin,echoPin7,  US1_values);   
  

  if (sensors_detected[0] == true || sensors_detected[1]== true) {
    object_location[0] = true;
    Serial.print("Front: Detected     ");
  } else {
    object_location[0] = false;
    Serial.print("Front: Nothing      ");
  }

  if (sensors_detected[2] == true || sensors_detected[3]== true) {
    object_location[1] = true;
    Serial.print("Right: Detected     ");

  } else {
    object_location[1] = false;
    Serial.print("Right: Nothing      ");
  }

  if (sensors_detected[4] == true || sensors_detected[5]== true) {
    object_location[2] = true;
    Serial.print("Left: Detected     ");

  } else {
    object_location[2] = false;
    Serial.print("Left: Nothing       ");
  }

  if (sensors_detected[6] == true || sensors_detected[7]== true) {
    object_location[3] = true;
    Serial.print("Back: Detected     ");
  } else {
    object_location[3] = false;
    Serial.print("Back: Nothing     ");
  }
  
  counter = (counter + 1)%NumOfValues;

  Serial.println("   ");

  delay(50);
}

void SonarSensor(int sensor_num, int trigPinSensor,int echoPinSensor, int values[]) {

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
  
//  Serial.print(US);
//  Serial.print(" cm      ");
  delay(50); 

  if (US <= 30) {
    sensors_detected[sensor_num] = true;
  } else {
    sensors_detected[sensor_num] = false;
  }

}
