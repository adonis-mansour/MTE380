// TX-RX COMMUNICATION
#define dataRequest 13
#define dataSent 12

//  COLOUR SENSOR DEFS
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

#define Colour_REQUEST 48
#define Colour_RED_HOUSE 49
#define Colour_YELLOW_HOUSE 50
#define Colour_OTHER 51
#define Colour_SENT 52

//  FRONT ULTRASONIC DEFS
#define trigPin0 9   //FR                                
#define echoPin0 10    
#define trigPin1 13    //FL                               
#define echoPin1 11

#define US_REQUEST 23
#define US_DATA 25
#define US_SENT 22



//  COLOUR SENSOR CONSTANTS
int R, G, B, R_counter=0, G_counter=0, B_counter=0;
const int Colour_NumOfValues = 3;
int R_values[Colour_NumOfValues]={0};
int G_values[Colour_NumOfValues]={0};
int B_values[Colour_NumOfValues]={0};
int frequency = 0;

//  FRONT ULTRASONIC DEFS
long duration, distance; 
int US_counter = 0, US = 0;
boolean sensors_detected[2] = {0};
bool object_location = false;

const int US_NumOfValues = 3;
int US0_values[US_NumOfValues]={0};    //FR
int US1_values[US_NumOfValues]={0};    //FL



//  MOTOR FUNCTIONS

//  FRONT ULTRASONIC 
void init_US_sensor() {
  pinMode(US_REQUEST, INPUT);
  pinMode(US_DATA, OUTPUT);
  pinMode(US_SENT, OUTPUT);
  
}
//void front_sensors_request() {
//  //while (digitalRead(US_REQUEST) == LOW) {} //Redundant???
//  if (object_location) {
//    Serial.println("US DATA HIGH");
//    digitalWrite(US_DATA, HIGH);
//  } else {
//    digitalWrite(US_DATA, LOW);
//  }
//  digitalWrite(US_SENT, HIGH);
//  while (digitalRead(US_REQUEST) == HIGH) {}
//  digitalWrite(US_DATA, LOW);
//  digitalWrite(US_SENT, LOW);
//}
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

  values[US_counter] = distance;
  US = 0;
  for (int i=0; i<US_NumOfValues; i++) {
    US = US+values[i];
  }
  US = (US)/US_NumOfValues;
  delay(50); 

  if(digitalRead(dataRequest) {
    sendData(          );
  }
  
  
//  if (US <= 30) {
//    sensors_detected[sensor_num] = true;
//  } else {
//    sensors_detected[sensor_num] = false;
//  }
}
void detect_objects() {
  //Serial.print("Sensor FR: ");
  SonarSensor(0, trigPin0,echoPin0,  US0_values);   
  
  //Serial.print("Sensor FL: ");
  SonarSensor(1, trigPin1,echoPin1,  US1_values);   
//  if (sensors_detected[0] == true || sensors_detected[1]== true) {
//    object_location = true;
//    Serial.println("Front: Detected     "); // wtf was dhruv thinking here TODO TO DO alhamdulilah
//
//  } else {
//    object_location = false;
//    Serial.println("Front: Nothing      "); // dhruv wtf again TODO TO DO bismillah
//  }    
  US_counter = (US_counter + 1)%US_NumOfValues;
  return object_location;
}

//  COLOUR SENSOR
void init_colour_sensor() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

  pinMode(Colour_REQUEST, OUTPUT);
  pinMode(Colour_RED_HOUSE, INPUT);
  pinMode(Colour_YELLOW_HOUSE, INPUT);
  pinMode(Colour_OTHER, INPUT);
  pinMode(Colour_SENT, INPUT);  
}
void DetectColour() {
    // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  R_values[R_counter] = pulseIn(sensorOut, LOW);
  R = 0;
  for (int i=0; i<Colour_NumOfValues; i++) {
    R = R+R_values[i];
  }
  R = (R)/Colour_NumOfValues;
  R_counter = (R_counter + 1)%Colour_NumOfValues;
  Serial.print("R= ");//printing name
  Serial.print(R);//printing RED color frequency
  Serial.print("  ");
  delay(100);

  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  G_values[G_counter%Colour_NumOfValues] = pulseIn(sensorOut, LOW);
  G = 0;
  for (int i=0; i<Colour_NumOfValues; i++) {
    G = G+G_values[i];
  }
  G = (G)/Colour_NumOfValues;
  G_counter = (G_counter + 1)%Colour_NumOfValues;
  Serial.print("G= ");//printing name
  Serial.print(G);//printing RED color frequency
  Serial.print("  ");
  delay(100);

  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  B_values[B_counter%Colour_NumOfValues] = pulseIn(sensorOut, LOW);
  B = 0;
  for (int i=0; i<Colour_NumOfValues; i++) {
    B = B+B_values[i];
  }
  B = (B)/Colour_NumOfValues;
  B_counter = (B_counter + 1)%Colour_NumOfValues;
  Serial.print("B= ");//printing name
  Serial.print(B);//printing RED color frequency
  Serial.print("  ");
  delay(100);

  if ((B-R) >= 15 && (B-R) >= 0) {
    Serial.print("Colour: Red");
    colour_sensor_result(1); //RED house detected    
  } else if ((B-R) <=14 && (B-R) >= 0 && G<15) {
    Serial.print("Colour: Yellow");
    colour_sensor_result(0);; //YELLOW house detected   
  } else {
    colour_sensor_result(3);
  }
 
//  else if ((R-B) < 9 && (R-B) >= 0) {
//    Serial.print("Colour: Blue");
//    return 3; //ERROR
//  } else {
//    return 2; //CANDLE detected
//  }
  Serial.println("  ");
}

void colour_sensor_result(int object) {
  while (digitalRead(Colour_REQUEST) == LOW) {}
  if (object == 0) {
    digitalWrite(Colour_YELLOW_HOUSE, HIGH);
    digitalWrite(Colour_RED_HOUSE, LOW);
    digitalWrite(Colour_OTHER, LOW); 
  } else if (object == 1) {
    digitalWrite(Colour_YELLOW_HOUSE, LOW);
    digitalWrite(Colour_RED_HOUSE, HIGH);
    digitalWrite(Colour_OTHER, LOW);    
  } else {
    digitalWrite(Colour_YELLOW_HOUSE, LOW);
    digitalWrite(Colour_RED_HOUSE, LOW);
    digitalWrite(Colour_OTHER, HIGH);    
  }
}

//homing in

// TX-RX FUNCTION
void sendData(int myint){
  byte data[2] = {0};
  data[0] = (byte) (myint & 0xFF);
  data[1] = (byte) ((myint >> 8) & 0xFF);
  
  Serial.write(data, 2);
  digitalWrite(dataSent, HIGH);
  while(digitalRead(dataRequest));
  digitalWrite(dataSent, LOW);
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  detect_objects();
  if(object_location) {
    digitalWrite(US_DATA, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else digitalWrite(US_DATA,LOW);
  Serial.println(object_location);
  //front_sensors_request();
}
