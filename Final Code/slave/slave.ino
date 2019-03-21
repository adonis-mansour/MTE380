#include <SparkFun_RFD77402_Arduino_Library.h>

//  COLOUR SENSOR DEFS
#define S0 8
#define S1 9
#define S2 10
#define S3 11
#define sensorOut 12

//COLOUR SENSOR DATA TRANSFER
#define Colour_REQUEST 25
#define Colour_SENT 50

// TOF SENSOR DATA TRANSFER
#define TOF_REQUEST 23
#define TOF_SENT 22

//BACKUP DATA TRANSFER
// (NOT NECESSARY - BACKUP)

//TOF SENSOR INIT
RFD77402 myDistance; //Hook object to the library

//  COLOUR SENSOR CONSTANTS
int R, G, B, R_counter=0, G_counter=0, B_counter=0;
const int Colour_NumOfValues = 3;
int R_values[Colour_NumOfValues]={0};
int G_values[Colour_NumOfValues]={0};
int B_values[Colour_NumOfValues]={0};
int frequency = 0;

//SENDING DATA TOF
void sendDataTOF(int myint){
  byte data[2] = {0};
  data[0] = (byte) (myint & 0xFF);
  data[1] = (byte) ((myint >> 8) & 0xFF);
  
  Serial.write(data, 2);
  digitalWrite(TOF_SENT, HIGH);
  while(digitalRead(TOF_REQUEST));
  digitalWrite(TOF_SENT, LOW);
}

//SENDING COLOUR SENSOR DATA

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

  pinMode(Colour_REQUEST, INPUT);
  pinMode(Colour_SENT, OUTPUT);  
  pinMode(TOF_REQUEST, INPUT);
  pinMode(TOF_SENT, OUTPUT);  
}
int DetectColour() {
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
//  Serial.print("R= ");//printing name
//  Serial.print(R);//printing RED color frequency
//  Serial.print("  ");
//  delay(100);

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
//  Serial.print("G= ");//printing name
//  Serial.print(G);//printing RED color frequency
//  Serial.print("  ");
//  delay(100);

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
//  Serial.print("B= ");//printing name
//  Serial.print(B);//printing RED color frequency
//  Serial.print("  ");
//  delay(100);

  if ((B-R) >= 15 && (B-R) >= 0) {
    //Serial.print("Colour: Red");
    return 1; //RED house detected    
  } else if ((B-R) <=14 && (B-R) >= 0 && G<15) {
    //Serial.print("Colour: Yellow");
    return 0; //YELLOW house detected   
  } else {
    return 2;
  }
 
//  else if ((R-B) < 9 && (R-B) >= 0) {
//    Serial.print("Colour: Blue");
//    return 3; //ERROR
//  } else {
//    return 2; //CANDLE detected
//  }
}

// send colour data
void sendDataColour(int myint){
  byte data[1] = {0};
  data[0] = (byte) (myint & 0xFF);
  
  Serial.write(data, 1);
  digitalWrite(Colour_SENT, HIGH);
  while(digitalRead(Colour_REQUEST)); //while still requested do nothing
  digitalWrite(Colour_SENT, LOW);
}

//TOF SENSOR FUNCTIONS
void init_tof()
{ 
  pinMode(TOF_REQUEST, INPUT);
  pinMode(TOF_SENT, OUTPUT);
  if (myDistance.begin() == false)
  {
    Serial.println("Sensor failed to initialize. Check wiring.");
    //while (1); //Freeze!
  }
}

int getDistance_tof()
{
  myDistance.takeMeasurement(); //Tell sensor to take measurement
  return myDistance.getDistance();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(19200);
  init_tof();
  init_colour_sensor();
}

void loop() {
  if (digitalRead(TOF_REQUEST) == HIGH) {
    sendDataTOF(getDistance_tof());
  }
  if (digitalRead(Colour_REQUEST) == HIGH) {
    sendDataColour(DetectColour());
  } 
}
