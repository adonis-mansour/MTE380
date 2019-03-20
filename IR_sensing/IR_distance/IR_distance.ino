#define sensor0 A2  //F                              // Sharp IR GP2Y0A41SK0F (4-30cm, analog)
#define sensor1 A3  //R
#define sensor2 A4  //L
#define sensor3 A6  //B

int IR_counter = 0;
float volts = 0;
int IR_distance = 0;
int terrain[4]; //0 = flat, 1 = sand, 2 = gravel, 3 = pit

const int NumOfValues = 5;
int IR0_values[NumOfValues]={0};
int IR1_values[NumOfValues]={0};
int IR2_values[NumOfValues]={0};
int IR3_values[NumOfValues]={0};
int IR, IR0, IR1, IR2, IR3;

void setup() {
  Serial.begin(9600);                         // start the serial port
  Serial.println("START");
 
}

float Read_Distance(int sensor, int values[]) {
  volts = analogRead(sensor)*0.0048828125;  // value from sensor * (5/1024)
    IR_distance = 13*pow(volts, -1);             // worked out from datasheet graph
    delay(10); // slow down serial port 

    values[IR_counter] = IR_distance;
    IR = 0;
    for (int i=0; i<NumOfValues; i++) {
      IR = IR+values[i];
    }
    IR = (IR)/NumOfValues;
       
    Serial.print(IR);               // print the IR_distance   
    return IR;
}
int detect_terrain() {
  //Front
  Serial.print("Front    ");
  terrain[0] = Read_Distance(sensor0, IR0_values);
  Serial.print("   ");
  
  //Right
  Serial.print("Right    ");
  terrain[1] = Read_Distance(sensor1, IR1_values);
  Serial.print("   ");
  
  //Left
  Serial.print("Left    ");
  terrain[2] = Read_Distance(sensor2, IR2_values);
  Serial.print("   ");
  
  //Back
  Serial.print("Rear    ");
  terrain[3] = Read_Distance(sensor3, IR3_values);
  IR_counter = (IR_counter + 1)%NumOfValues;
  Serial.print("   ");
  
  delay(50); 
}

void loop() {
  detect_terrain();
  
//  Serial.print("Front    ");
//  Read_Distance(sensor1, IR1_values);
//  Serial.print("   ");
//
//  Serial.print("Right    ");
//  Read_Distance(sensor2, IR2_values);
//  Serial.print("   ");
//
//   Serial.print("Left    ");
//  Read_Distance(sensor4, IR4_values);
//  Serial.print("   ");
//
//  Serial.print("Rear    ");
//  Read_Distance(sensor3, IR3_values);
//  Serial.print("   ");
//
//  Serial.println("   ");
//  
//  counter = (counter + 1)%NumOfValues;
//  delay(50);
}
