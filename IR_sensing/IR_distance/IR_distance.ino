#define sensor1 A0                             // Sharp IR GP2Y0A41SK0F (4-30cm, analog)
#define sensor2 A1  
#define sensor3 A2  
#define sensor4 A3  

int counter = 0;
float volts = 0;
int distance = 0;

const int NumOfValues = 3;
int IR1_values[NumOfValues]={0};
int IR2_values[NumOfValues]={0};
int IR3_values[NumOfValues]={0};
int IR4_values[NumOfValues]={0};
int IR, IR1, IR2, IR3, IR4;

void setup() {
  Serial.begin(9600);                         // start the serial port
  Serial.println("START");
 
}

float Read_Distance(int sensor, int values[]) {
  volts = analogRead(sensor)*0.0048828125;  // value from sensor * (5/1024)
    distance = 13*pow(volts, -1);             // worked out from datasheet graph
    delay(10); // slow down serial port 

    values[counter] = distance;
    IR = 0;
    for (int i=0; i<NumOfValues; i++) {
      IR = IR+values[i];
    }
    IR = (IR)/NumOfValues;
    
    
    if (IR <= 30){
      Serial.println(IR);               // print the distance
    }
    else {
      Serial.println(-1);
    }
}


void loop() {
  Serial.print("Front    ");
  Read_Distance(sensor1, IR1_values);

  Serial.print("Left    ");
  Read_Distance(sensor2, IR2_values);

  Serial.print("Right    ");
  Read_Distance(sensor3, IR3_values);

  Serial.print("Rear    ");
  Read_Distance(sensor4, IR4_values);

  counter = (counter + 1)%NumOfValues;
    
}
