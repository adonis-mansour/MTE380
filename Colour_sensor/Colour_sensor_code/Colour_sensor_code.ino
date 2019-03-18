#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

int R, G, B, R_counter=0, G_counter=0, B_counter=0;
const int NumOfValues = 3;
int R_values[NumOfValues]={0};
int G_values[NumOfValues]={0};
int B_values[NumOfValues]={0};
int frequency = 0;

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  
  Serial.begin(9600);
}

void DetectColour() {
    // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  R_values[R_counter] = pulseIn(sensorOut, LOW);
  R = 0;
  for (int i=0; i<NumOfValues; i++) {
    R = R+R_values[i];
  }
  R = (R)/NumOfValues;
  R_counter = (R_counter + 1)%NumOfValues;
  Serial.print("R= ");//printing name
  Serial.print(R);//printing RED color frequency
  Serial.print("  ");
  delay(100);

  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  G_values[G_counter%NumOfValues] = pulseIn(sensorOut, LOW);
  G = 0;
  for (int i=0; i<NumOfValues; i++) {
    G = G+G_values[i];
  }
  G = (G)/NumOfValues;
  G_counter = (G_counter + 1)%NumOfValues;
  Serial.print("G= ");//printing name
  Serial.print(G);//printing RED color frequency
  Serial.print("  ");
  delay(100);

  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  B_values[B_counter%NumOfValues] = pulseIn(sensorOut, LOW);
  B = 0;
  for (int i=0; i<NumOfValues; i++) {
    B = B+B_values[i];
  }
  B = (B)/NumOfValues;
  B_counter = (B_counter + 1)%NumOfValues;
  Serial.print("B= ");//printing name
  Serial.print(B);//printing RED color frequency
  Serial.print("  ");
  delay(100);

  if ((B-R) >= 15 && (B-R) >= 0) {
    Serial.print("Colour: Red");
  } else if ((B-R) <=14 && (B-R) >= 0 && G<15) {
    Serial.print("Colour: Yellow");
  } else if ((R-B) < 9 && (R-B) >= 0) {
    Serial.print("Colour: Blue");
  } 
  Serial.println("  ");
}

void loop() {

  DetectColour();
}
