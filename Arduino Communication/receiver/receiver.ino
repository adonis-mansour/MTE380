int requestLine = 31;
int dataReceived = 33;

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

void setup() {
  Serial.begin(19200);

  pinMode(requestLine, OUTPUT);
  pinMode(dataReceived, INPUT);
}

void loop() {
  Serial.println(requestData(), DEC);
}
