unsigned int myint = 100;
int dataRequest = 13;
int dataSent = 12;

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
  Serial.begin(19200);
  pinMode(dataRequest, INPUT);
  pinMode(dataSent, OUTPUT);
}

void loop() {  
  if(digitalRead(dataRequest))  sendData(myint);
}
