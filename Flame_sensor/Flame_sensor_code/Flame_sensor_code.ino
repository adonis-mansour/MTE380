#define FLAME1 53 
#define FLAME2 51 
#define FLAME3 50 
#define FLAME4 52 
#define FAN 49

void setup() {
  Serial.begin(9600);
  pinMode(FLAME1, INPUT);
  pinMode(FLAME2, INPUT);
  pinMode(FLAME3, INPUT);
  pinMode(FLAME4, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FAN, OUTPUT);

  //delay(5000);
}

bool Detect_Flame() {
  int fire1 = digitalRead(FLAME1);
  int fire2 = digitalRead(FLAME2);
  int fire3 = digitalRead(FLAME3);
  int fire4 = digitalRead(FLAME4);

  if (fire1 == LOW) {
    Serial.print("Fire1    ");
  }

  if (fire2 == LOW) {
    Serial.print("Fire2    ");
  }

  if (fire3 == LOW) {
    Serial.print("Fire3    ");
  }

  if (fire4 == LOW) {
    Serial.print("Fire4    ");
  }
  if(  fire4 == LOW)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(FAN, HIGH);
    Serial.println("Fire");
    return true;
    delay(200); // ??? IS THIS DELAY REQUIRED?
    
  }else{
    Serial.println("Nothing");
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(FAN, LOW);
    return false;
  }

  delay(200);
}

void loop() {
  Detect_Flame();
  
}
