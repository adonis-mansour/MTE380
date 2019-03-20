#define FLAME1 50 
#define FLAME2 48 
#define FLAME3 51 
#define FLAME4 49 
#define FAN 52

void setup() {
  init_flame_sensor();  

  //delay(5000);
}

void init_flame_sensor() {
  Serial.begin(9600);
  pinMode(FLAME1, INPUT);
  pinMode(FLAME2, INPUT);
  pinMode(FLAME3, INPUT);
  pinMode(FLAME4, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FAN, OUTPUT);
}

void fan_control(bool mode) {
  if (mode == 1) {
    digitalWrite(FAN, HIGH);
  } else if (mode == 0) {
    digitalWrite(FAN, LOW);
  }
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
    fan_control(1);
    Serial.println("Fire");
    return true;
    //delay(200); // ??? IS THIS DELAY REQUIRED?
    
  }else{
    Serial.println("Nothing");
    fan_control(0);
    digitalWrite(FAN, LOW);
    return false;
  }

  delay(200);
}

void loop() {
  Detect_Flame();
  
}
