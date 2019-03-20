//int HE1 = 0;
//int HE1 = 1;
//int HE2 = 2;
//int HE3 = 3;
//int HE4 = 4;
//int HE5 = 5;
//int HE6 = 6;
//int HE7 = 7;
//int HE8 = 8;

#define HE0 0
#define HE1 1
#define HE2 2
#define HE3 3
#define HE4 4
#define HE5 5
#define HE6 6
#define HE7 7
#define HE8 8



bool Mag_detect() {

  long NOFIELD = 512;
  float TOMILLIGAUSS = 3.756010;
  //int pin_array[9] = {HE0, HE1, HE2, HE3, HE4, HE5, HE6, HE7, HE8};
  float gauss[9] = {0};
  int mag = 0;
  int no_mag = 0;
  int threshold = 7;

  for (int i = 0; i < 13; i++ ) {
    delay(100);
    gauss[0] = (analogRead(HE1) - NOFIELD) * TOMILLIGAUSS;
    Serial.print(gauss[0]);
    Serial.println(i);
    gauss[1] = (analogRead(HE1) - NOFIELD) * TOMILLIGAUSS;
    gauss[2] = (analogRead(HE2) - NOFIELD) * TOMILLIGAUSS;
    gauss[3] = (analogRead(HE3) - NOFIELD) * TOMILLIGAUSS;
    gauss[4] = (analogRead(HE4) - NOFIELD) * TOMILLIGAUSS;
    gauss[5] = (analogRead(HE5) - NOFIELD) * TOMILLIGAUSS;
    gauss[6] = (analogRead(HE6) - NOFIELD) * TOMILLIGAUSS;
    gauss[7] = (analogRead(HE7) - NOFIELD) * TOMILLIGAUSS;
    gauss[8] = (analogRead(HE8) - NOFIELD) * TOMILLIGAUSS;
  }

  if ( (gauss[0] < -7) or (gauss[1] < -7) or (gauss[2] < -7) or (gauss[3] < -7) or (gauss[4] < -7) or (gauss[5] < -7) or (gauss[6] < -7) or (gauss[7] < -7) or (gauss[8] < -7)) {
    mag++;
  }
  else if ( (gauss[0] > 7) or (gauss[1] > 7) or (gauss[2] > 7) or (gauss[3] > 7) or (gauss[4] > 7) or (gauss[5] > 7) or (gauss[6] > 7) or (gauss[7] > 7) or (gauss[8] > 7)) {
    mag++;
  }
  else {
    no_mag++;
  }
    
  if (mag >= 3)
    return 1;
  else if (no_mag >= 10)
    return 0;
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  if (Mag_detect()) digitalWrite(LED_BUILTIN,HIGH);
  //Serial.print("Hello");
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
  
}
