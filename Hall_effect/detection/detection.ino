int pin1 = 0;
int pin2 = 1;
int pin3 = 2;
int pin4 = 3;
int pin5 = 4;
int pin6 = 5;
int pin7 = 6;
int pin8 = 7;
int pin9 = 8;

bool Mag_detect() {

  long NOFIELD = 512;
  float TOMILLIGAUSS = 3.756010;
  //int pin_array[9] = {pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8, pin9};
  float gauss[9] = {0};
  int mag = 0;
  int no_mag = 0;
  int threshold = 7;

  for (int i = 0; i < 13; i++ ) {
    delay(100);
    gauss[0] = (analogRead(pin1) - NOFIELD) * TOMILLIGAUSS;
    Serial.print(gauss[0]);
    Serial.println(i);
//    gauss[1] = (analogRead(pin2) - NOFIELD) * TOMILLIGAUSS;
//    gauss[2] = (analogRead(pin3) - NOFIELD) * TOMILLIGAUSS;
//    gauss[3] = (analogRead(pin4) - NOFIELD) * TOMILLIGAUSS;
//    gauss[4] = (analogRead(pin5) - NOFIELD) * TOMILLIGAUSS;
//    gauss[5] = (analogRead(pin6) - NOFIELD) * TOMILLIGAUSS;
//    gauss[6] = (analogRead(pin7) - NOFIELD) * TOMILLIGAUSS;
//    gauss[7] = (analogRead(pin8) - NOFIELD) * TOMILLIGAUSS;
//    gauss[8] = (analogRead(pin9) - NOFIELD) * TOMILLIGAUSS;

//    if ( (gauss[0] < -7) or (gauss[1] < -7) or (gauss[2] < -7) or (gauss[3] < -7) or (gauss[4] < -7) or (gauss[5] < -7) or (gauss[6] < -7) or (gauss[7] < -7) or (gauss[8] < -7)) {
//      mag++;
//    }
//    else if ( (gauss[0] > 7) or (gauss[1] > 7) or (gauss[2] > 7) or (gauss[3] > 7) or (gauss[4] > 7) or (gauss[5] > 7) or (gauss[6] > 7) or (gauss[7] > 7) or (gauss[8] > 7)) {
//      mag++;
//    }
//    else {
//      no_mag++;
//    }

  if (gauss[0] > threshold | gauss[0] < -threshold) mag++;


  }

  if (mag > 2) return 1;
//  if (mag >= 3)
//    return 1;
//  else if (no_mag >= 10)
//    return 0;
  return 0;
}

void setup()
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop()
{
  if (Mag_detect()) digitalWrite(LED_BUILTIN,HIGH);
  //Serial.print("Hello");
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
  
}
