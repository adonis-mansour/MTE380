// IMU Libs
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Motor Definitoions
// Right Motors
#define in1 32
#define in2 30
#define in3 28
#define in4 26
#define en1 12 //RL
#define en2 11 //FL

// Left Motors
#define in5 33
#define in6 31
#define in7 29
#define in8 27
#define en3 10 // FR
#define en4 4  // RR

// PID Consts
#define Kp 3
#define Ki 0.5
#define Kd 0.001

// IMU Vars
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float target_angle_x = 0;

void setup(void) {
  Serial.begin(9600);

  // Motor Setup
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); 
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT); 
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);
  pinMode(en1, OUTPUT); // RL
  pinMode(en2, OUTPUT); // FL
  pinMode(en3, OUTPUT); // FR
  pinMode(en4, OUTPUT); // RR

  // Initial direction (forwards)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void stopMotors()
{
  analogWrite(en4, 0);
  analogWrite(en3, 0);
  analogWrite(en2, 0);
  analogWrite(en1, 0);
}

void setForwards()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
}

void setBackwards()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
}

void setClockwise()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
}

void setCounterClockwise()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
}

void setMotorPWM(int pwm_r, int pwm_l)
{
  float fl_fac = 1;
  float rl_fac = -1.6094*pow(10, -7)*pow(pwm_l, 3) + 0.00012148*pow(pwm_l, 2) - 0.026964*pwm_l + 2.5528;
  float fr_fac = -1.3899*pow(10, -7)*pow(pwm_r, 3) + 0.00010587*pow(pwm_r, 2) - 0.023731*pwm_r + 2.3867;
  float rr_fac = -3.3351*pow(10, -7)*pow(pwm_r, 3) + 0.00023117*pow(pwm_r, 2) - 0.051059*pwm_r + 4.4313;

  float pwm_rl = rl_fac * pwm_l;
  float pwm_fl = fl_fac * pwm_l;
  float pwm_fr = fr_fac * pwm_r;
  float pwm_rr = rr_fac * pwm_r;

  pwm_rl = constrain(pwm_rl, -255, 255);
  pwm_fl = constrain(pwm_fl, -255, 255);
  pwm_fr = constrain(pwm_fr, -255, 255);
  pwm_rr = constrain(pwm_rr, -255, 255);

  Serial.print("RL: ");
  Serial.print(pwm_rl);
  Serial.print("  FL: ");
  Serial.print(pwm_fl);
  Serial.print("  FR: ");
  Serial.print(pwm_fr);
  Serial.print("  RR: ");
  Serial.println(pwm_rr);
  
  analogWrite(en1,pwm_rl);
  analogWrite(en2,pwm_fl);
  analogWrite(en3,pwm_fr);
  analogWrite(en4,pwm_rr);
}

/*
 *  bool dir 0 goes forwards, 1 goes backwards
 */
void goStraight(int dir, int counter)
{
  int delayTime_millis = 50;
  float angle_x = 0;
  float prev_angle_x = 0;
  float error = 0, errorSum = 0;
  float sampleTime = 0.05;
  unsigned long currTime = 0, prevTime = 0;

  int pwm_r = 250;
  int pwm_l = 250;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  target_angle_x = euler.x();
  
  if (dir == 0)
  {
    setForwards();
  }
  else if (dir == 1)
  {
    setBackwards();
  }


  for (int i = 0; i < counter; i++)
  {
    currTime = millis();
    sampleTime = (currTime - prevTime)/1000.0;
    prevTime = currTime;

    // Motor Movement
    setMotorPWM(pwm_r, pwm_l);
  
    // IMU Data
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    angle_x = euler.x();
    
    // Note: Clockwise is +ive
    error = angle_x - target_angle_x;
  
    // Condition if angle shifts from 0 to 359...
    if (error > 300) {
      error = error - 360;
    }
    
    if (error < -300) {error = error + 360;}
  
    errorSum = errorSum + error;
    errorSum = constrain(errorSum, -300, 300);

    if (dir == 0)
    {
      pwm_r = pwm_r + ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(angle_x - prev_angle_x)/sampleTime));
      pwm_l = pwm_l - ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(angle_x - prev_angle_x)/sampleTime));
    }
    else if (dir == 1)
    {
      pwm_r = pwm_r - ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(angle_x - prev_angle_x)/sampleTime));
      pwm_l = pwm_l + ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(angle_x - prev_angle_x)/sampleTime));
    }
  
    prev_angle_x = angle_x;
  
    pwm_r = constrain(pwm_r, -255, 255);
    pwm_l = constrain(pwm_l, -255, 255);

    delay(delayTime_millis);
  }
  
  stopMotors();

  // Angle Correction
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  angle_x = euler.x();

  if (angle_x - target_angle_x > 200) {angle_x = angle_x - 360;}
  if (angle_x - target_angle_x < -200) {angle_x = angle_x + 360;}
  
  delay(100);
  
  if (abs(angle_x - target_angle_x) > 1)
  {
    if (angle_x < target_angle_x)
    {
      turnAngle(0, abs(angle_x - target_angle_x));
    }
    else if (angle_x > target_angle_x)
    {
      turnAngle(1, abs(angle_x - target_angle_x));
    }
  }
  
}

/*
 * bool dir:  0 is clockwise, 1 is counter-clockwise
 * 
 * angle MUST be positive!!!
 * 
 */
void turnAngle(bool dir, float angle)
{
  float angle_x = 0;
  int pwm_r = 250;
  int pwm_l = 250;
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  angle_x = euler.x();
  
  // Setting Direction
  if (dir == 0)
  {
    // Modifying angle to account for momentum to stop
    if (angle > 6)
    {
      angle = angle - 6;
    }
    
    angle = angle_x + angle;
    
    // Check for > 359
    if (angle > 359) {angle = angle - 360;}
 
    setClockwise();

    setMotorPWM(pwm_r, pwm_l);

    // For example, from 350 to 80 when going positive direction, go from 350 to 360, then 360=0, then 0 to 80 later
    while (angle_x - angle > 0)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      angle_x = euler.x();
    }
  
    while (angle_x < angle)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      angle_x = euler.x();
    }
  }

  else if (dir == 1)
  {
    // Modifying angle to account for momentum to stop
    if (angle > 6)
    {
      angle = angle - 5;
    }
    
    angle = angle_x - angle; // Modifying angle to account for momentum to stop
    // Check for < 0
    if (angle < 0) {angle = angle + 360;}
    
    setCounterClockwise();

    setMotorPWM(pwm_r, pwm_l);

    // For example, from 80 to 350 when going negative direction, go from 80 to 0, then 0=360, then 360 to 350 later
    while (angle_x - angle < 0)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      angle_x = euler.x();
    }
  
    while (angle_x > angle)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      angle_x = euler.x();
    }
  }
  
  stopMotors();
}

/*
 *  Continuous Move
 */
void continuousAngle(bool dir)
{
  int pwm_r = 250;
  int pwm_l = 250;
  
  // Setting Direction
  if (dir == 0)
  {
    setClockwise();
      
    setMotorPWM(pwm_r, pwm_l);
  }

  else if (dir == 1)
  {
    setCounterClockwise();
      
    setMotorPWM(pwm_r, pwm_l);
  }
}

void loop(void) 
{
  delay(500);
  goStraight(0, 50);
  delay(500);
  turnAngle(0, 180);
  delay(500);
  goStraight(0,50);
  delay(500);
  turnAngle(1, 180);


  
  while(true);
}
