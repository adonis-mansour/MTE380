#include <TimeLib.h>

// IMU Libs
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// US init
#define trigPin0 2                                   
#define echoPin0 3  
#define trigPin1 4
#define echoPin1 5

// Motor Definitoions
// Right Motors
#define in1 6
#define in2 5
#define in3 4
#define in4 3
#define en1 7
#define en2 2

// Left Motors
#define in5 9
#define in6 8
#define in7 13
#define in8 12
#define en3 10
#define en4 11

// PID Consts
#define Kp 3
#define Ki 0.5
#define Kd 0.001

// IMU Vars
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float target_angle_x = 0;

// US Vars
long duration, distance; 
int counter = 0, US = 0, US0, US1;
const int NumOfValues = 1;
int US0_values[NumOfValues]={0};
int US1_values[NumOfValues]={0};
float commands[2][2];

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
    analogWrite(en1,pwm_l);
    analogWrite(en2,pwm_l);
    analogWrite(en3,pwm_r);
    analogWrite(en4,pwm_r);
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

  int pwm_r = 200;
  int pwm_l = 200;

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

    Serial.print("PWM L: ");
    Serial.println(pwm_l);
    Serial.print("PWM R: ");
    Serial.println(pwm_r);
    
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
  int pwm_r = 140;
  int pwm_l = 140;
  
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
  int pwm_r = 140;
  int pwm_l = 140;
  
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

float goStraightContinuous(int dir) {
  int delayTime_millis = 50;
  float angle_x = 0;
  float prev_angle_x = 0;
  float error = 0, errorSum = 0;
  float sampleTime = 0.05;
  unsigned long currTime = 0, prevTime = 0;

  int pwm_r = 200;
  int pwm_l = 200;

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


  while (US0 >= 2 || US1 >=2) {
    currTime = millis();
    sampleTime = (currTime - prevTime)/1000.0;
    prevTime = currTime;

    Serial.print("PWM L: ");
    Serial.println(pwm_l);
    Serial.print("PWM R: ");
    Serial.println(pwm_r);
    
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
}

int SonarSensor(int sensor_num, int trigPinSensor,int echoPinSensor, int values[]) {

  pinMode(trigPinSensor, OUTPUT);
  pinMode(echoPinSensor, INPUT);   
 
  digitalWrite(trigPinSensor, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinSensor, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinSensor, LOW);

  duration = pulseIn(echoPinSensor, HIGH);
  distance= (duration/2) / 29.1;  

  values[counter] = distance;
  US = 0;
  for (int i=0; i<NumOfValues; i++) {
    US = US+values[i];
  }
  US = (US)/NumOfValues;
  return US;
  
//  Serial.print(US);
//  Serial.print(" cm      ");
  delay(50); 


}

int time() {
  int h = hour();
  int m = minute();
  int s = second();

  int t = h*60*60 + m*60 + s;
  return t;
}

void loop(void) 
{
  //Serial.print("Sensor FR: ");
  US0 = SonarSensor(0, trigPin0, echoPin0, US0_values);                                  

  //Serial.print("Sensor FL: ");
  US1 = SonarSensor(1, trigPin1,echoPin1,  US1_values); 

  if (US0 > 2 || US1 > 2) {
      float time_start = time();
      //DRIVE FORWARD
      float time_end = goStraightContinuous(1);

     commands[0][0] = 1;
     commands[0][1] = time_end - time_start;

      delay(500);
    }
    if (US1 - US0 >= 3) {
      float time_start = time();
      //TURN CW
      while (US1 - US0 >= 3) {
        continuousAngle(1);
      }
      float time_end = time();

     commands[1][0] = 1;
     commands[1][1] = time_end - time_start;
    } else if (US0 - US1 >= 3) {
      float time_start = time();
      //TURN CCW
      while (US1 - US0 >= 3) {
        continuousAngle(0);
      }
      float time_end = time();

      commands[1][0] = 0;
      commands[1][1] = time_end - time_start;
    }
  
  delay(500);
  goStraight(0, 50);
  delay(500);
  turnAngle(0, 90);
  delay(500);
  turnAngle(1,90);
  delay(500);
  goStraight(1,50);
  
  while(true);
}
