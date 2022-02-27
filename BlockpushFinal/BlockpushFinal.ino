#include <QTRSensors.h>// Library for IR sensors

// DO NOT USE PINS 15,16, 8, 3
// 15 and 16 are used for flash
//pin 4 is input ONLY

//ultrasonic sensor pins
int echol = 10;
int trigl = 9;
int echoc = 18;
int trigc = 17;
int echor = 6;
int trigr = 5;

//H-bridge pins (motor control)
#define ENA 1
#define ENB 39
#define IN1 42 //purple
#define IN2 2 //light gray
#define IN3 40 //white
#define IN4 41//black 
#define LEDPIN A2
#define BUTTONPIN A3
// Pins for Tslots:
// 6 Brown LED and 7 Black Transistor output

//speed of robot maybe add dif. speeds
#define rightmotor 255
#define leftmotor 255
#define carspeed 126
#define ramspeed 126

//initial distances on sensors are zero
int rightdistance = 0, leftdistance = 0, centerdistance = 0;
int rotCnt;// rotation counter go forward after 14
bool dir = 0;
int TIMEOUT = 7000; // ultrasonic timeout

// IR sensor setup
QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

//IR Sensor pins ********************************************
//int LeftFrontIRPin = 7;
//int RightFrontIRPin = 4;
//int LeftBackIRPin = 21;
//int RightBackIRPin =33;

// non blocking nav delay
long int motorStartTime;
long int currentMotorTime;
bool movingForward, movingRight, movingLeft = false;
int forwardDelay = 100; // delay for motor control
int turnDelay = 150;
//direction functions

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, ramspeed);
  analogWrite(ENB, ramspeed);
  // Serial.println("forward");
}

void back() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  //Serial.println("back");
}

void right() {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  //Serial.println("right");
}

void left() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  //Serial.println("left");
}

void stopp() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  delay(50);
}

void fine_delay() {
  delay(150);
  stopp();
  detection();

}

//IR sensor detection function
void detection()        //Measuring three angles(0.90.179) *********************************************************************
{
  qtr.read(sensorValues);
  /* FOR DEBUG
    Serial.print("Front Left IR sensor: ");
    Serial.println(sensorValues[0]);
    Serial.print("Front Right IR sensor: ");
    Serial.println(sensorValues[1]);
    Serial.print("Back Right IR sensor: ");
    Serial.println(sensorValues[2]);
    Serial.print("Back Left IR sensor: ");
    Serial.println(sensorValues[3]);
  */
  if (sensorValues[0] < 150 || sensorValues[1] < 150) // front 2 sensors
  {
    back();
    delay(500);

  }
  else if (sensorValues[2] < 150 || sensorValues[3] < 150)// back 2 sensors
  {
    forward();
    delay(500);
  }

}

//ultrasonic left sensor function
int distance_left() {
  digitalWrite(trigl, LOW);
  delayMicroseconds(2);
  digitalWrite(trigl, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigl, LOW);
  float left_distance = pulseIn(echol, HIGH, TIMEOUT);
  left_distance = (left_distance * 0.034) / 2;
  //Serial.print("Left: ");
  //Serial.println(left_distance);
  return (int)left_distance;
}

//ultrasonic center sensor function
int distance_center() {
  digitalWrite(trigc, LOW);
  delayMicroseconds(2);
  digitalWrite(trigc, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigc, LOW);
  float center_distance = pulseIn(echoc, HIGH, TIMEOUT);
  center_distance = (center_distance * 0.034) / 2;
  //Serial.print("Center: ");
  //Serial.println(center_distance);
  return (int)center_distance;
}

//ultrasonic right sensor function
int distance_right() {
  digitalWrite(trigr, LOW);
  delayMicroseconds(2);
  digitalWrite(trigr, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigr, LOW);
  float right_distance = pulseIn(echor, HIGH, TIMEOUT);
  right_distance = (right_distance * 0.034) / 2;
  //Serial.print("Right: ");
  //Serial.println(right_distance);
  return (int)right_distance;
}
void setup() {// ***********************************************************************************************************

  Serial.begin(115200);
  //initializing ultrasonic pins
  pinMode(echol, INPUT);
  pinMode(trigl, OUTPUT);
  pinMode(echoc, INPUT);
  pinMode(trigc, OUTPUT);
  pinMode(echor, INPUT);
  pinMode(trigr, OUTPUT);
  //initiallizing h-bridge pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LEDPIN, OUTPUT);

  pinMode(BUTTONPIN, INPUT);
  //stop();

  //IR pins
  pinMode(4, INPUT); // IR PIN SETUP
  pinMode(7, INPUT);
  pinMode(33, INPUT);
  pinMode(21, INPUT);
  // IR sensor setup
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {7, 4, 33, 21}, SensorCount);// start with top left IR sensor and go clockwise
  int countdown = 0;
  //button code
  while (!digitalRead(BUTTONPIN))
  {
    Serial.println(digitalRead(BUTTONPIN)); // is currently floating
  }
  while (countdown <= 5)
  {
    unsigned long currentTime = millis();
    if ((currentTime - previousTime) >= 1000)
    {
      previousTime = currentTime;
      countdown = countdown + 1;
      digitalWrite(LEDPIN, HIGH);
    }
    else if ((currentTime - previousTime) >= 500)
    {
      digitalWrite(LEDPIN, LOW);
    }
  }
  digitalWrite(LEDPIN, LOW);
  rotCnt = 0;
  motorStartTime = -200;
}

void loop() // **************************************************************************************************************
{
  currentMotorTime = millis();
  centerdistance = distance_center();
  rightdistance = distance_right();
  leftdistance = distance_left();
  detection();
  if (centerdistance <= 50)
  {
    if (!movingForward || (currentMotorTime - motorStartTime) > forwardDelay)
    {
      movingLeft = false;
      movingRight = false;
      movingForward = true;
      forward();
      motorStartTime = millis();
    }
    rotCnt = 0;
    //Serial.println("Count reset");
  }
  else if (leftdistance <= 50)
  {
    if (!movingLeft || (currentMotorTime - motorStartTime) > turnDelay)
    {
      movingLeft = true;
      movingRight = false;
      movingForward = false;
      left();
      motorStartTime = millis();
    }
  }
  else if (rightdistance <= 50)
  {
    if (!movingRight || (currentMotorTime - motorStartTime) > turnDelay)
    {
      movingLeft = false;
      movingRight = true;
      movingForward = false;
      right();
      motorStartTime = millis();
    }
  }
  else if (rotCnt >= 13)
  {
    if (!movingForward || (currentMotorTime - motorStartTime) > forwardDelay)
    {
      movingForward = true;
      forward();
      motorStartTime = millis();
    }
    rotCnt = 0;
    dir = dir ^ 1;
    //Serial.println("dir has changed");
  }
  else
  {
    if (dir)
    {
      if (!movingLeft || (currentMotorTime - motorStartTime) > turnDelay) //move left
      {
        movingLeft = true;
        movingRight = false;
        movingForward = false;
        left();
        motorStartTime = millis();
      }
    }
    else
    {
      if (!movingRight || (currentMotorTime - motorStartTime) > turnDelay)//move right
      {
        movingLeft = false;
        movingRight = true;
        movingForward = false;
        right();
        motorStartTime = millis();
      }
    }
    rotCnt = rotCnt + 1;
  }
  detection();
}
