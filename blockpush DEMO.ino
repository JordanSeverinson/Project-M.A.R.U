#include <QTRSensors.h>// Library for IR sensors
#include <Wire.h> // i2c communication
#include <VL53L1X.h> // POLOLU tof sensor library

// DO NOT USE PINS 15,16, 8, 3
// 15 and 16 are used for flash
//pin 4 is input ONLY

//ultrasonic sensor pins
int echol = 35;
int trigl = 2;
int echoc = 36;
int trigc = 25;
int echor = 39;
int trigr = 26;
//Time of flight varibles
VL53L1X tof_sensor; //time of flight sensor 
int tof_reading; 

//H-bridge pins (motor control)
#define ENA 33
#define ENB 15
#define IN1 18 //purple
#define IN2 19 //light gray
#define IN3 21 //white
#define IN4 22//black 
#define LEDPIN 32
#define BUTTONPIN 34

// Pins for Tslots:
// 6 Brown LED and 7 Black Transistor output
#define scl 14
#define sda 17
//speed of robot maybe add dif. speeds
#define carspeed 180
#define ramspeed 220

//initial distances on sensors are zero
int rightdistance = 0, leftdistance = 0, centerdistance = 0;
int tof_centerdistance;
int US_THRESHOLD = 20; // ********************************************THRESHHOLD!!!!!!!!!
int rotCnt;// rotation counter go forward after 14
bool dir = 0;
int TIMEOUT = 7000; // ultrasonic timeout

// IR sensor setup
QTRSensors qtr;
const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];
volatile int customSpeed;
//IR Sensor pins ********************************************
//int LeftFrontIRPin = 7;
//int RightFrontIRPin = 4;
//int LeftBackIRPin = 21;
//int RightBackIRPin =33;

// non blocking nav delay
long int motorStartTime;
long int currentMotorTime;
long int turnStart;
long int turnCurrent;
long int stopTime;
long int currentStop;
bool movingForward, movingRight, movingLeft,isStopped = false;
bool isDetecting;
// ************************************************************delay knobs to be adjusted
int forwardDelay = 100; // delay for motor control
int turnDelay = 20;
int stopDelay = 80;
int tofThreshold = 60;
//direction functions
int previousTime; // pushbutton timer
volatile bool hasTurned;
//***************************************************************************************Motor Functions************************************************
void forward(int cspeed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(0,cspeed);//analogWrite(ENA, cspeed);
  ledcWrite(1,cspeed);//analogWrite(ENB, cspeed);
  // Serial.println("forward");
}
void ram() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  //analogWrite(ENA, ramspeed);
  //analogWrite(ENB, ramspeed);
  ledcWrite(0,carspeed);//analogWrite(ENA, cspeed);
  ledcWrite(1,carspeed);//analogWrite(ENB, cspeed);
  // Serial.println("forward");
}
void back() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  //analogWrite(ENA, carspeed);
  //analogWrite(ENB, carspeed);
  ledcWrite(0,carspeed);//analogWrite(ENA, cspeed);
  ledcWrite(1,carspeed);//analogWrite(ENB, cspeed);
  //Serial.println("back");
}

void right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  //analogWrite(ENA, carspeed);
  //analogWrite(ENB, carspeed);
  ledcWrite(0,carspeed);//analogWrite(ENA, cspeed);
  ledcWrite(1,carspeed);//analogWrite(ENB, cspeed);
  //Serial.println("right");
}

void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  //analogWrite(ENA, carspeed);
  //analogWrite(ENB, carspeed);
  ledcWrite(0,carspeed);//analogWrite(ENA, cspeed);
  ledcWrite(1,carspeed);//analogWrite(ENB, cspeed);
  //Serial.println("left");
}

void stopp() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(0,0);//analogWrite(ENA, cspeed);
  ledcWrite(1,0);//analogWrite(ENB, cspeed);
  //digitalWrite(ENA, LOW);
  //digitalWrite(ENB, LOW);
}
void motorBuffer()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(0,0);//analogWrite(ENA, cspeed);
  ledcWrite(1,0);//analogWrite(ENB, cspeed);
  //digitalWrite(ENA, LOW);
  //digitalWrite(ENB, LOW);
  delay(10);
}
/*
void fine_delay() {
  delay(150);
  stopp();

}
*/
//***********************************************************************************IR sensor detection function**************************************************
void detection()        
{
  qtr.read(sensorValues);
  
    Serial.print("Front Left IR sensor: ");
    Serial.println(sensorValues[0]);
    Serial.print("Front Right IR sensor: ");
    Serial.println(sensorValues[1]);/*
    Serial.print("Back Right IR sensor: ");
    Serial.println(sensorValues[2]);
    Serial.print("Back Left IR sensor: ");
    Serial.println(sensorValues[3]);
    delay(500);*/
    
  if (sensorValues[1] > 200) // front 2 sensors
  {
    hasTurned = true;
    motorBuffer();
    right();
    delay(250);
    motorBuffer();
    forward(ramspeed);
    delay(250);
    left();
    delay(100);
  }/*
  else if (sensorValues[2] > 200 && sensorValues[3] > 200)// back 2 sensors
  {
    motorBuffer();
    right();
    delay(200);
    motorBuffer();
    forward(ramspeed);
    delay(200);
  }*/
}
//***********************************************************************************Object Detection Sensor Fucntions********************************************************************************
//Time of Flight sensor Function
int tof_detect(){
  tof_reading = tof_sensor.readRangeSingleMillimeters() / 10; //in cm
  return (int) tof_reading;
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
  Serial.print("Left: ");
  Serial.println(left_distance);
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
  Serial.print("Center: ");
  Serial.println(center_distance);
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
  Serial.print("Right: ");
  Serial.println(right_distance);
  return (int)right_distance;
}

void setup() {// ***********************************************************************************************************

  Serial.begin(115200);
  
  //initializing time of flight sensor
  Wire.begin(14, 27);// might need to set actual pins used
  tof_sensor.init();
  //  tof_sensor.setTimeout(60); // tof sensor timeout
  
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
  /*
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);*/
  ledcAttachPin(ENA, 0);
  ledcAttachPin(ENB, 1);
  ledcSetup(0, 1000, 8);// setup pwm channels
  ledcSetup(1, 1000, 8);
  pinMode(LEDPIN, OUTPUT);

  pinMode(BUTTONPIN, INPUT);
  //stop();

  //IR pins
  pinMode(4, INPUT); // IR PIN SETUP
  pinMode(7, INPUT);
  //pinMode(33, INPUT);
  //pinMode(21, INPUT);
  // IR sensor setup
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {7, 4}, 2);// start with top left IR sensor and go clockwise
  int countdown = 0;
  //*********************************************************** Start button *********************************************************
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
  //********************************************************** Start button **********************************************************
  rotCnt = 0;
  motorStartTime = -200;
  stopTime = -200;
  turnStart = -200;
  isStopped = true;
  movingForward = false;
  movingRight = false;
  customSpeed = carspeed;
  hasTurned = false;
}
void loop()
{
  detection();
  rightdistance = distance_right();
  leftdistance = distance_left();
  currentMotorTime = millis();
  turnCurrent = millis();
  if (!movingForward || (currentMotorTime - motorStartTime) > forwardDelay)
    {
      movingLeft = false;
      movingRight = false;
      movingForward = true;
      isStopped = false;
      forward(carspeed);
      motorStartTime = millis();
    }
    else if ((rightdistance <= US_THRESHOLD && rightdistance !=0) && !hasTurned)
  {
    if (!movingRight || (turnCurrent - turnStart) > turnDelay)
    {
      motorBuffer();
      movingLeft = false;
      movingRight = true;
      movingForward = false;
      isStopped = false;
      //motorBuffer(); // added stop
      left();
      turnStart = millis();
    }
   }
   else if((rightdistance > 40 && rightdistance !=0 ) && !hasTurned)
  {
    if (!movingRight || (turnCurrent - turnStart) > turnDelay)
    {
      motorBuffer();
      movingLeft = true;
      movingRight = true;
      movingForward = false;
      isStopped = false;
      //stopp(); // added stop
      right();
      turnStart = millis();
    }
  }
    else if (leftdistance <= US_THRESHOLD && leftdistance !=0)
  {
    if (!movingRight || (turnCurrent - turnStart) > turnDelay)
    {
      motorBuffer();
      movingLeft = false;
      movingRight = true;
      movingForward = false;
      isStopped = false;
      //motorBuffer(); // added stop
      right();
      turnStart = millis();
    }
   }
   else if(leftdistance > 40 && leftdistance !=0 )
  {
    if (!movingRight || (turnCurrent - turnStart) > turnDelay)
    {
      motorBuffer();
      movingLeft = true;
      movingRight = true;
      movingForward = false;
      isStopped = false;
      //stopp(); // added stop
      left();
      turnStart = millis();
    }
    }
}
