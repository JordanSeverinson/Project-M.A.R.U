#include <QTRSensors.h>// Library for IR sensors

//ultrasonic sensor pins might need to be changed
int echol = 8;
int trigl = 18;
int echoc = 17;
int trigc = 16;
int echor = 5;
int trigr = 4;

//H-bridge pins (motor control)
#define ENA 1
#define ENB 39
#define IN1 2 //purple
#define IN2 42 //light gray
#define IN3 41 //white
#define IN4 40//black 
#define LEDPIN A2
#define BUTTONPIN A3
// Pins for Tslots:
// 
//speed of robot maybe add dif. speeds
#define rightmotor 255
#define leftmotor 255
#define carspeed 150
#define ramspeed 180

//initial distances on sensors are zero
int rightdistance = 0, leftdistance = 0, centerdistance = 0;
int rotCnt;// rotation counter go forward after 14
bool dir = 0;
unsigned long previousTime = 0;
//LEDPIN = A2;
//BUTTONPIN = A3;
// IR sensors
QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

//direction functions
void forward(){
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
  analogWrite(ENA, ramspeed);
  analogWrite(ENB, ramspeed); 
  delay(100);
 // Serial.println("forward"); 
}

void back(){
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  delay(250);
  //Serial.println("back");  
}

void right(){
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  delay(150);
  //Serial.println("right"); 
}

void left(){
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, carspeed);
  analogWrite(ENB, carspeed);
  delay(150);
  //Serial.println("left");  
}

void stopp(){
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  delay(50);
}

void fine_delay(){
  delay(150);
  stopp();
  detection();
  
}

//IR sensor detection function
void detection()        //Measuring three angles(0.90.179)
    {      
      qtr.read(sensorValues);
      /*
      Serial.print("Front Left IR sensor: ");
      Serial.println(sensorValues[1]);
      Serial.print("Front Right IR sensor: ");
      Serial.println(sensorValues[2]);
      Serial.print("Back Left IR sensor: ");
      Serial.println(sensorValues[0]);
      Serial.print("Back Right IR sensor: ");
      Serial.println(sensorValues[3]);
      */
      if(sensorValues[1] < 1500 || sensorValues[2] < 1500)
      {
        stopp();
        back();
        rotCnt = 0;
        delay(500);
        
      }
      else if(sensorValues[0] < 1500 || sensorValues[3] < 1500)
      {
        stopp();
        forward(); 
        rotCnt = 0;
        delay(500);
      }
      
    }
    
//ultrasonic left sensor function
int distance_left(){
  digitalWrite(trigl, LOW);
  delayMicroseconds(2);
  digitalWrite(trigl, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigl, LOW);
  float left_distance = pulseIn(echol, HIGH);
  left_distance = (left_distance * 0.034) / 2;
  Serial.print("Left: "); 
  Serial.println(left_distance);
  return(int)left_distance;
}

//ultrasonic center sensor function
int distance_center(){
  digitalWrite(trigc, LOW);
  delayMicroseconds(2);
  digitalWrite(trigc, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigc, LOW);
  float center_distance = pulseIn(echoc, HIGH);
  center_distance = (center_distance * 0.034) / 2;
  Serial.print("Center: "); 
  Serial.println(center_distance);
  return(int)center_distance;
}

//ultrasonic right sensor function
int distance_right(){
  digitalWrite(trigr, LOW);
  delayMicroseconds(2);
  digitalWrite(trigr, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigr, LOW);
  float right_distance = pulseIn(echor, HIGH);
  right_distance = (right_distance * 0.034) / 2;
  Serial.print("Right: "); 
  Serial.println(right_distance);
  return(int)right_distance;
}

void setup() {
  
  Serial.begin(112500);
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
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A4,INPUT);
  pinMode(A5,INPUT);
  // IR sensor setup
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A4,A5}, SensorCount);
  qtr.setEmitterPin(HIGH);
  digitalWrite(LEDPIN, LOW);
  int countdown = 0;
  while(!digitalRead(BUTTONPIN))
  {
    Serial.println(digitalRead(BUTTONPIN)); // is currently floating
  }
  while(countdown <= 5)
  {
    unsigned long currentTime = millis();
    if((currentTime - previousTime) >= 1000)
    {
      previousTime = currentTime;
      countdown = countdown +1;
      digitalWrite(LEDPIN, HIGH);
    }
    else if((currentTime - previousTime) >= 500)
    {
      digitalWrite(LEDPIN, LOW);
    }
  }
  digitalWrite(LEDPIN,LOW);
  rotCnt = 0;
}

void loop() {
  
  centerdistance = distance_center();
  rightdistance = distance_right();
  leftdistance = distance_left();
  detection();
  
if(centerdistance <= 50){
  forward();
  detection();
  rotCnt = 0;
  //Serial.println("Count reset");

  }
else if (leftdistance <=50){
  left();
  detection();
  stopp();
  //rotCnt = 0; 
  }
else if (rightdistance <=50){
  right(); 
  detection();
  stopp();
  //rotCnt = 0;
  }
 else if(rotCnt >= 13) 
 {
  stopp();
  forward();
  fine_delay();
  rotCnt = 0;
  dir = dir ^ 1;
  //Serial.println("dir has changed");
 }
else{
  if(dir)
  {
    left();
  }
  else
  {
  right();
  }
  //Serial.print("The count is incremeted to: ");
 // Serial.println(rotCnt);
  rotCnt = rotCnt + 1;
  
  detection(); 
  stopp(); 
}
 
}
