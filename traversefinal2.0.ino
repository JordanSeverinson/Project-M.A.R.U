#include <QTRSensors.h>// Library for IR sensors

//ultrasonic sensor pins might need to be changed
int echol = 13;
int trigl = 12;
int echoc = 5;
int trigc = 4;
int echor = 3;
int trigr = 2;

//H-bridge pins (motor control)
#define ENA 11
#define ENB 10
#define IN1 9 //purple
#define IN2 8 //light gray
#define IN3 7 //white
#define IN4 6//black 
#define LEDPIN A2
#define BUTTONPIN A3

//speed of robot maybe add dif. speeds
#define rightmotor 255
#define leftmotor 255
#define carspeed 140
#define ramspeed 255

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
  delay(200);
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
      
      if(sensorValues[1] > 1500 && sensorValues[2] > 1500)
      {
        stopp();
        right();
        delay(500);
        forward();
        delay(800);
        
      }
      else if(sensorValues[0] > 1500 && sensorValues[3] > 1500)
      {
        stopp();
        right();
        delay(500);
        forward();
        delay(800);
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
  left_distance = left_distance / 58;
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
  center_distance = center_distance / 58;
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
  right_distance = right_distance / 58;
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
  
}
void loop() {
 
 
  //checking center sensor first
 centerdistance = distance_center();
 leftdistance = distance_left();
 rightdistance = distance_right();
 detection();
 
 if(centerdistance <= 50){
  stopp();
  delay(500);
  detection();

  rightdistance = distance_right();
  leftdistance = distance_left();

      if(rightdistance < leftdistance){
          left();
          delay(100);//change the delay to see how much it turns
          detection();
        }
      else if(rightdistance > leftdistance){
          right();
          delay(100);
          detection();
        }
      else if((rightdistance <= 33) || (leftdistance <= 33)){
          back();
          delay(150);
          detection();
        }
  }
 

 //checking right sensor
 

 else if(rightdistance <= 50){
  detection();
  stopp();
  delay(500);

  rightdistance = distance_right();
  leftdistance = distance_left();
  centerdistance = distance_center();
  
        if(rightdistance < leftdistance){
          left();
          detection();
          delay(100);
          }
        else if(centerdistance < leftdistance){
          left();
          detection();
          delay(100);//change the delay to see how much it turns
          }
        else if((centerdistance <= 15) || (leftdistance <= 15)){
          back();
          detection();
          delay(300);
          }
 }
//checking left sensor
 

 else if(leftdistance <= 50){
  stopp();
  detection();
  delay(500);

  leftdistance = distance_left();
  rightdistance = distance_right();
  centerdistance = distance_center();
  
        if(leftdistance < rightdistance){
          right();
          detection();
          delay(100);
          }
        else if(centerdistance < rightdistance){
          right();
          detection();
          delay(100);//change the delay to see how much it turns
          }
        else if((centerdistance <= 15) || (rightdistance <= 15)){
          back();
          detection();
          delay(300);
          }

 }

 else {
  forward();
  detection();
 }
}
