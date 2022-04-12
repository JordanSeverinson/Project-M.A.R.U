//FDR Traversal Code 
#include <QTRSensors.h>
const int drive_distance = 2000;   // mm
const int motor_power = 180;      // 0-255
const int start_speed = 130;
const int motor_offset = 4;       // Diff. when driving straight
const int wheel_d = 68;           // Wheel diameter (mm)
const float wheel_c = PI * wheel_d; // Wheel circumference (mm)
const int counts_per_rev = 40;   // (4 pairs N-S) * (48:1 gearbox) * (2 falling/rising edges) = 384
const int backDistance = 200;
const int backMotorPower = 120;
const int turningDistance =136;
const int turnDelay = 100;
// Pins
const int enc_l_pin = 36;          // Motor A
const int enc_r_pin = 35;          // Motor B
const int pwma_pin = 1;
const int ain1_pin = 42; //2
const int ain2_pin = 2;  //42
const int pwmb_pin = 39;
const int bin1_pin = 40; //41
const int bin2_pin = 41; //40

//ultrasonic sensor pins

int echol = 46;
int trigl = 3;
int echor = 6;
int trigr = 5;
#define LEDPIN 12
#define BUTTONPIN 11
int rightdistance = 0, leftdistance = 0, centerdistance = 0;
int US_THRESHOLD = 50; // ********************************************THRESHHOLD!!!!!!!!!
int TIMEOUT = 7000; // ultrasonic timeout

// Globals
volatile unsigned long enc_l = 0;
volatile unsigned long enc_r = 0;
int previousTime;
// IR sensor setup
QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];
volatile bool hasDetected;

void detection()        
{
  qtr.read(sensorValues);
  /*
    Serial.print("Front Left IR sensor: ");
    Serial.println(sensorValues[0]);
    Serial.print("Front Right IR sensor: ");
    Serial.println(sensorValues[1]);
    Serial.print("Back Right IR sensor: ");
    Serial.println(sensorValues[2]);
    Serial.print("Back Left IR sensor: ");
    Serial.println(sensorValues[3]);
    delay(1000);*/
  if (sensorValues[0] > 400 && sensorValues[1] > 400) // front 2 sensors
  {
    brake();
    driveBack(backDistance, backMotorPower);
    brake();
    turn(turningDistance, motor_power);
    brake();
    driveStraightWithoutIR(drive_distance, motor_power);
    
  }
  else if (sensorValues[2] > 100 && sensorValues[3] > 100)// back 2 sensors
  {
    brake();
    driveBack(backDistance, backMotorPower);
    turn(turningDistance, motor_power);
    driveStraightWithoutIR(drive_distance, motor_power);
  }
}
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
void setup() {

  // Debug
  Serial.begin(115200);

  // Set up pins
  pinMode(enc_l_pin, INPUT_PULLUP);
  pinMode(enc_r_pin, INPUT_PULLUP);
  pinMode(pwma_pin, OUTPUT);
  pinMode(ain1_pin, OUTPUT);
  pinMode(ain2_pin, OUTPUT);
  pinMode(pwmb_pin, OUTPUT);
  pinMode(bin1_pin, OUTPUT);
  pinMode(bin2_pin, OUTPUT);
  //********************* ultra sonic sensor pins
  pinMode(echol, INPUT);
  pinMode(trigl, OUTPUT);
  pinMode(echor, INPUT);
  pinMode(trigr, OUTPUT);
  pinMode(BUTTONPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);
  //stop();

  //IR pins
  pinMode(7, INPUT); // IR PIN SETUP
  pinMode(4, INPUT);
  pinMode(33, INPUT);
  pinMode(21, INPUT);
  // IR sensor setup
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {7, 4,33, 21}, 4);// start with top left IR sensor and go clockwise

  // button code
  int countdown = 0;
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
  // go after button press
  // Drive straight
  // Set up interrupts
  attachInterrupt(digitalPinToInterrupt(enc_l_pin), countLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_r_pin), countRight, CHANGE);
  //delay(1000);
  // start moving forward
  /*
  digitalWrite(ain1_pin, HIGH);
  digitalWrite(ain2_pin, LOW);
  digitalWrite(bin1_pin, HIGH);
  digitalWrite(bin2_pin, LOW);
  analogWrite(pwma_pin, motor_power);
  analogWrite(pwmb_pin, motor_power);
  delay(100);*/
  driveStraightWithIR(9000, motor_power);
  //driveStraightA(drive_distance, motor_power);
}

void loop() {
  // do nothing
}
void driveBack(float dist, int power) {

  unsigned long num_ticks_l;
  unsigned long num_ticks_r;

  // Set initial motor power
  int power_l = power;
  int power_r = power;

  // Used to determine which way to turn to adjust
  unsigned long diff_l;
  unsigned long diff_r;

  // Reset encoder counts
  enc_l = 0;
  enc_r = 0;

  // Remember previous encoder counts
  unsigned long enc_l_prev = enc_l;
  unsigned long enc_r_prev = enc_r;

  // Calculate target number of ticks
  float num_rev = (dist) / wheel_c;  // gets number of revs
  unsigned long target_count = num_rev * counts_per_rev;

  // Debug
  /*
  Serial.print("Driving for ");
  Serial.print(dist);
  Serial.print(" cm (");
  Serial.print(target_count);
  Serial.print(" ticks) at ");
  Serial.print(power);
  Serial.println(" motor power");
*/
  // Drive until one of the encoders reaches desired count
  while ( (enc_l < target_count) && (enc_r < target_count) ) {
    
    // Sample number of encoder ticks
    num_ticks_l = enc_l;
    num_ticks_r = enc_r;

    // Print out current number of ticks
    /*
    Serial.print(num_ticks_l);
    Serial.print("\t");
    Serial.println(num_ticks_r);
*/
    // Drive
    driveBackward(power_l, power_r);

    // Number of ticks counted since last time
    diff_l = num_ticks_l - enc_l_prev; // DIFFERENCE BETWEEN current value and previous
    diff_r = num_ticks_r - enc_r_prev;

    // Store current tick counter for next time
    enc_l_prev = num_ticks_l;
    enc_r_prev = num_ticks_r;

    // If left is faster, slow it down and speed up right
    if ( diff_l > diff_r ) {
      power_l -= motor_offset;
      power_r += motor_offset;
    }

    // If right is faster, slow it down and speed up left
    if ( diff_l < diff_r ) {
      power_l += motor_offset;
      power_r -= motor_offset;
    }

    // Brief pause to let motors respond
    delay(20);
  }

  // Brake
  brake();
}
void driveStraightWithIR(float dist, int power) {

  unsigned long num_ticks_l;
  unsigned long num_ticks_r;

  // Set initial motor power
  int power_l = power;
  int power_r = power;
  int leftDistance = 0; 
  int rightDistance = 0; 
  // Used to determine which way to turn to adjust
  unsigned long diff_l;
  unsigned long diff_r;

  // Reset encoder counts
  enc_l = 0;
  enc_r = 0;

  // Remember previous encoder counts
  unsigned long enc_l_prev = enc_l;
  unsigned long enc_r_prev = enc_r;

  // Calculate target number of ticks
  float num_rev = (dist) / wheel_c;  // gets number of revs
  unsigned long target_count = num_rev * counts_per_rev;
int currentTurnTime;
int turnStartTime;
  // Debug
  /*
  Serial.print("Driving for ");
  Serial.print(dist);
  Serial.print(" cm (");
  Serial.print(target_count);
  Serial.print(" ticks) at ");
  Serial.print(power);
  Serial.println(" motor power");
*/
turnStartTime = 0;
  // Drive until one of the encoders reaches desired count
  while ( (enc_l < target_count) && (enc_r < target_count) ) {
    detection(); // detection for IR sensors
    leftDistance = distance_left();
    rightDistance = distance_right();
    // Sample number of encoder ticks
    currentTurnTime = millis();
    if(leftDistance <= US_THRESHOLD && leftDistance != 0 && (currentTurnTime - turnStartTime) > turnDelay)
    {
      enc_l = enc_l + motor_offset;
      turnStartTime = millis();
    }
    else if(rightDistance <= US_THRESHOLD && rightDistance != 0 && (currentTurnTime - turnStartTime) > turnDelay)
    {
      enc_r = enc_r + motor_offset;
      turnStartTime = millis();
    }
    num_ticks_l = enc_l;
    num_ticks_r = enc_r;

    // Print out current number of ticks
    /*
    Serial.print(num_ticks_l);
    Serial.print("\t");
    Serial.println(num_ticks_r);
*/
    // Drive
    driveForward(power_l, power_r);

    // Number of ticks counted since last time
    diff_l = num_ticks_l - enc_l_prev; // DIFFERENCE BETWEEN current value and previous
    diff_r = num_ticks_r - enc_r_prev;

    // Store current tick counter for next time
    enc_l_prev = num_ticks_l;
    enc_r_prev = num_ticks_r;

    // If left is faster, slow it down and speed up right
    if ( diff_l > diff_r ) {
      power_l -= motor_offset;
      power_r += motor_offset;
    }

    // If right is faster, slow it down and speed up left
    if ( diff_l < diff_r ) {
      power_l += motor_offset;
      power_r -= motor_offset;
    }

    // Brief pause to let motors respond
    delay(20);
  }

  // Brake
  brake();
}
void driveStraightWithoutIR(float dist, int power) {

  unsigned long num_ticks_l;
  unsigned long num_ticks_r;

  // Set initial motor power
  int power_l = power;
  int power_r = power;

  // Used to determine which way to turn to adjust
  unsigned long diff_l;
  unsigned long diff_r;

  // Reset encoder counts
  enc_l = 0;
  enc_r = 0;

  // Remember previous encoder counts
  unsigned long enc_l_prev = enc_l;
  unsigned long enc_r_prev = enc_r;

  // Calculate target number of ticks
  float num_rev = dist / wheel_c;  // Convert to mm
  unsigned long target_count = num_rev * counts_per_rev;

  // Debug
  /*
  Serial.print("Driving for ");
  Serial.print(dist);
  Serial.print(" cm (");
  Serial.print(target_count);
  Serial.print(" ticks) at ");
  Serial.print(power);
  Serial.println(" motor power");
*/
  // Drive until one of the encoders reaches desired count
  while ( (enc_l < target_count) && (enc_r < target_count) ) {
    //detection();
    // Sample number of encoder ticks
    num_ticks_l = enc_l;
    num_ticks_r = enc_r;

    // Print out current number of ticks
    /*
    Serial.print(num_ticks_l);
    Serial.print("\t");
    Serial.println(num_ticks_r);
*/
    // Drive
    driveForward(power_l, power_r);

    // Number of ticks counted since last time
    diff_l = num_ticks_l - enc_l_prev; // DIFFERENCE BETWEEN current value and previous
    diff_r = num_ticks_r - enc_r_prev;

    // Store current tick counter for next time
    enc_l_prev = num_ticks_l;
    enc_r_prev = num_ticks_r;

    // If left is faster, slow it down and speed up right
    if ( diff_l > diff_r ) {
      power_l -= motor_offset;
      power_r += motor_offset;
    }

    // If right is faster, slow it down and speed up left
    if ( diff_l < diff_r ) {
      power_l += motor_offset;
      power_r -= motor_offset;
    }

    // Brief pause to let motors respond
    delay(20);
  }

  // Brake
  brake();
}

void turn(float dist, int power) {

  unsigned long num_ticks_l;
  unsigned long num_ticks_r;

  // Set initial motor power
  int power_l = motor_power;
  int power_r = motor_power;

  // Used to determine which way to turn to adjust
  unsigned long diff_l;
  unsigned long diff_r;

  // Reset encoder counts
  enc_l = 0;
  enc_r = 0;

  // Remember previous encoder counts
  unsigned long enc_l_prev = enc_l;
  unsigned long enc_r_prev = enc_r;

  // Calculate target number of ticks
  float num_rev = dist / wheel_c;  // Convert to mm
  unsigned long target_count = num_rev * counts_per_rev;

  // Debug
  /*
  Serial.print("Driving for ");
  Serial.print(dist);
  Serial.print(" cm (");
  Serial.print(target_count);
  Serial.print(" ticks) at ");
  Serial.print(power);
  Serial.println(" motor power");
*/
  // Drive until one of the encoders reaches desired count
  while ( (enc_l < target_count) && (enc_r < target_count) ) {

    // Sample number of encoder ticks
    num_ticks_l = enc_l;
    num_ticks_r = enc_r;

    // Print out current number of ticks
    /*
    Serial.print(num_ticks_l);
    Serial.print("\t");
    Serial.println(num_ticks_r);
*/
    // Drive
    driveRight(power_l, power_r);

    // Number of ticks counted since last time
    diff_l = num_ticks_l - enc_l_prev; // DIFFERENCE BETWEEN current value and previous
    diff_r = num_ticks_r - enc_r_prev;

    // Store current tick counter for next time
    enc_l_prev = num_ticks_l;
    enc_r_prev = num_ticks_r;

    // If left is faster, slow it down and speed up right
    if ( diff_l > diff_r ) {
      power_l -= motor_offset;
      power_r += motor_offset;
    }

    // If right is faster, slow it down and speed up left
    if ( diff_l < diff_r ) {
      power_l += motor_offset;
      power_r -= motor_offset;
    }

    // Brief pause to let motors respond
    delay(20);
  }

  // Brake
  brake();
}
void driveForward(int power_a, int power_b) {

  // Constrain power to between -255 and 255
  power_a = constrain(power_a, 0, 220);
  power_b = constrain(power_b, 0, 220);

  // Left motor direction
  if ( power_a < 0 ) {
    digitalWrite(ain1_pin, LOW);
    digitalWrite(ain2_pin, HIGH);
  } else {
    digitalWrite(ain1_pin, HIGH);
    digitalWrite(ain2_pin, LOW);
  }

  // Right motor direction
  if ( power_b < 0 ) {
    digitalWrite(bin1_pin, LOW);
    digitalWrite(bin2_pin, HIGH);
  } else {
    digitalWrite(bin1_pin, HIGH);
    digitalWrite(bin2_pin, LOW);
  }

  // Set speed
  analogWrite(pwma_pin, abs(power_a));
  analogWrite(pwmb_pin, abs(power_b));
}
void driveRight(int power_a, int power_b) {

  // Constrain power to between -255 and 255
  power_a = constrain(power_a, -220, 220);
  power_b = constrain(power_b, -220, 220);

  // Left motor direction
  if ( power_a < 0 ) {
    digitalWrite(ain1_pin, LOW);
    digitalWrite(ain2_pin, HIGH);
  } else {
    digitalWrite(ain1_pin, HIGH);
    digitalWrite(ain2_pin, LOW);
  }

  // Right motor direction
  if ( power_b < 0 ) {
    digitalWrite(bin1_pin, HIGH);
    digitalWrite(bin2_pin, LOW);
  } else {
    digitalWrite(bin1_pin, LOW);
    digitalWrite(bin2_pin, HIGH);
  }

  // Set speed
  analogWrite(pwma_pin, abs(power_a));
  analogWrite(pwmb_pin, abs(power_b));
}
void driveBackward(int power_a, int power_b) {

  // Constrain power to between -255 and 255
  power_a = constrain(power_a, 0, 220);
  power_b = constrain(power_b, 0, 220);

  // Left motor direction
  if ( power_a < 0 ) {
    digitalWrite(ain1_pin, HIGH);
    digitalWrite(ain2_pin, LOW);
  } else {
    digitalWrite(ain1_pin, LOW);
    digitalWrite(ain2_pin, HIGH);
  }

  // Right motor direction
  if ( power_b < 0 ) {
    digitalWrite(bin1_pin, HIGH);
    digitalWrite(bin2_pin, LOW);
  } else {
    digitalWrite(bin1_pin, LOW);
    digitalWrite(bin2_pin, HIGH);
  }
  // Set speed
  analogWrite(pwma_pin, abs(power_a));
  analogWrite(pwmb_pin, abs(power_b));
}
void brake() {
  digitalWrite(ain1_pin, LOW);
  digitalWrite(ain2_pin, LOW);
  digitalWrite(bin1_pin, LOW);
  digitalWrite(bin2_pin, LOW);
  analogWrite(pwma_pin, LOW);
  analogWrite(pwmb_pin, LOW);
  delay(100);
}

void countLeft() {
  enc_l++;
}

void countRight() {
  enc_r++;
}
