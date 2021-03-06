<<<<<<< HEAD:maru_going_straight/maru_going_straight.ino
// Parameters
const int drive_distance = 1000 ;   // cm
const int motor_power = 120;      // 0-255
=======
const int drive_distance = 50;   // cm
const int motor_power = 200;      // 0-255
>>>>>>> 7109542c5becbb10363b8213339f6f9b5e731102:maru_going_straight.ino
const int motor_offset = 5;       // Diff. when driving straight
const int wheel_d = 67;           // Wheel diameter (mm)
const float wheel_c = PI * wheel_d; // Wheel circumference (mm)
const int counts_per_rev = 384;   // (4 pairs N-S) * (48:1 gearbox) * (2 falling/rising edges) = 384

// Pins
<<<<<<< HEAD:maru_going_straight/maru_going_straight.ino
const int enc_l_pin = 19;          // Motor A
const int enc_r_pin = 20;          // Motor B
=======
const int enc_l_pin = 35;          // Motor A
const int enc_r_pin = 36;          // Motor B
>>>>>>> 7109542c5becbb10363b8213339f6f9b5e731102:maru_going_straight.ino
const int pwma_pin = 1;
const int ain1_pin = 42;
const int ain2_pin = 2;
const int pwmb_pin = 39;
<<<<<<< HEAD:maru_going_straight/maru_going_straight.ino
const int bin1_pin = 41;
const int bin2_pin = 40;
=======
const int bin1_pin = 40;
const int bin2_pin = 41;
>>>>>>> 7109542c5becbb10363b8213339f6f9b5e731102:maru_going_straight.ino
const int stby_pin = 10;

// Globals
volatile unsigned long enc_l = 0;
volatile unsigned long enc_r = 0;

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
  pinMode(stby_pin, OUTPUT);

  // Set up interrupts
  attachInterrupt(digitalPinToInterrupt(enc_l_pin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(enc_r_pin), countRight, RISING);

  // Drive straight
  delay(1000);
  enableMotors(true);
}

void loop() {
  driveStraight(drive_distance, motor_power);
  // Do nothing
}

void driveStraight(float dist, int power) {

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
  float num_rev = (dist * 10) / wheel_c;  // Convert to mm
  unsigned long target_count = num_rev * counts_per_rev;
  
  // Debug
  Serial.print("Driving for ");
  Serial.print(dist);
  Serial.print(" cm (");
  Serial.print(target_count);
  Serial.print(" ticks) at ");
  Serial.print(power);
  Serial.println(" motor power");

  // Drive until one of the encoders reaches desired count
  while ( (enc_l < target_count) && (enc_r < target_count) ) {

    // Sample number of encoder ticks
    num_ticks_l = enc_l;
    num_ticks_r = enc_r;

    // Print out current number of ticks
    Serial.print(num_ticks_l);
    Serial.print("\t");
    Serial.println(num_ticks_r);

    // Drive
    drive(power_l, power_r);

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

void enableMotors(boolean en) {
  if ( en ) {
    digitalWrite(stby_pin, HIGH);
  } else {
    digitalWrite(stby_pin, LOW);
  }
}

void drive(int power_a, int power_b) {

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

void brake() {
  digitalWrite(ain1_pin, LOW);
  digitalWrite(ain2_pin, LOW);
  digitalWrite(bin1_pin, LOW);
  digitalWrite(bin2_pin, LOW);
  analogWrite(pwma_pin, 0);
  analogWrite(pwmb_pin, 0);
}

void countLeft() {
  enc_l++;
}

void countRight() {
  enc_r++;
}
