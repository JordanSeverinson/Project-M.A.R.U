//ultrasonic sensor pins might need to be changed
int echol = 13;
int trigl = 12;
int echoc = 5;
int trigc = 4;
int echor = 3;
int trigr = 2;

//initial distances on sensors are zero
int rightdistance = 0, leftdistance = 0, centerdistance = 0;
//LEDPIN = A2;
//BUTTONPIN = A3;
// IR sensors

//IR sensor detection function
int startLeft;
int currentLeft;
int startCenter;
int currentCenter;
int startRight;
int currentRight;
int finalTime;
//ultrasonic left sensor function
int distance_left(){
  digitalWrite(trigl, LOW);
  delayMicroseconds(2);
  digitalWrite(trigl, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigl, LOW);
  float left_distance = pulseIn(echol, HIGH);
  left_distance = left_distance / 58;
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
  currentLeft = 0;
  currentCenter = 0;
  currentRight = 0;
  delay(3000);
  
}
void loop() {
 
 //Center Timing ***********************************************
 startCenter = millis();
 currentCenter= millis();
 finalTime = currentCenter - startCenter;
 Serial.print("millis latency: ");
 Serial.println(finalTime);
 startCenter = millis();
 centerdistance = distance_center();
 currentCenter = millis();
 finalTime = currentCenter - startCenter;
 Serial.print("Center latency: ");
 Serial.println(finalTime);
 //left timing *************************************************
 startLeft = millis();
 leftdistance = distance_left();
 currentLeft = millis();
 finalTime = currentLeft - startLeft;
 Serial.print("Left latency: ");
 Serial.println(finalTime);
 currentLeft = millis();
 //right timing ************************************************
 startRight = millis();
 rightdistance = distance_right();
 currentRight = millis();
 finalTime = currentRight - startRight;
 Serial.print("Right latency: ");
 Serial.println(finalTime);
 Serial.println("******************");
 delay(5000);
}
