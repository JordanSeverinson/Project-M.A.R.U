


// TimerOne
#include "TimerOne.h"

//constants for Interrupt Pins
// change values if not using Arduino Uno

const byte MOTOR_1 = 2; // Motor 1 Interrupt Pin(INT 0)
//const byte MOTOR_2 = 3; // Motor 2 Interrupt Pin(INT1)

// Intergers fpr pulse counters
unsigned int counter_1 = 0;
//unsigned int counter_2 = 0; 

// Float for number of slots in encoder disk
float diskslots = 20.00; // number of slots in each encoder

// Interrupt Service Routines
unsigned int counter1 = 0;
//unsigned int counter2 = 0;

// Motor 1 pulse count ISR
void ISR_count1()
{
  counter1++; // increment Motor_1 value
}

// Motor 2 pulse count ISR

//void ISR_count2()
/*{
  counter2++; // increment motor_1 value
} */

//timerOne ISR
void ISR_timerone()
{
  Timer1.detachInterrupt(); // Stop the timer 
  Serial.print("Motor Speed 1: ");
  float rotation1= (counter1/ diskslots) *60.00; //Calculate RPM Motor_1
  Serial.print(rotation1);
  Serial.println(" RPM: " );
  counter1 = 0; // reset counter to zero
  /*Serial.print("Motor Speed 2:");
  float rotation2 = (counter2/ diskslots) * 60.00; // Calculate RPM motor_2
  Serial.print(rotation2);
  Serial.println(" RPM " );
  counter2 = 0; // reset counter to zero */
  Timer1.attachInterrupt( ISR_timerone); // Enable timer 
}

void setup() {
  Serial.begin(112500);
  
  Timer1.initialize(1000000); // Timer set for 1 second
  attachInterrupt(digitalPinToInterrupt (MOTOR_1), ISR_count1, RISING); //Increase counter 1when speed sensor pin goes high
  //attachInterrupt(digitalPinToInterrupt (MOTOR_2), ISR_count2, RISING); // Increase counter 2 when speed sensor pin goes high
  Timer1.attachInterrupt( ISR_timerone); // Enable timer
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
