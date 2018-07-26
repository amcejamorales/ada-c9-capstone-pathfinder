
// data structures 
#include "vector.h"
#include "graph.h"

// servo 
#include <Servo.h>
Servo robotServo; 
int servoPin = 12; 

// ultrasonic sensor 
int inputPin = 11; // signal receiver pin 
int outputPin = 10; // signal emission pin 

// speed measuring module with photoelectric encoders 
const int encPinRight = 2; 
const int encPinLeft = 3; 
unsigned long ticskLeft = 0; 
unsigned long ticskRight = 0; 

// dc motors 
const int lb = A1; 
const int lf = A2; 
const int rb = A3;
const int rf = A4; 
const int leftSpeedPin = 5;
const int rightSpeedPin = 6;

void setup() {
  
}

void loop() {
  
}

