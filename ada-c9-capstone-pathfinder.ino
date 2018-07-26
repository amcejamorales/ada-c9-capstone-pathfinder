
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
    Serial.begin(9600);
    
  // set up ultrasonic sensor pins 
  pinMode(inputPin, INPUT); 
  pinMode(outputPin, OUTPUT); 

  // dc motors 
  pinMode(lb, OUTPUT); 
  pinMode(lf, OUTPUT); 
  pinMode(rb, OUTPUT); 
  pinMode(rf, OUTPUT);
  pinMode(leftSpeedPin, OUTPUT);
  pinMode(rightSpeedPin, OUTPUT);

  // encoders 
  pinMode(encPinLeft, INPUT_PULLUP); 
  pinMode(encPinRight, INPUT_PULLUP); 

}

void loop() {
  
}

void moveLeftForward(int power) {
  power = constrain(power, -255, 255);
  digitalWrite(lb, HIGH);
  digitalWrite(lf, LOW);
  analogWrite(leftSpeedPin, abs(power));
}

void moveRightForward(int power) {
  power = constrain(power, -255, 255);
  digitalWrite(rb, HIGH);
  digitalWrite(rf, LOW);
  analogWrite(rightSpeedPin, abs(power));
}

void moveLeftBackward(int power) {
  power = constrain(power, -255, 255);
  digitalWrite(lb, LOW);
  digitalWrite(lf, HIGH);
  analogWrite(leftSpeedPin, abs(power));
} 

void moveRightBackward(int power) {
  power = constrain(power, -255, 255);
  digitalWrite(rb, LOW);
  digitalWrite(rf, HIGH);
  analogWrite(rightSpeedPin, abs(power));
} 

void turnLeft(int powerLeft, int powerRight) { 
  moveLeftBackward(powerLeft);
  moveRightForward(powerRight); 
}

void turnRight(int powerLeft, int powerRight) {
  moveLeftForward(powerLeft); 
  moveRightBackward(powerRight);
} 
