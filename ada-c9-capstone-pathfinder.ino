
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
unsigned long ticksLeft = 0; 
unsigned long ticksRight = 0; 

// dc motors 
const int lb = A1; 
const int lf = A2; 
const int rb = A3;
const int rf = A4; 
const int leftSpeedPin = 5;
const int rightSpeedPin = 6;

// straight driving 
const int motorOffset = 1;  
const int ticksPerRev = 80; 
const float wheelDiameter = 65; // mm
const float wheelCircumference = wheelDiameter * PI; 

// exact degree turning 
const int distanceBetweenWheels = 15; // cm  
const int distanceFor90DegTurns = 0.25 * PI * distanceBetweenWheels;

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
  attachInterrupt(digitalPinToInterrupt(encPinRight), countTicksRight, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encPinLeft), countTicksLeft, CHANGE); 

}

void loop() {
  
}

/*
 * sensing / wall detection  
*/
 
void turnServo(int deg) {
  robotServo.attach(servoPin); 
  robotServo.write(deg);
  delay(500);
  robotServo.detach();
}

float detectDistance() {
  digitalWrite(outputPin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(outputPin, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(outputPin, LOW); 
  float dist_to_wall = pulseIn(inputPin, HIGH); 
  dist_to_wall = dist_to_wall/5.8/10; 
  return dist_to_wall; 
}

float detectForward() {
  turnServo(90); 
  float distance_forward = detectDistance(); 
  delay(20); 
  return distance_forward; 
}

float detectLeft() {
  turnServo(180); 
  float distance_left = detectDistance(); 
  delay(20); 
  return distance_left; 
}

float detectRight() {
  turnServo(0); 
  float distance_right = detectDistance();
  delay(20); 
  return distance_right; 
}

/*
 * navigation 
*/

// drive straight a given distance, with a specified 'speed' (power) and in a specified direction 
// function adapted from SparkFun Electronics Fred Bot tutorial 
void driveStraight(float dist, int power, char direction) { 
  unsigned long numTicksL;
  unsigned long numTicksR;

  // set initial motor power
  int powerL = power;
  int powerR = power;

  // used to determine which way to turn to adjust
  unsigned long diffL;
  unsigned long diffR;

  // reset encoder counts
  ticksLeft = 0;
  ticksRight = 0;

  // remember previous encoder counts
  unsigned long encLPrev = ticksLeft;
  unsigned long encRPrev = ticksRight;

  // calculate target number of ticks
  float targetWheelRev = (dist * 10) / wheelCircumference;  
  unsigned long targetTickCount = targetWheelRev * ticksPerRev;

  // drive until one of the encoders reaches desired count
  while ( (ticksLeft < targetTickCount) && (ticksRight < targetTickCount) ) { 
    
    // sample number of encoder ticks
    numTicksL = ticksLeft;
    numTicksR = ticksRight;
    
    // drive 
    if (direction == 'F') {
      moveLeftForward(powerL); 
      moveRightForward(powerR);
    } else if (direction == 'B') {
      moveLeftBackward(powerL); 
      moveRightBackward(powerR);
    } else if (direction == 'L') {
      turnLeft(powerL, powerR); 
    } else if (direction == 'R') {
      turnRight(powerL, powerR);
    }

    // number of ticks counted since last time
    diffL = numTicksL - encLPrev;
    diffR = numTicksR - encRPrev;

    // store current tick counter for next time
    encLPrev = numTicksL;
    encRPrev = numTicksR;

    // if left is faster, slow it down and speed up right
    if ( diffL > diffR ) {
      powerL -= motorOffset;
      powerR += motorOffset;
    }

    // if right is faster, slow it down and speed up left
    if ( diffL < diffR ) {
      powerL += motorOffset;
      powerR -= motorOffset;
    }

    // brief pause to let motors respond
    delay(20);
  }

  stopMotors(); 
} 

void countTicksLeft() {
  ticksLeft++;
}

void countTicksRight() {
  ticksRight++;
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

void smallTurnLeft(int powerLeft, int powerRight) { 
  turnLeft(powerLeft, powerRight); 
  delay(100); 
  stopMotors();
}

void smallTurnRight(int powerLeft, int powerRight) {
  turnRight(powerLeft, powerRight); 
  delay(100); 
  stopMotors();
}

void turn90DegLeft(int power) { 
  driveStraight(distanceFor90DegTurns, power, 'L');
}

void turn90DegRight(int power) {
  driveStraight(distanceFor90DegTurns, power, 'R'); 
}

void stopMotors() {
  digitalWrite(lb, LOW); 
  digitalWrite(lf, LOW); 
  digitalWrite(rb, LOW); 
  digitalWrite(rf, LOW); 
  analogWrite(leftSpeedPin, 0); 
  analogWrite(rightSpeedPin, 0);  
}


