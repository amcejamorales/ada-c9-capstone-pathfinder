
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

// speed regulation 
const int speed = 180; 
const int turnSpeed = 150; 
const int courseCorrectSpeed = 130; 

// straight driving 
const int motorOffset = 1;  
const int ticksPerRev = 80; 
const float wheelDiameter = 65; // mm
const float wheelCircumference = wheelDiameter * PI; 

// exact degree turning 
const int distanceBetweenWheels = 15; // cm  
const int distanceFor90DegTurns = 0.25 * PI * distanceBetweenWheels;

// wall detection, course correction, and exit conditions
const int wallThreshold = 39; // cm 
int maxDistToWall = 16; // cm 
int minDistToWall = 14; // cm 
int turnMaxDistToWall = 14; // cm 
int turnMinDistToWall = 12; // cm 
int consecutiveLeftOpenings; 
int consecutiveRightOpenings; 
int openFieldCount = 0; 
const int maxOpenFieldCount = 1; 
float stoppingDistance = 13; // cm 

// navigation 
float distLeft; 
float distRight;
float travelDistance = 6.5; 
float distanceTravelled = 0.0;
char currentRelativeDirection = 'F'; 

// graph setup 
Graph newGraph; 

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
 * Graph Functions 
*/

// create nodes and arcs (if appropriate) at important points in the maze 
void createNodeAndSearchThroughGraph(float dist) { 
  Vector<float> distancesToWalls; 
  scanEnvironment(&distancesToWalls);

  Vector<char> openings; 
  if (distancesToWalls[0] > wallThreshold) { openings.add('F'); }
  if (distancesToWalls[1] > wallThreshold) { openings.add('L'); }
  if (distancesToWalls[2] > wallThreshold) { openings.add('R'); }

  // create node 
  addNode(newGraph, newGraph.nodes.size() + 1, distancesToWalls, openings, distanceTravelled); 

  // create arc from previous node to current node unless there is no previous node 
  if (newGraph.nodes.size() > 1) {
    Arc *lastArc = newGraph.arcs[newGraph.arcs.size() - 1]; 
    
    Node *previousNode; 
    if (newGraph.arcs.size() == 0) {
      previousNode = newGraph.nodes[newGraph.nodes.size() - 2]; 
    } else {
      previousNode = lastArc->finish;  
    } 
    
    Node *currentNode = newGraph.nodes[newGraph.nodes.size() - 1]; 
    
    addArc(newGraph, previousNode, currentNode, currentRelativeDirection, dist);
  } 
    
  if (openFieldCount >= maxOpenFieldCount) {
    Serial.println(F("You've exited the maze!")); 
  } else {  
    // deflate consecutive left and right 'openings' after a node is made 
    // to not confuse an opening left or right of the node with the occurrence of a new node 
    consecutiveLeftOpenings = -2; 
    consecutiveRightOpenings = -2;  

    dfs(newGraph);
  }
} 

// search through graph to determine whether to keep going or whether exit is reached
void dfs(Graph & g) {
  Node *mostRecentNode = g.nodes[g.nodes.size() - 1];
  Vector<float> distancesToWalls = mostRecentNode->distancesToWalls; 
  Vector<char> openings = mostRecentNode->openings;
    
  if (openings.size() == 3) {
      Serial.println(F("Openings Forwards, Left, and Right."));
  } else if (openings.size() == 0) {
    Serial.println(F("----------------------------"));
    Serial.println(F("HIT A DEAD END"));
    Serial.println(F("----------------------------"));

    // backtrack to the first previous node in the graph that has an alternative opening 
    Node *secondMostRecentNode = g.nodes[g.nodes.size() - 2]; 
    findPreviousOpening(g, secondMostRecentNode, mostRecentNode);
  } else { 
    float dist; 
    if (openings[0] == 'F') {
      currentRelativeDirection = 'F'; 
      dist = driveStraightAndCourseCorrect(speed); 
    } else if (openings[0] == 'L') {
      currentRelativeDirection = 'L'; 
      turn90DegLeft(turnSpeed); 
      dist = driveStraightAndCourseCorrect(speed); 
    } else if (openings[0] == 'R') {
      currentRelativeDirection = 'R'; 
      turn90DegRight(turnSpeed);
      dist = driveStraightAndCourseCorrect(speed);
    } 
    Node *lastNode = g.nodes[g.nodes.size() - 1]; 
    if (distanceTravelled > lastNode->distanceTravelled) {
      createNodeAndSearchThroughGraph(dist);
    }
  }  
} 

// find the first previous node in the graph that has an alternative opening
void findPreviousOpening(Graph & g, Node *start, Node *finish) {
  Arc *arcStartToFinish; 
  for (int i = 0; i < g.arcs.size(); i++) {
    Arc *currentArc = g.arcs[i]; 
    if (currentArc->start->order == start->order and currentArc->finish->order == finish->order) {
      arcStartToFinish = currentArc; 
    }
  } 

  Vector<Arc *> outGoingArcs; 
  for (int i = 0; i < start->arcs.size(); i++) {
    if (start->order == start->arcs[i]->start->order) {
      outGoingArcs.add(start->arcs[i]);
    }
  }
 
  Vector<char> availableOpenings; 
  for(int i = 0; i < start->openings.size(); i++) {
    char availableOpening = start->openings[i]; 
    bool availableOpeningVisited = false;
    for(int j = 0; j < outGoingArcs.size(); j++) { 
      if (availableOpening == outGoingArcs[j]->direction) { 
        availableOpeningVisited = true;
      }
    }
    if (!availableOpeningVisited) { availableOpenings.add(availableOpening); }
  }

  float distBtwNodes = arcStartToFinish->distanceBetweenNodes; 

  if (arcStartToFinish->direction == 'F') {
    driveBackward(distBtwNodes, speed);
  } else if(arcStartToFinish->direction == 'L') {
    driveBackward(distBtwNodes, speed);
    turn90DegRight(turnSpeed);
  } else if(arcStartToFinish->direction == 'R') {
    driveBackward(distBtwNodes, speed);
    turn90DegLeft(turnSpeed);
  }

    Vector <Node *> upstreamNeighbors; 
    for(int k = 0; k < start->arcs.size(); k++) {  
      Arc *arc = start->arcs[k]; 
      if(arc->finish->order == start->order) {
        upstreamNeighbors.add(arc->start);
      }
    }
  
  if(availableOpenings.size() == 0) { 
    // if there are no available alternative openings, go back yet another node 
    findPreviousOpening(g, upstreamNeighbors[0], start);
  } else {
    float startingDistance = g.nodes[g.nodes.size() - 1]->distanceTravelled; 
    float finishingDistance = start->distanceTravelled; 
    float distDifference = abs(startingDistance - finishingDistance); 
    addArc(g, g.nodes[g.nodes.size() - 1], start, currentRelativeDirection, distDifference); 

    float dist; 
    
    if (availableOpenings[0] == 'F') {
      currentRelativeDirection = 'F'; 
      dist = driveStraightAndCourseCorrect(speed);
    } else if (availableOpenings[0] == 'L') {
      currentRelativeDirection = 'L'; 
      turn90DegLeft(turnSpeed); 
      dist = driveStraightAndCourseCorrect(speed);
    } else if (availableOpenings[0] == 'R') {
      currentRelativeDirection = 'R';  
      turn90DegRight(turnSpeed);
      dist = driveStraightAndCourseCorrect(speed);
    }
    
    Node *currentNode = g.arcs[g.arcs.size() - 1]->finish;
    if (distanceTravelled > currentNode->distanceTravelled) {
      createNodeAndSearchThroughGraph(dist);
    }
  }
} 

// end graph functions 

/*
 * sensing / wall detection  
*/

// sense presence or absence of a wall forwards, left, and right 
void scanEnvironment(Vector<float> *distances) {
  float forward = detectForward(); 
  float left = detectLeft(); 
  float right = detectRight();

  turnServo(90); 

  distances->add(forward);
  distances->add(left);
  distances->add(right);
} 
 
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

// drive straight backward a specified distance and correct course at each interval 
void driveBackward(float dist, int power) { 
  courseCorrect(turnMaxDistToWall, turnMinDistToWall);
  while (dist >= 0) { 
    driveStraight(travelDistance, power, 'B'); 
    stopMotors(); 
    courseCorrect(maxDistToWall, minDistToWall); 
    dist -= travelDistance; 
    distanceTravelled -= travelDistance;
  }
}

// drive straight until the wall ahead is reached 
// stop if there are four consecutive openings on the left AND right
// return distance travelled
float driveStraightAndCourseCorrect(int power) { 
  float forward = detectForward();
  float totalDistance = 0.0;   
  
  while (forward >= stoppingDistance) {
    forward = detectForward(); 
    if (consecutiveLeftOpenings == 4 && consecutiveRightOpenings == 4) {
      Serial.println(F("You've stopped driving because you've left the maze!")); 
      break;
    } else if (consecutiveLeftOpenings == 3 || consecutiveRightOpenings == 3) {
      createNodeAndSearchThroughGraph(totalDistance);
      forward = detectForward();
      break;
    }
    
    driveStraight(travelDistance, speed, 'F'); 
    stopMotors(); 
    courseCorrect(maxDistToWall, minDistToWall); 

    if (distLeft > wallThreshold) {
      consecutiveLeftOpenings++;
    } else {
      consecutiveLeftOpenings = 0;
    }

    if (distRight > wallThreshold) {
      consecutiveRightOpenings++;
    } else {
      consecutiveRightOpenings = 0;
    }
    
    forward = detectForward(); 
    totalDistance += travelDistance; 
    distanceTravelled += travelDistance;  
  }
  return totalDistance; 
}

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
  courseCorrect(turnMaxDistToWall, turnMinDistToWall);
}

void turn90DegRight(int power) {
  driveStraight(distanceFor90DegTurns, power, 'R'); 
  courseCorrect(turnMaxDistToWall, turnMinDistToWall);
}

void stopMotors() {
  digitalWrite(lb, LOW); 
  digitalWrite(lf, LOW); 
  digitalWrite(rb, LOW); 
  digitalWrite(rf, LOW); 
  analogWrite(leftSpeedPin, 0); 
  analogWrite(rightSpeedPin, 0);  
}

// correct course based on distance to left and right barriers 
void courseCorrect(int maximum, int minimum) {
  distLeft = detectLeft(); 
  distRight = detectRight(); 
  float diff = abs(distLeft - distRight);
  if (distLeft <= wallThreshold and distRight > wallThreshold) { 
    if (distLeft < minimum) {
      smallTurnRight(courseCorrectSpeed, courseCorrectSpeed); 
    } else if (distLeft > maximum) {  
      smallTurnLeft(courseCorrectSpeed, courseCorrectSpeed); 
    }
    courseCorrect(maximum, minimum);
  } else if (distRight <= wallThreshold and distLeft > wallThreshold) {
    if (distRight < minimum) {
      smallTurnLeft(courseCorrectSpeed, courseCorrectSpeed); 
    } else if (distRight > maximum) {
      smallTurnRight(courseCorrectSpeed, courseCorrectSpeed); 
    }
    courseCorrect(maximum, minimum);
  } else if (distLeft <= wallThreshold and distRight <= wallThreshold) {
    if (diff > 2) {
      if (distLeft > distRight) {
        smallTurnLeft(courseCorrectSpeed, courseCorrectSpeed);
      } else if (distRight > distLeft) {
        smallTurnRight(courseCorrectSpeed, courseCorrectSpeed); 
      }
    }
    courseCorrect(maximum, minimum);
  }
}


