// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>
#include <Servo.h>
#include "pitches.h"

// Drive Motors
AF_DCMotor leftMotor(1);
AF_DCMotor rightMotor(2);

// Pan & Tilt Servos
Servo tiltServo;
Servo panServo;


// Proximity Sensor
#define trigger 12
#define echo 13

// LEDs
int leftLED = 5;
int rightLED = 6;

// Piezo
int piezo = 8;


// Config
int currentDistanceLeft, currentDistanceRight, currentDistanceAhead;
int minimumDistance = 30;
int pos = 90;
bool obstacleLeft, obstacleRight, obstacleAhead;
int tiltServoDefaultPosition = 150;
int panServoDefaultPosition = 90;
int defaultServoActuationTime = random(0, 4);
int turnFactor = 90;

void setup() {

  Serial.begin(9600);

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  // Turn on servos
  tiltServo.attach(10);
  panServo.attach(9);

  // Set servos starting position
  tiltServo.write(tiltServoDefaultPosition);
  panServo.write(panServoDefaultPosition);

  // Setup proximity sensor pins
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);

  leftMotor.setSpeed(100);
  rightMotor.setSpeed(100);

  robotInitiated();
  
}

void loop() {
  simpleObstacleAvoidence();

  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
}

void moveForwards() {
  leftMotor.run(FORWARD);
  rightMotor.run(BACKWARD);
};


void moveBackwards() {
  leftMotor.run(BACKWARD);
  rightMotor.run(FORWARD);
}

void halt() {
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
}

void turnLeft() {
  for (int i = 0; i <= turnFactor; i++) {
    leftMotor.run(BACKWARD);
    rightMotor.run(BACKWARD);
    delay(10);
  }
  halt();
}

void turnRight() {
  for (int i = 0; i <= turnFactor; i++) {
    leftMotor.run(FORWARD);
    rightMotor.run(FORWARD);
    delay(10);
  }
  halt();
}

void smartTurnLeft() {
  turning();
  while (currentDistanceAhead <= minimumDistance) {
    scanAhead();
    leftMotor.setSpeed(50);
    rightMotor.setSpeed(50);
    leftMotor.run(BACKWARD);
    rightMotor.run(BACKWARD);
    blinkLED(leftLED);
    digitalWrite(rightLED, LOW);
    delay(20);
  }
}

void smartTurnRight() {
  turning();
  while (currentDistanceAhead <= minimumDistance) {
    scanAhead();
    leftMotor.setSpeed(50);
    rightMotor.setSpeed(50);
    leftMotor.run(FORWARD);
    rightMotor.run(FORWARD);
    blinkLED(rightLED);
    digitalWrite(leftLED, LOW);
    delay(20);
  }
}


void lookLeft() {
  rotateServo(panServo, 90, 180, defaultServoActuationTime);
  tiltAround();
  currentDistanceLeft = lookAndEvaluate();
  Serial.println("LEFT distance: ");
  Serial.println(currentDistanceLeft);
}

void lookRight() {
  rotateServo(panServo, 90, 0, defaultServoActuationTime);
  tiltAround();
  currentDistanceRight = lookAndEvaluate();
  Serial.println("RIGHT distance: ");
  Serial.println(currentDistanceRight);
}

void lookAhead() {
  rotateServo(panServo, panServo.read(), 90, defaultServoActuationTime);
  tiltAround();
  currentDistanceAhead = lookAndEvaluate();
  Serial.println("AHEAD distance: ");
  Serial.println(currentDistanceAhead);
}

void tiltAround() {
  int rFrom = tiltServo.read();
  int rTo = random(140, 180);
  rotateServo(tiltServo, rFrom, rTo, defaultServoActuationTime);
}

void rotateServo(Servo servo, int from, int to, int rotationSpeed) {
  if (from >= to) {
    for (pos = from; pos >= to; pos--) {
      servo.write(pos);
      delay(rotationSpeed);
    }
  } else if (from <= to) {
    for (pos = from; pos <= to; pos++) {
      servo.write(pos);
      delay(rotationSpeed);
    }
  }
}

int lookAndEvaluate() {
  delay(80);
  long duration, distance;
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, HIGH);
  distance = (duration / 2) / 29.1;
  return distance;
}

void scanSurroundings() {
  lookRight();
  delay(random(100, 400));
  lookAhead();
  delay(random(100, 400));
  lookLeft();
  delay(random(100, 400));
  lookAhead();
  delay(random(100, 400));
}

void scanAhead() {
  currentDistanceAhead = lookAndEvaluate();
  delay(100);
  Serial.println(currentDistanceAhead);
}

void simpleObstacleAvoidence() {
  //  scanSurroundings();
  if (currentDistanceAhead >= minimumDistance) {
    scanAhead();
    moveForwards();
  } else {
    halt();
    scanSurroundings();
    makeNewDirectionDecision();
  }
}

void makeNewDirectionDecision() {
  if (currentDistanceLeft >= currentDistanceRight) {
    Serial.println("Turning Left");
    //    turnLeft();
    smartTurnLeft();
  } else if (currentDistanceLeft <= currentDistanceRight) {
    Serial.println("Turning Right");
    //    turnRight();
    smartTurnRight();
  } else if (currentDistanceAhead >= minimumDistance) {
    moveForwards();
  }
  moveForwards();
}

void blinkLED(int LED) {
  if(digitalRead(LED) == LOW) {
    digitalWrite(LED, HIGH);
    delay(80); 
    digitalWrite(LED, LOW);     
  } else {
    digitalWrite(LED, LOW);
    delay(80); 
    digitalWrite(LED, HIGH);       
  }
}

void vocalize(int speakerPin, float noteFrequency, long noteDuration) {
  
  int x;
  
  // Convert the frequency to microseconds
  float microsecondsPerWave = 1000000/noteFrequency;
  
  // Calculate how many HIGH/LOW cycles there are per millisecond
  float millisecondsPerCycle = 1000/(microsecondsPerWave * 2);
  
  // Multiply noteDuration * number or cycles per millisecond
  float loopTime = noteDuration * millisecondsPerCycle;
  
  // Play the note for the calculated loopTime.
  
  for (x=0;x<loopTime;x++) {   
    digitalWrite(speakerPin,HIGH); 
    delayMicroseconds(microsecondsPerWave); 
    digitalWrite(speakerPin,LOW); 
    delayMicroseconds(microsecondsPerWave); 
  }
  
}   

void R2D2(){
  vocalize(piezo, note_A7,100); //A 
  vocalize(piezo, note_G7,100); //G 
  vocalize(piezo, note_E7,100); //E 
  vocalize(piezo, note_C7,100); //C
  vocalize(piezo, note_D7,100); //D 
  vocalize(piezo, note_B7,100); //B 
  vocalize(piezo, note_F7,100); //F 
  vocalize(piezo, note_C8,100); //C 
  vocalize(piezo, note_A7,100); //A 
  vocalize(piezo, note_G7,100); //G 
  vocalize(piezo, note_E7,100); //E 
  vocalize(piezo, note_C7,100); //C
  vocalize(piezo, note_D7,100); //D 
  vocalize(piezo, note_B7,100); //B 
  vocalize(piezo, note_F7,100); //F 
  vocalize(piezo, note_C8,100); //C 
}

void robotInitiated() {
  for (int i=100; i<12000; i=i*1.45) {
    vocalize(piezo,i,60);
  }
  delay(10);
  for (int i=100; i<12000; i=i*1.5) {
    vocalize(piezo,i,20);
  }
}

void turning() {
  int multiplyFactor = random(0.20, 1.45);
  for (int i=100; i<2000; i=i*1.35) {
    vocalize(piezo, i, 60);
  }
}



