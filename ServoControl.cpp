#include "ServoControl.h"
#include <avr/io.h>
#include <util/delay.h>
#include <cmath>
#include <Arduino.h>

uint8_t kTurnScaler = 130;

//turn about yellow
void ServoControl::turnY() {
  // Rotate opposite?
  // Or spin one and pivot?
  servoYellow.write(90);
  servoRed.write(60);
}

//turn about red
void ServoControl::turnR() {
  // Rotate opposite?
  // Or spin one and pivot?
  servoYellow.write(60);
  servoRed.write(90);
}

void ServoControl::goForward() {
  servoYellow.write(0);
  servoRed.write(0);
}

// speed 1 - 500
//void ServoControl::goForwardAtSpeed(uint16_t sp) {
//  servoYellow.writeMicroseconds(1500 - sp);
//  servoRed.writeMicroseconds(1500 - sp);
//}

void ServoControl::goForwardAtSpeed(uint16_t sp) {
  servoYellow.write(90 - sp);
  servoRed.write(90 - sp);
}

void ServoControl::stopCar() {
  servoYellow.writeMicroseconds(1500);
  servoRed.writeMicroseconds(1500);
}

void ServoControl::turnForTime(unsigned long time) {
  turnY();
  _delay_ms(time);
  stopCar();
}

void ServoControl::turnAngle(int16_t angle) {
  if (angle > 0) 
  {
  //pivot on R
    turnR();
  } else 
  {
    //pivot on Y
    turnY();
  }
  _delay_ms((abs(angle) * kTurnScaler) / 10);
  stopCar();
}

void ServoControl::beginDriving()
{
  // read sensors
  sensorRed.readValue();
  sensorYellow.readValue();
  
  // choose which sensor is black
  if (sensorRed.currentValue >= sensorYellow.currentValue)
  {      
    Serial.println("Using Red as Black");
    sensorBlack = &sensorRed;
    sensorWhite = &sensorYellow;
    servoBlack = &servoRed;
    servoWhite = &servoYellow;
  }
  else
  {
    Serial.println("Using Yellow as Black");
    sensorBlack = &sensorYellow;
    sensorWhite = &sensorRed;
    servoBlack = &servoYellow;
    servoWhite = &servoRed;
  }
  lineDirection = 1;
  drive();
  Serial.println("Stopping...");
  stopCar();
}



void ServoControl::drive() 
{
  Serial.println("Beginning to drive");
  boolean drive = true;
  while(drive)
  {
    // read inputs inc. button input
    (*sensorBlack).readValue();
    (*sensorWhite).readValue();

    // stop driving if button is low? maybe low multiple times
//    int value = digitalRead(5); // Button FIXME
//    if (value == LOW)
//    {
//      drive = false;
//    }
    
//    uint8_t turnOffset = 4;
//    uint8_t maxTurn = 8;
//    uint16_t defaultSpeed = 8;

    uint8_t turnOffset = 16;
    uint8_t maxTurn = 32;
    uint16_t defaultSpeed = 8;
    
    // respond to inputs
    if ((*sensorBlack).isBlack() && !((*sensorWhite).isBlack()))
    {
     // B=B, W=W 
     //Serial.println("Moving forward");
     goForwardAtSpeed(defaultSpeed);
     lineDirection = 1;
//     Serial.println("Turning towards black (B=B, W=W)");
//      turnTowardsBlack(turnOffset, maxTurn, defaultSpeed);
    }
    else if ((*sensorBlack).isBlack() && (*sensorWhite).isBlack())
    {
      // B=B, W=B
      // Pivot on white side
      //Serial.println("Forward (B=B, W=B)");
      goForwardAtSpeed(defaultSpeed);
      //turnTowardsWhite(turnOffset, maxTurn, defaultSpeed);
    }
    else if (!((*sensorBlack).isBlack()) && !((*sensorWhite).isBlack()))
    {
      // B=W, W=W
      // Pivot on black side
      //Serial.println("Turning towards line (B=W, W=W)");
      turnTowards(lineDirection, turnOffset, maxTurn, defaultSpeed);
      //turnTowardsBlack(turnOffset, maxTurn, defaultSpeed);
    }
    else 
    {
      // B=W, W=B
      // Pivot on white side
      //Serial.println("Forward (B=W, W=B)");
      goForwardAtSpeed(defaultSpeed);
      //turnTowardsWhite(turnOffset, maxTurn, defaultSpeed);
      lineDirection = -1;
    }
  }
}

void ServoControl::turnTowardsBlack(uint8_t turnOffset, uint8_t maxDiff, uint16_t defaultSpeed)
{
  turnTowards(1, turnOffset, maxDiff, defaultSpeed);
}

void ServoControl::turnTowardsWhite(uint8_t turnOffset, uint8_t maxDiff, uint16_t defaultSpeed)
{
  turnTowards(-1, turnOffset, maxDiff, defaultSpeed);
}

// +1 black
// -1 white
void ServoControl::turnTowards(int8_t kDir, uint8_t turnOffset, uint8_t maxDiff, uint16_t defaultSpeed)
{
  uint8_t servoValueB = (*servoBlack).read(); //0-180
  uint8_t servoValueW = (*servoWhite).read(); //0-180
  Serial.print("CurrentB Servo Value: ");
  Serial.println(servoValueB);
  Serial.print("CurrentW Servo Value: ");
  Serial.println(servoValueW);
  
  if ((kDir * servoValueW) - (kDir * servoValueB) > 0)
  {
    servoValueB = 90 - defaultSpeed;
    servoValueW = 90 - defaultSpeed;
    
  }
  
  if ((kDir * servoValueB) - (kDir * servoValueW) < maxDiff)
  {
    (*servoBlack).write(servoValueB + (kDir * turnOffset));
    (*servoWhite).write(servoValueW - (kDir * turnOffset));
  }
}

