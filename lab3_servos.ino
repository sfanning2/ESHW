#include <avr/io.h>
#include "ServoControl.h"
#include <Servo.h>

// Pin 13 has an LED connected on most Arduino boards:
#define BOARD_LED 13
#define RED 10
#define GREEN 11
#define BLUE 12

#define BUTTON 5
#define SERVO_RED 9
#define SERVO_YELLOW 8

Servo servoR;
Servo servoY;

ServoControl car;

void setup() 
{                
  Serial.begin(9600);
  Serial.println("Starting up.");
  LEDInit();
  buttonInit();    
  carInit();
  blackWhiteInit();
  Serial.println("Finished setup");
}

void carInit() {
 // Servos
 servoR.attach(SERVO_RED);
 servoY.attach(SERVO_YELLOW); 
 
 car.servoRed = servoR;
 car.servoYellow = servoY;
 
 // Sensors
 car.sensorRed.pin = A4;
 car.sensorYellow.pin = A3;
}

void loop() {
  if (digitalRead(BUTTON) == LOW) {
    //square();
    car.beginDriving();
    delay(500);
    readSensors();
  }
}

/*
 * Start: One sensor on black, one on white. They should attempt to stay this way.
 * If the black sensor reads a whtie value, pivot on the white side?
 * If the white sensor reads a black value, pivot on the white side?
 * If 
 * 
 */

/**
 * Init all of the LEDs and test them
 **/ 
void LEDInit(){
  //Serial.begin(9600);
  pinMode(BOARD_LED, OUTPUT);     
  pinMode(RED, OUTPUT);     
  pinMode(GREEN, OUTPUT);     
  pinMode(BLUE, OUTPUT);     

   //Turn all off
  digitalWrite(RED,LOW);
  digitalWrite(GREEN,LOW);
  digitalWrite(BLUE,LOW);

  //Test LEDs
  uint16_t time = 100;
  Serial.print("Testing LEDs...");
  digitalWrite(RED,HIGH);
  delay(time);
  digitalWrite(RED,LOW);
  digitalWrite(GREEN,HIGH);
  delay(time);
  digitalWrite(GREEN,LOW);
  digitalWrite(BLUE,HIGH);
  delay(time);
  digitalWrite(BLUE,LOW);
  Serial.println("done.");
  }
  
void buttonInit()
{
  pinMode(BUTTON, INPUT);  
  //Enable pullup  
  digitalWrite(BUTTON, HIGH); 
}

void blackWhiteInit()
{
  // Wait for button press and read black
  uint16_t d_time = 2000;
  
  // Read black
  readBW(BLACK_INDEX);  

  // Read White
  readBW(WHITE_INDEX);
  
  int blackY = car.sensorYellow.whiteBlack[BLACK_INDEX];
  int blackR = car.sensorRed.whiteBlack[BLACK_INDEX];
  
  int whiteY = car.sensorYellow.whiteBlack[WHITE_INDEX];
  int whiteR = car.sensorRed.whiteBlack[WHITE_INDEX];
  
  // Verify acceptable numbers
  if (min(blackY, blackR) > max(whiteY, whiteR) + 100)
  {
    // Acceptable
    digitalWrite(GREEN, HIGH);
    delay(d_time);
    digitalWrite(GREEN, LOW);
  }
  else
  {
    // Not acceptable 
    digitalWrite(RED, HIGH);
    delay(d_time);
    digitalWrite(RED, LOW);
  }
}

void readBW(int blackOrWhite) {
  while(digitalRead(BUTTON) != LOW)
  {
   asm("nop"); 
  }  
  // Read and store black value
  int valueR_Y = analogRead(A3);
  int valueL_R = analogRead(A4);
  
  car.sensorYellow.whiteBlack[blackOrWhite] = valueR_Y;
  car.sensorRed.whiteBlack[blackOrWhite] = valueL_R;
  
  Serial.print("Sensor Y: ");
  Serial.println(car.sensorYellow.whiteBlack[blackOrWhite]);
  Serial.print("Sensor R: ");
  Serial.println(car.sensorRed.whiteBlack[blackOrWhite]);
  
  digitalWrite(BLUE, HIGH);
  delay(500);
  digitalWrite(BLUE, LOW);
}

void readWhite() {
  
}

void goForward()
{
  unsigned int fTime = 1000;
 unsigned int fSpeed = 200;
 car.goForwardAtSpeed(fSpeed);
 delay(fTime);
 car.stopCar();
}

void readSensors()
{
  
  int val1 = analogRead(A3);
  Serial.print("Sensor Y: ");
  Serial.println(val1);
  int val2 = analogRead(A4);
  Serial.print("Sensor R: ");
  Serial.println(val2);
}
