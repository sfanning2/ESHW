#include "Sensor.h"
#include <Arduino.h>
bool Sensor::isBlack()
{
  blackThreshold = whiteBlack[1] - 100;
  Serial.print("Th: ");
  Serial.println(blackThreshold);
  Serial.println("CV:");
  Serial.println(currentValue);
  return (currentValue >= blackThreshold); 
}

int Sensor::readValue()
{
  currentValue = analogRead(pin);
  return currentValue; 
}
