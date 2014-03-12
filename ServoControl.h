#include <Servo.h>
#include "Sensor.h"

#define WHITE_INDEX 0
#define BLACK_INDEX 1

class ServoControl 
{
  public:
  Servo servoYellow;
  Servo servoRed;
  Sensor sensorYellow;
  Sensor sensorRed;
  
  Servo *servoBlack;
  Servo *servoWhite;
  Sensor *sensorBlack;
  Sensor *sensorWhite;
  
  int8_t lineDirection;// 1 or -1
  
  void turn(int angle);

  void turnY();
  void turnR();

  void goForward();
  void goForwardAtSpeed(uint16_t sp);

  void stopCar();  
  
  void turnForTime(unsigned long time);
  void turnAngle(int16_t angle);
  
  void beginDriving();
  void drive();
  
  void turnTowardsBlack(uint8_t turnOffset, uint8_t maxDiff, uint16_t defaultSpeed);
  void turnTowardsWhite(uint8_t turnOffset, uint8_t maxDiff, uint16_t defaultSpeed);
  void turnTowards(int8_t kDir, uint8_t turnOffset, uint8_t maxDiff, uint16_t defaultSpeed);
};

