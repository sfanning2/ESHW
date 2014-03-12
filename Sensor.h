#include <avr/io.h>
class Sensor
{
  public:
    int pin;
    int whiteBlack[2];
    int whiteThreshold;
    int blackThreshold;
    int previousValueBuffer[10];
    int bufferIndex;
    int currentValue;
    
    bool isBlack();
    int readValue();
};
