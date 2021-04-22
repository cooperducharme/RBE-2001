#include <Arduino.h>

class Rangefinder {
public:
    void setup(int, int);
    void loop();
    void updateDistance();
    float getDistanceCM();
    void isr();

private: 
    volatile float distance;
    volatile long echoStart;
    long startTime;
    int tPin, ePin;
};