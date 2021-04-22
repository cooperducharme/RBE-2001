#include "pid.h"
#include <Arduino.h>

class BlueMotor {

public:
    void setup(int d1, int d2, int pwm, int hA, int hB);
    void setEffort(int power);
    void loop();
    void setEffortNoDB(int power);
    void isr();
    long getPosition();
    void setTargetPosition(int deg);
    void reset();
    double getDeg();
    void armPID(double deg);
    bool armAtPos();


private:
    int16_t pos;
    int8_t dir1, dir2, pwm;
    int8_t hallA, hallB;
    int16_t targetPos;
    double error;
    const char X = 5;
    const float encoderDegValue = 1.52;
    char encoderArray[4][4] = {
        {0, -1, 1, X}, 
        {1, 0, X, -1}, 
        {-1, X, 0, 1}, 
        {X, 1, -1, 0}};
    PID controller;
};