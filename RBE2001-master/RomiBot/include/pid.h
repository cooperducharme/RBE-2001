#ifndef PID_H_
#define PID_H_

#include <Arduino.h>

class PID
{
public:
    void init(double, double, double, double, double);
    double updatePID(double, double);
    void reset();

private:
    double p, i, d,
        bias,
        error,
        prevError,
        integral,
        prevInt,
        derivative,
        iSwitch,
        output;

    int iterationTime;
    unsigned long prevMillis;
};

#endif