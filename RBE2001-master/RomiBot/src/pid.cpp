#include "pid.h"


void PID::init(double P, double I, double D, double Bias, double Iswitch) {
    p = P;
    i = I;
    d = D;
    bias = Bias;
    iSwitch - Iswitch;
}

double PID::updatePID(double point, double setpoint) {
    if(millis() > 10 + prevMillis) {

    iterationTime = millis() - prevMillis;
    prevMillis = millis();
  
    error = setpoint - point;

    if(abs(error) > iSwitch) {
        integral = 0;
    } 
    else {
        integral = prevInt + error * iterationTime;
    }
    derivative = (error - prevError) / iterationTime;
    output = (p * error) + (i * integral) + (d * derivative) + bias;
    prevError = error;
    prevInt = integral;

    if(output > 400) {
        output = 400;
    }
    if(output < -400) {
        output = -400;
    }
    return output;
    }
}

void PID::reset() {
    iterationTime = millis();
    prevMillis = millis();
}