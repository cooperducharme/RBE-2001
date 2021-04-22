#include "linefollower.h"

double LineFollower::getLineFollowValues()
{
    double kP = .15;

    int effort = analogRead(RSL) - analogRead(RSR);

    return effort*kP;

    //robot.drive(100 - effort * kP, 100 + effort * kP);
}