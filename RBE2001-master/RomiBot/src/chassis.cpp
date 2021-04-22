#include "chassis.h"

const float wheelDiameter = 2.8;
const int CPR = 1440;
const float wheelTrack = 5.75;

int target = 0;
float error = 0;
int current = 0;

bool doneTurning = true;

/**
 * Function that returns a clipped version of v
 * @param v number to be clipped
 * @param lo lower bound
 * @param hi upper bound
 * @return int
 *
 * */
int16_t Chassis::clip(int16_t v, int16_t lo, int16_t hi)
{
  return v > lo ? (v < hi ? v : hi) : lo;
}

/**
 * Method to drive straight inches using encoders. Blocking!
 * @param inches number of inches to drive
 *
 * */
void Chassis::driveDistance(float inches, int maxspeed)
{
  int target = CPR * inches / (wheelDiameter * 3.14);

  // Reset encoders
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  int current = 0;
  int error = target;

  // Arbitrarily choosing to only poll left motor
  while (abs(error) > 15)
  {
    // increment distance travelled
    current += encoders.getCountsAndResetLeft();
    error = (target - current) * 0.25;

    // Clip the error value for proportional control
    motors.setEfforts(-clip(error, -maxspeed, maxspeed),
                      clip(error, -maxspeed, maxspeed));
  }

  // Stop
  motors.setEfforts(0, 0);
}

/**
 * Method to turn on point using encoders converting degrees to distance
 * @param degrees number of degrees to turn (+ is counterclockwise)
 *
 * */
void Chassis::turnAngle(float degrees)
{
  int target = CPR * wheelTrack * degrees / (wheelDiameter * 360);

  // Reset encoders
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  int current = 0;
  int error = target;

  // Use the right motor because it reflects positive = counterclockwise
  while (abs(error) > 15)
  {
    // increment distance travelled
    current += encoders.getCountsAndResetRight();
    error = (target - current) * 0.25;

    // Clip the error value for proportional control
    motors.setEfforts(-clip(error, -300, 300), clip(error, -300, 300));
  }

  // Stop
  motors.setEfforts(0, 0);
}

/**
 * Method to set powers to the drive motors
 * @params left, right integers [-300, 300] to set motor powers
 * */
void Chassis::drive(int left, int right) { motors.setEfforts(left, right); }

void Chassis::startTurn(float degrees) {
	target = CPR * wheelTrack * degrees / (wheelDiameter * 360);

	// Reset encoders
	encoders.getCountsAndResetLeft();
	encoders.getCountsAndResetRight();
	current = 0;
	error = target;

	doneTurning = false;
}

void Chassis::loop() {

  // increment distance travelled
  if (abs(error) > 15) {
    current += encoders.getCountsAndResetRight();
    error = (target - current) * 0.5;

    // Clip the error value for proportional control
    motors.setEfforts(-clip(error, -300, 300), clip(error, -300, 300));
  } else {
        motors.setEfforts(0, 0);
        doneTurning = true;
  }
}

bool Chassis::isTurnOver() {
  return doneTurning;
}