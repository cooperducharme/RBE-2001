#include "motor.h"
#include "pid.h"

volatile int errorCount = 0;
volatile long count = 0;
volatile int oldValue = 0;

/**
 * Method to setup the motor pins
 * @param d1, d2, pwm, hA, hB: where its plugged in
 *
 * */
void BlueMotor::setup(int d1, int d2, int p, int hA, int hB) {
  dir1 = d1;
  dir2 = d2;
  pwm = p;
  hallA = hA;
  hallB = hB;
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(hallA, INPUT);
  pinMode(hallB, INPUT);

  controller.init(2, .02, .01, 0, 50);

  oldValue = (digitalRead(hallA) << 1) | digitalRead(hallB);
  targetPos = 0;
}

/**
 * Method to run the motor in some direction
 * @param power value between -255 and 255 to run the motor
 * */
void BlueMotor::setEffort(int power) {
  if (power > 0) {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    analogWrite(pwm, power);
  } else if (power < 0) {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    analogWrite(pwm, -power);
  } else {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, HIGH);
    analogWrite(pwm, 0);
  }
}
/**
 * Method to convert a number from one range to another proportional range
 * @param power value between -400 and 400 to convert to a Dead Band corrected
 * range
 * */
double noDBRangeConvert(int power) {
  double DBMin = 0;
  double DBMax = 400;
  double noDBMin = 100;
  double noDBMax = 255;
  double noDBRange = (noDBMax - noDBMin);
  if (power > 0) {
    return (((abs(power) - DBMin) * noDBRange) / DBMax) + noDBMin;
  } else {
    return -((((abs(power) - DBMin) * noDBRange) / DBMax) + noDBMin);
  }
}

void BlueMotor::setEffortNoDB(int power) {
  int noDBPower = noDBRangeConvert(power);
  setEffort(noDBPower);
}

/**
 * Method to set the target position for the motor
 * @param deg value in deg for the motor to go
 * */
void BlueMotor::setTargetPosition(int counts) {
  // Convert from deg to counts
  targetPos = counts;
}

long BlueMotor::getPosition() { return count; }

double BlueMotor::getDeg() { return count / encoderDegValue; }

void BlueMotor::reset() { count = 0; }

void BlueMotor::loop() {
    setEffortNoDB(controller.updatePID(getPosition(), targetPos));
}

void BlueMotor::isr() {
  int newValue = (digitalRead(hallA) << 1) | digitalRead(hallB);
  char value = encoderArray[oldValue][newValue];
  if (value == X) {
    errorCount++;
  } else {
    count += value;
  }
  oldValue = newValue;
}

/**
 * Method that utilizes PID control in order to move Blue motor position to a
 * user inputted angle
 * @param deg desired angle of motor position
 * */
void BlueMotor::armPID(double deg) {

  // while(abs(deg - getDeg()) >= 0.5)
    // isr();
    // error = deg - getDeg();
    // integral = prevIntegral + error * iterationTime;
    // derivative = (error - prevError) / iterationTime;
    // double output = Kp * error + Ki * integral + Kd * derivative;
    // prevError = error;
    // prevIntegral = integral;
    // setEffortNoDB(output);
    // delay(iterationTime);
}

bool BlueMotor::armAtPos() {
    return abs(getPosition() - targetPos) <= 50;
}