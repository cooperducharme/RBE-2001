#include "motor.h"
#include "chassis.h"
#include "rangefinder.h"
#include <Arduino.h>
#include <Romi32U4.h>
#include "RemoteConstants.h"
#include "IRdecoder.h"
#include "servo32u4.h"
#include "pid.h"
#include "linefollower.h"

/**
 * Define pin numbers
 * */
#define trigPin 12
#define echoPin 0
#define servoPin 5
#define PWM 6
#define DIR1 13
#define DIR2 4
#define HALLA 2
#define HALLB 3
#define RECV_PIN 14
#define RSL 20
#define RSR 21
#define ESTOP remotePlayPause
#define CONFIRM remoteVolPlus

void isr_us();
void isr_e();
void lineFollow();
enum MainState
{
  E_STOP,
  WAIT_FOR_IR,
  FOLLOW_LINE1,
  FOLLOW_LINE2,
  MOVE_ARM,
  TURN,
  SWITCH_SERVO,
  WAIT_FOR_ARM
};
enum ArmState
{
  PLATFORM = -2950,
  DEGREE25 = 5700,
  DEGREE45 = 0,
  DEGREE50 = 600,
  DEGREE20 = 5500,
  PLATFORM_OFFSET = -2600
};

Chassis robot;
Rangefinder us;
LineFollower lf;
Romi32U4ButtonB buttonB;
BlueMotor arm;

MainState mainState = WAIT_FOR_IR; // WAIT_FOR_IR;
MainState prev;
ArmState armTarget = DEGREE45;

IRDecoder decoder;
uint8_t currentCode;

//decode_results results;

PID pidcontrol;

volatile int countA = 0;
volatile int countB = 0;

Servo32U4 grabber;

int confirmed = 0;
float targetDist = 18;
int stateCount = 5;

int servoTarget = 1500;

void setup()
{
  Serial.begin(11520);
  us.setup(trigPin, echoPin);
  arm.setup(DIR1, DIR2, PWM, HALLA, HALLB);

  grabber.Init();
  grabber.Attach();
  // servo.Attach();
  grabber.SetMinMaxUS(900, 2100);

  grabber.Write(1500);

  //pidcontrol.init(2, .02, 0.01, 0, 50);

  //robot.startTurn(90);

  decoder.init();

  attachInterrupt(digitalPinToInterrupt(echoPin), isr_us, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLA), isr_e, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLB), isr_e, CHANGE);

  us.updateDistance();
  //irrecv.enableIRIn();
  delay(5);
}

void loop()
{
  // Serial.print("Right: ");
  // Serial.println(analogRead(RSR));
  // Serial.print("Left: ");
  // Serial.print(analogRead(RSL));

  currentCode = decoder.getKeyCode();
  if (currentCode == ESTOP)
  {
    prev = mainState;
    mainState = E_STOP;
  }

  if (mainState == E_STOP)
  {
    robot.drive(0, 0);
    arm.setEffort(0);
  }
  else
  {
    us.loop();
    arm.loop();
    //arm.setTargetPosition(armTarget);
  }

  switch (mainState)
  {
  case E_STOP:
    if (currentCode == CONFIRM)
    {
      mainState = prev;
    }
    break;

  case WAIT_FOR_IR:
    //Serial.println("Waiting for IR");
    // Based on this value and the previous
    if (currentCode == CONFIRM)
    {
      Serial.println("IR CONFIRMED");
      mainState = stateCount == 0 || stateCount == 5 ? FOLLOW_LINE1 : SWITCH_SERVO;

      stateCount++;
      //delay(15);
      Serial.println(stateCount);
    }
    break;

  case FOLLOW_LINE1:
    if (stateCount == 1)
    {
      armTarget = DEGREE45;
      arm.setTargetPosition(DEGREE45);
    }
    else if (stateCount == 4)
    {
      arm.setTargetPosition(DEGREE50);
    }
    else if (stateCount == 6 || stateCount == 9)
    {
      targetDist = 10;
      armTarget = DEGREE25;
      arm.setTargetPosition(DEGREE25);
    }

    if (us.getDistanceCM() > targetDist)
    {
      robot.drive(65 - lf.getLineFollowValues(), 65 + lf.getLineFollowValues());
      // Set effort based on line sensors
    }
    // else if (!arm.armAtPos())
    // {
    //   // Keep waiting
    //   //Serial.print("Arm pos: ");
    //   //Serial.println(arm.getPosition());
    //   //robot.drive(0, 0);
    // }
    else
    {
      robot.drive(0, 0);
      if (stateCount == 2 || stateCount == 7)
      {
        armTarget = PLATFORM;
        arm.setTargetPosition(PLATFORM);
      }
      mainState = WAIT_FOR_IR;
    }

    break;

  case FOLLOW_LINE2:
    Serial.println(mainState);
    // Drive backward until both line sensors are on the black line or US sensor
    if (!(analogRead(RSL) > 850 && analogRead(RSR) > 850))
    {
      // Set effort based on line sensors
      //Serial.print(analogRead(RSL));
      //Serial.print("\t");
      //Serial.println(analogRead(RSR));
      robot.drive(-71, -60); //-(70 + lf.getLineFollowValues()), -(70 - lf.getLineFollowValues()));
    }
    else
    {
      robot.drive(100, 100);
      delay(200);
      robot.drive(0, 0);
      mainState = TURN;
      Serial.println(stateCount);
      if (stateCount == 2 || stateCount == 9)
        robot.startTurn(-85);
      else
        robot.startTurn(85);
    }

    break;

  case SWITCH_SERVO:
    // While the servo is not in the position we want
    if (servoTarget == 1500)
    {
      servoTarget = 900;
      if (armTarget == DEGREE45)
      {
        Serial.println("Arm at 45, going to 50");

        armTarget = DEGREE50;
        arm.setTargetPosition(DEGREE50);
      }
      else if (armTarget == DEGREE25)
      {
        armTarget = DEGREE20;
        arm.setTargetPosition(DEGREE20);
      }
      else
      {
        armTarget = PLATFORM_OFFSET;
        arm.setTargetPosition(PLATFORM_OFFSET);
      }
    }
    else
    {
      servoTarget = 1500;
    }

    grabber.Write(servoTarget);
    mainState = WAIT_FOR_ARM;

    break;

  case WAIT_FOR_ARM:
    Serial.println("Waiting for arm");
    if (arm.armAtPos())
    {
      mainState = stateCount == 3 || stateCount == 5 || stateCount == 8 || stateCount == 10 ? WAIT_FOR_IR : FOLLOW_LINE2;
    }
    break;

  case TURN:
    // Set left and right motor encoder counts to a turn value
    robot.loop();

    if (robot.isTurnOver())
    {
      mainState = FOLLOW_LINE1;
      targetDist = targetDist == 18 ? 12 : 18;
    }
    break;

  default:
    break;
  }
}

void isr_us() { us.isr(); }

void isr_e() { arm.isr(); }

void linefollow()
{
}