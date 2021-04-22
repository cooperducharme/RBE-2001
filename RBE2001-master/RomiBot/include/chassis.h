#include <Romi32U4.h>

class Chassis {

public:
  void driveDistance(float inches, int maxspeed);
  void drive(int left, int right);
  void turnAngle(float degrees);
  void loop();
  void startTurn(float);
  bool isTurnOver();

private:
  int16_t clip(int16_t, int16_t, int16_t);
  Romi32U4Motors motors;
  Romi32U4Encoders encoders;
  double error;
};

