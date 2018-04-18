#ifndef DRIVE_H
#define DRIVE_H
#include <Arduino.h>
#include "Motor.h"

/**********************************
 * Drive class to control motors  *
 **********************************/

class drive {
public:
  drive();
  void initialize();
  void setPower(int lMotor, int rMotor);

private:
  Motor leftMotor;
  Motor rightMotor;
};

#endif
