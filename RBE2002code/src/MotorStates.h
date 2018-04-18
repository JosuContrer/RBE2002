#ifndef MOTORSTATES_H
#define MOTORSTATES_H

#include <Arduino.h>
#include "States.h"
#include <LiquidCrystal.h>
#include "drive.h"
#include "globalPins.h"
#include "Motor.h"


/**********************************************
 * Use this for testing out motor controllers *
 **********************************************/

class MotorStates{
public:
  MotorStates();
  void initialize();
  testState test;
  void motorDrive(testState test);

private:
  drive driveTrain;
};
#endif
