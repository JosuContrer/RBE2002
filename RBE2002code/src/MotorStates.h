#ifndef MOTORSTATES_H
#define MOTORSTATES_H

#include <Arduino.h>
#include "States.h"
#include <LiquidCrystal.h>
#include "drive.h"
#include "globalPins.h"
#include "Motor.h"

class MotorStates{
public:
  MotorStates();
  void initialize();
  testState test;
  void motorDrive(testState test);

private:
  drive driveTrain;
  //Motor leftMotor(DRIGHTMOTOR,ARIGHTMOTOR,false);
  //LiquidCrystal lcd2(40, 41, 42, 43, 44, 45);
};
#endif
