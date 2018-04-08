#include "Motor.h"
#include "drive.h"
#include "globalPins.h"
#include <Ultrasonic.h>
#include "PID.h"


drive::drive():
  leftMotor(DLEFTMOTOR, ALEFTMOTOR, false),
  rightMotor(DRIGHTMOTOR, ARIGHTMOTOR, true)
  {

  }

void drive::initialize(){
  leftMotor.setPower(0);
  rightMotor.setPower(0);
}

void drive::setPower(int lMotor, int rMotor){
  leftMotor.setPower(lMotor);
  rightMotor.setPower(rMotor);
}
