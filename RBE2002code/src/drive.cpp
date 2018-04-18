#include "Motor.h"
#include "drive.h"
#include "globalPins.h"
#include <Ultrasonic.h>
#include "PID.h"

/**
 * Constructor for drive class
 */
drive::drive():
  leftMotor(DLEFTMOTOR, ALEFTMOTOR, false),
  rightMotor(DRIGHTMOTOR, ARIGHTMOTOR, true)
  {

  }


/**
 * Initialize drive class by setting motors to be stopped
 */
void drive::initialize(){
  leftMotor.setPower(0);
  rightMotor.setPower(0);
}


/**
 * Sets drivetrain motors to be left and right input speeds
 * @param lMotor Left motor speed
 * @param rMotor Right motor speed
 */
void drive::setPower(int lMotor, int rMotor){
  leftMotor.setPower(lMotor);
  rightMotor.setPower(rMotor);
}
