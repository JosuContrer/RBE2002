#ifndef FAN_H
#define FAN_H

#include <Arduino.h>
#include <Servo.h>
//#include "Ultrasonic.h"
/**************************************************
 * Definition of functions for control of the fan *
 **************************************************/

//////////////
//CONSTANTS //
//////////////
#define ON 1
#define OFF 0


///////////////
// FUNCTIONS //
///////////////
void fanInitialize();
void fanState(bool);

class Fan{
public:
  Fan();
  void initialize();
  void setPower(int pp);

private:
  Servo bladeMotor;
  //Ultrasonic frontUltra(FRONTULTRATRIG, FRONTULTRAECHO);
};

#endif
