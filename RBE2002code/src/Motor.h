#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include "Servo.h"

/************************
 * Class for the motors *
 ************************/

class Motor {
public:
	Motor(int digitalPin, int analogPin, boolean reversed);
	void initialize();
	void setPower(int);

private:
	long lastSetTime;
	boolean isReversed;
  int dPin;
  int aPin;
};
#endif
