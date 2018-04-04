#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include "Servo.h"


class Motor {
public:
	Motor(int digitalPin, int analogPin, boolean reversed);
	void motorSetup();
	void setPower(int);

private:
	long lastSetTime;
	boolean isReversed;
  int dPin;
  int aPin;
};
#endif
