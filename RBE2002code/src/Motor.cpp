#include "Motor.h"

Motor::Motor(int digitalPin, int analogPin, boolean reversed){
  dPin = digitalPin;
  aPin = analogPin;
  isReversed = reversed;
}

/**
* Sets up the motor
*/
void Motor::motorSetup() {
  pinMode(dPin,OUTPUT);
  pinMode(aPin,OUTPUT);
	setPower(0);
}

/*
* Sets power to the motor. Anything under 20% speed will automatically become 0 for stall prevention
* @param int power from -100 to 100
*/
void Motor::setPower(int power) {

	if ((millis() - lastSetTime) > 20) {	//prevents from updating the motor too quickly
		if (power < 0) {
			digitalWrite(dPin, LOW);
    }else{
      digitalWrite(dPin, HIGH);
    }
    analogWrite(aPin, abs(power));

		lastSetTime = millis();
	}
}
