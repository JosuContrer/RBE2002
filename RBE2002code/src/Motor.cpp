#include "Motor.h"
#include "globalPins.h"

/**Constructor for Motor
 *
 * @param int digitalPin Digital Pin for the motor
 * @param int analogPin Analog Pin for the motor
 * @param boolean reversed True id the motor is reversed, otherwise false
 */
Motor::Motor(int digitalPin, int analogPin, boolean reversed){
  dPin = digitalPin;
  aPin = analogPin;
  isReversed = reversed;
}

/**Initliaizes the digital and analog pins as outputs for the motor
 */
void Motor::initialize() {
  pinMode(dPin,OUTPUT);
  pinMode(aPin,OUTPUT);
	setPower(0);
}

/**Sets the power to the motors
 *
 * @param int power from -255 to 255
 */
void Motor::setPower(int power) {

  //Inverts the motor direction if true
  if(isReversed == true){
    power = -1*power;
  }

//	if ((millis() - lastSetTime) > 20) {	//prevents from updating the motor too quickly
		if (power < 0) {
			digitalWrite(dPin, LOW);
    }
    else{
      digitalWrite(dPin, HIGH);
    }

    analogWrite(aPin, abs(power));
	//	lastSetTime = millis();
	//}
}
