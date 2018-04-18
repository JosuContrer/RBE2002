#include "Motor.h"
#include "globalPins.h"
#include <Arduino.h>


/**
 * Constructor for the motor
 * @param digitalPin Digital pin for the motor
 * @param analogPin  Analog pin for the motor
 * @param reversed   True if the motor is reversed, otherwise false
 */
Motor::Motor(int digitalPin, int analogPin, boolean reversed){
  dPin = digitalPin;
  aPin = analogPin;
  isReversed = reversed;
}


/**
 * Initializes the digital and analog pins as outputs for the motor
 */
void Motor::initialize() {
  pinMode(dPin,OUTPUT);
  pinMode(aPin,OUTPUT);
	setPower(0);
}


/**
 * Sets the motor power
 * @param power -255 to 255, negative makes go in reverse
 */
 void Motor::setPower(int power) {

   //Inverts the motor direction if true
   if(isReversed == true){
     power = -1*power;
   }
   if (power < 0) {
     digitalWrite(dPin, LOW); //set in reverse direction
   }
   else{
     digitalWrite(dPin, HIGH); //set in forwards direction
   }
   analogWrite(aPin, abs(power)); //write PWM signal to motr
 }
