#include "Fan.h"
#include "globalPins.h"
#include <Arduino.h>
#include <Servo.h>

//////////////
//CONSTANTS //
//////////////
Servo fanServo; //NOTE: If want access to servo in other file, must declare as extern


/**
* Initializes both the digital pin for the fan and the servo to move the fan
*/
void fanInitialize(){
  pinMode(FANPIN, OUTPUT);
  digitalWrite(FANPIN, LOW);
  fanServo.attach(FANSERVOPIN);
}


/**
* Turns the fan on or off
* @param on Boolean to turn fan on or off
*/
void fanState(bool on){
  if(on == true){
    digitalWrite(FANPIN, HIGH);
  }
  else{
    digitalWrite(FANPIN, LOW);
  }
}
