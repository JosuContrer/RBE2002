#include "Fan.h"
#include "globalPins.h"
#include <Arduino.h>
#include <Servo.h>

Servo fanServo; //If want access to servo in other file, must declare as extern

void fanInitialize(){
  pinMode(FANPIN, OUTPUT);
  digitalWrite(FANPIN, LOW);
  fanServo.attach(FANSERVOPIN);
}

void fanState(bool on){
  if(on == true){
    digitalWrite(FANPIN, HIGH);
  }
  else{
    digitalWrite(FANPIN, LOW);
  }
}
