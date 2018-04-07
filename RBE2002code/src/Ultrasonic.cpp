#include "Ultrasonic.h"
#include <Arduino.h>

Ultrasonic::Ultrasonic(int trig, int echo){
  trigPin = trig;
  echoPin = echo;
  initialize();
}

void Ultrasonic::initialize() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}



int Ultrasonic::readDistance(){
  unsigned long initial = micros();
  
  if (micros()-initial <= 2){
    digitalWrite(trigPin, LOW);
  }

  initial = micros();
  if (micros()-initial <= 10){
    digitalWrite(trigPin, HIGH);
  }

  digitalWrite(trigPin, LOW);

  int durationFrequency = pulseIn(echoPin, HIGH);
  //Serial.print("durationFrequency: ");
  //Serial.println(durationFrequency);

  int distanceFromWall = (durationFrequency/2) / 29.1;


  Serial.print("Distance: ");
  Serial.println(distanceFromWall);

  return distanceFromWall;
}
