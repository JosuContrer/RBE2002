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
int Ultrasonic::avg(){
  int avg=0;

  for(int i=1;i<9;i++){
  savedReads[9-i]=savedReads[9-i-1];
  }
  savedReads[0]=readDistance();
  int sum=0,divider=0;

  for(int i=0;i<5;i++){
    if (savedReads[i]<30 && savedReads[i]>0){
      sum+=savedReads[i];
      divider++;
    }
  }
if(divider!=0){
  avg=sum/divider;
}
else{
  avg = 50;//random number that wont trigger anything
}
if(!(avg<40&&avg>0)){
  avg=40;
}
return avg;
}
