#include "Ultrasonic.h"
#include <Arduino.h>


/**
 * Constructor for ultrasonic
 * @param trig Trig pin for ultrasonic
 * @param echo Echo pin for ultrasonic
 */
Ultrasonic::Ultrasonic(int trig, int echo){
  trigPin = trig;
  echoPin = echo;
  initialize();
}


/**
 * Initializes ultrasonic by setting trig and echo pins as outputs/inputs
 */
void Ultrasonic::initialize() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}


/**
 * Determines distance of ultrasonic from object
 * @return Distance from object in centimeters
 */
int Ultrasonic::readDistance(){
  unsigned long initial = micros();

  //set pin low for 2 microseconds
  if (micros()-initial <= 2){
    digitalWrite(trigPin, LOW);
  }

  //Pulse high for 10 microseconds
  initial = micros();
  if (micros()-initial <= 10){
    digitalWrite(trigPin, HIGH);
  }
  digitalWrite(trigPin, LOW);

  //Read in echo value
  int durationFrequency = pulseIn(echoPin, HIGH);

  //Convert echo value to distance
  int distanceFromWall = (durationFrequency/2) / 29.1;

  // Serial.print("Distance: ");
  // Serial.println(distanceFromWall);
  return distanceFromWall;
}


/**
 * Averages 10 readings of ultrasonic to return best value
 * @return Average ultrasonic distance
 */
 int Ultrasonic::avg(){
   int avg=0, sum=0,divider=0;

   for(int i=1; i < 9; i++){
     savedReads[9-i]=savedReads[9-i-1]; //shift array values to make room for new
   }

   savedReads[0]=readDistance(); //add new distance to array

   //Count number of values
   for(int i=0; i<5; i++){
     if(savedReads[i]<30 && savedReads[i]>0){
       sum+=savedReads[i];
       divider++;
     }
   }

   //Take average of values
   if(divider!=0){
     avg=sum/divider;
   }

   else{
     avg = readDistance();//random number that wont trigger anything
   }

   //Wrap protection
   if(!(avg<40&&avg>0)){
     avg=40;
   }

   return avg;
 }

 void Ultrasonic::clear(){
   for(int i = 0; i < 10; i++){
     savedReads[i] = 39;
   }
 }
