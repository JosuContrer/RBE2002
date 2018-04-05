//Written by Joseph St. Germain for RBE 2002 Lab 3 A16.
//Students are expected to fill in the PID::calc function and the calcCur function
// The PID calc function is inteded to run a PID velocity controller 
//and the calcCur funstion should calculate the current in ma from the current sensor.

#include "LookupTable.h"
#include "PID.h"
#include <Servo.h>
#include <Encoder.h>

//Setup lookup table class as lk
Lookup lk;
//setup PID class as pidCon
PID pidCon;
Encoder myEnc(2, 3);

//Variables to run the code
double setVel=90;//90
double curTime=0;
double prevTime=0;
double timeInterval=0;
double movement=0;
double ticksToDeg= 0.1777777;//.2206
double prevPos;
long lastTime=0;
long lastTime2=0;
double Vel;
float velocity;
float RPM;
float current;
float PWM;
float analogWriteVal;

// Returns Vel in degress/second
double calcVel(){
  //current positions
  double curPos=myEnc.read();
  //current time
  curTime=millis();
  //time change in ms from last call
  timeInterval=curTime-prevTime;
  //encoder ticks since last call
  movement=curPos-prevPos;
  //encoder ticks to degrees
  movement= movement *ticksToDeg;
  //timeInterval in seconds
  timeInterval=timeInterval/1000;
  //Velocity in degrees per milliseconds
  Vel=movement/timeInterval;
  //sets curent vals to previous
  prevPos=curPos;
  prevTime=curTime;
  return Vel;
}


//This function should analogRead the current sense from the motor driver
//and convert the value to current in milliamps
double calcCur(void)
{
  float motorResistance = 2.4;    //voltage / stall current ---> voltage = 12V, stall current = 5 A
 
  //read analog value
  double analogVal = analogRead(0);

  //convert to volts
  //double voltVal = map(analogVal, 0.0, 1023.0, 0.0, 5.0);
   double voltVal = analogVal*0.0048875855;
   //Serial.println(voltVal);
  //converts to current in milliamps
  double currentVal = voltVal /.525 *1000;
  
  return currentVal;
}


void setup() {
  Serial.begin(115200);
  pidCon.setpid(.075,.05,.001);
  pinMode(0,INPUT);
}


//Main loop
void loop() {
 
//Waits for the time constant
if(millis()-lastTime>=10){
  
  //Calculate velocity from the encoder in degrees/second
  velocity=calcVel();

  //Calculates RPM from velocity
  RPM =((velocity*60)/360);
 
  //calculates the current of the motor from the analogRead function
  //use the motor drivers data sheet to find the conversion from ADC counts
  //to milliamps 
  current = calcCur();
  
  //Calculate the PID value to be sent to the motor using the analogWrite function.
  //The calc function should take in a set velocity and the current velocity.
  analogWriteVal = pidCon.calc(setVel,velocity);
  analogWrite(5,analogWriteVal);
  
  //Calculates the dutycycle of the PWM 0-1
  PWM = analogWriteVal/255;
 
 //Sets the lastTime variable 
  lastTime=millis();
}

//print data time
if(millis()-lastTime2>50){

  //Calculate velocity from the encoder in degrees/second
  //Serial.print("velocity= ,");
  Serial.print(velocity);
  Serial.print(", ");  

 //Calculates RPM from velocity
  //Serial.print("RPM = ,");
  Serial.print(RPM);
  Serial.print(", "); 

  //calculates the current of the motor from the analogRead function
  //use the motor drivers data sheet to find the conversion from ADC counts
  //to milliamps 
  //Serial.print("current = ,");
  Serial.print(current);
  Serial.print(", "); 

  //Calculates the dutycycle of the PWM 0-1
  //Serial.print("PWM = ,");
  Serial.print(PWM);
  Serial.print(", "); 

  //Use the lookup table class to find the torque given the current and RPM
  //Serial.print("torque = ,");
  Serial.print(lk.torque(current,RPM));
  Serial.println("");
  lastTime2=millis();
  }
}

