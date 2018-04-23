#include "Fan.h"
#include "globalPins.h"

//////////////
//CONSTANTS //
//////////////
Servo fanServo; //NOTE: If want access to servo in other file, must declare as extern

//TODO: Tests all this code in main
/**
* Constructor for Fan class
*/
Fan::Fan(){}

/**
* New fucntion to initialize both the digital pin for the fan and the servo to move
* the fan blade.
*/
void Fan::initialize(){
  //In order for Speed Controller to intialize correct
  bladeMotor.attach(FANPIN);
  delay(1);
  bladeMotor.write(10);
  delay(5000);
  fanServo.attach(FANSERVOPIN);
}

// /**
// * Initializes both the digital pin for the fan and the servo to move the fan
// */
// void fanInitialize(){
//   pinMode(FANPIN, OUTPUT);
//   digitalWrite(FANPIN, LOW);
//   fanServo.attach(FANSERVOPIN);
// }

/**
* Sets the blade power
* @param pp Integer that ranges from 0 to 180 for blade power
*/
void Fan::setPower(int pp){
  //int ppn = map(pp,0,180,10,100); // More intuitixe to understand
  bladeMotor.write(pp);
}

/**
* Sets the blade power to the max of 10 m/s
* @param allowed Boolean if true sets the fan on else fan off
*/
void Fan::maxPower(bool allowed){
  if(allowed){
    bladeMotor.write(100);
  }else{
    bladeMotor.write(10);
  }
}

/**
* Checks if the flame is in proximity while driving towards it
* @param dist Integer to indicate the distance wanted
* @return Boolean true is in distance and false is still to far away
*//*
bool Fan::flameProximity(int dist){
if(frontUltra.avg() < dist){
return true;}
else{
return false;
}
}
*/

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
