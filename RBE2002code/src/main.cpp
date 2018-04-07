#include <Arduino.h>
#include "FireSensor.h"
//#include "Motor.h"
#include "drive.h"
#include "Ultrasonic.h"
#include "globalPins.h"

//Object Creation
FireSensor fireSensor;
drive driveTrain;

Ultrasonic backLeftUltra(BACKLEFTULTRATRIG, BACKLEFTULTRAECHO);
Ultrasonic frontLeftUltra(FRONTLEFTULTRATRIG, FRONTLEFTULTRAECHO);

//Still have to implement the revesre part
//Motor(digitalPin,analogPin,isReverse);
// Motor leftMotor(29,7,true);
// Motor rightMotor(28,6,false);

void setup() {
    //Fire Sensor
    // pinMode(29,OUTPUT);
    // pinMode(28,OUTPUT);
    // pinMode(6,OUTPUT);
    // pinMode(7, OUTPUT);
    fireSensor.initialize(); //this initializes the fire sensor
    // leftMotor.initialize();
    // rightMotor.initialize();
    driveTrain.initialize();
    Serial.begin(9600);


}

void loop() {
    //Fire Sensor hey tye something
    //fireSensor.useSensor();
    //fireSensor.showAll();
    //-----------Works-------------
    // digitalWrite(29, LOW);
    // digitalWrite(28, LOW);
    // analogWrite(7,255);
    // analogWrite(6,255);
    // delay(1000);
    // digitalWrite(29, HIGH);
    // digitalWrite(28, HIGH);
    // analogWrite(7,255);
    // analogWrite(6,255);
    // delay(1000);
    //---------------------------
    //--------Testing------------
  // leftMotor.setPower(255);
  // rightMotor.setPower(255);
  //
  //driveTrain.setPower(-100,100);
  frontLeftUltra.readDistance();
  //backLeftUltra.readDistance();
  //  leftDrive.setPower(50);
}
