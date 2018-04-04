#include <Arduino.h>
#include "FireSensor.h"
#include "Motor.h"

//Object Creation
FireSensor fireSensor;

//Still have to implement the revesre part
Motor leftMotor(29,7,true);

void setup() {
    //Fire Sensor
    pinMode(29,OUTPUT);
    pinMode(7, OUTPUT);
    fireSensor.initialize(); //this initializes the fire sensor
    leftMotor.motorSetup();

}

void loop() {
    //Fire Sensor hey tye something
    fireSensor.useSensor();
    fireSensor.showAll();
    //-----------Works-------------
    // digitalWrite(29, LOW);
    // analogWrite(7,255);
    // delay(1000);
    // digitalWrite(29, HIGH);
    // analogWrite(7,255);
    // delay(1000);
    //---------------------------
    //--------Testing------------
    leftMotor.setPower(255);
  //  leftDrive.setPower(50);
}
