#include "States.h"
#include "MotorStates.h"

MotorStates::MotorStates(){}

void MotorStates::initialize(){
//leftMotor.initialize();
driveTrain.initialize();
//lcd.begin(16, 2);
}

void MotorStates::motorDrive(testState test){
  //lcd2.clear();
  switch(test){
    case STOP2:
    //lcd2.setCursor(0, 1);
    //lcd2.print("STOP MODE");
    driveTrain.setPower(0,0);
    break;
    case STRAIGHT:
    //lcd2.setCursor(0, 1);
    //lcd2.print("STRAIGHT MODE");
    driveTrain.setPower(255,255);
    break;
    case TURNLEFT:
    //lcd2.setCursor(0, 1);
    //lcd2.print("TURNLEFT MODE");
    driveTrain.setPower(0, 255);
    break;
    case TURNRIGHT:
    //lcd2.setCursor(0, 1);
    //lcd2.print("TURNRIGHT MODE");
    driveTrain.setPower(255,0);
    break;
    case BACKWARDS:
    //lcd2.setCursor(0, 1);
    //lcd2.print("BACKWARDS MODE");
    driveTrain.setPower(-255,-255);
    break;
  //  case ONEMOTOR: //left motor working // right -
    //lcd2.setCursor(0, 1);
    //lcd2.print("ONEMOTOR MODE");
    //leftMotor.setPower(255);
    //break;
    case TURNLEFTCENTER:
    //lcd2.setCursor(0,1);
    //lcd2.print("LEFTCENTERTURN MODE");
    driveTrain.setPower(-255,255);
    break;
    case TURNRIGHTCENTER:
    //lcd2.setCursor(0,1);
    //lcd2.print("RIGHTCENTER MODE");
    driveTrain.setPower(255,-255);
    break;
  }
}
