/*
 * Messages.h
 *
 *  Created on: 10/1/16
 *      Author: joest
 */
#include "PID.h"
#include "Arduino.h"

//Class constructor
PID::PID(){
  
}

//Function to set PID gain values
void PID::setpid(float P, float I, float D){
  kp=P;
  ki=I;
  kd=D;
}

//Write this function to calculate a control signal from the set velocity 
//and the current velocity 
float PID::calc(double setVel, double curVel){
    
    
    // calculate error
    double error = setVel - curVel;

    // calculate derivative of error
    double derivative = kd * (error - last_error);
    last_error = error;

    // calculate integral error. Running average is best but hard to implement
    float average = (error + sum_error)/2;
    sum_error += error;
    
    double integral = ki/average;
    
    
    // sum up the error value to send to the motor based off gain values. 
     double controlSignal = (kp * error) + derivative + integral;
 
    // limit control value to 0-254
    if(controlSignal < 0){
      controlSignal = 0;
    }
    if(controlSignal > 254){
      controlSignal = 254;
    }
    
    //return the control signal
    
    return controlSignal;

}
