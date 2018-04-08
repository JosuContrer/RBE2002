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
float PID::calc(int sensorOne, int sensorTwo){

    // calculate error
    double error = sensorOne - sensorTwo;

    // calculate derivative of error
    double derivative = kd * (error - last_error);
    last_error = error;

    // calculate integral error. Running average is best but hard to implement
    //float average = (error + sum_error)/2;
    sum_error += error;

    double integral = ki * sum_error;


    // sum up the error value to send to the motor based off gain values.
     double controlSignal = (kp * error) + derivative + integral;

    // limit control value to 0-254 *put in thr main dont add here*
    if(controlSignal < 0){
      //controlSignal = 0;
    }
    if(controlSignal > 254){
      //controlSignal = 254;
    }

    //return the control signal

    return controlSignal;

}
