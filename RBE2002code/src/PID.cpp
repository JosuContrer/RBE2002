#include "PID.h"
#include "Arduino.h"

/**
 * PID Class constructor
 */
PID::PID(){
}


/**
 * Set PID gain values
 * @param P Proportional constant
 * @param I Integral constant
 * @param D Derivative constant
 */
void PID::setpid(float P, float I, float D){
  kp=P;
  ki=I;
  kd=D;
}


/**
 * Calculates PID using input values
 * @param  sensorOne Input value that sensorTwo is compared to
 * @param  sensorTwo Input value compared against sensorOne
 * @return           PID-calculated error
 */
float PID::calc(int sensorOne, int sensorTwo){

    // calculate error
    double error = sensorOne - sensorTwo;

    // calculate derivative of error
    double derivative = kd * (error - last_error);
    last_error = error;

    // calculate integral error
    sum_error += error;

    //Wrap Protection
    if (error==0){
      sum_error=0; //reset dont oscillate around the desired point
    }
    if(derivative<0){ //if  getting closer
      sum_error=0;
    }

    double integral = ki * sum_error;

    // sum up the error value
     double controlSignal = (kp * error) + derivative + integral;

    // limit control value to 0-254 *put in thr main dont add here*
    // if(controlSignal < 0){
    //   controlSignal = 0;
    // }
    // if(controlSignal > 254){
    //   controlSignal = 254;
    // }

    return controlSignal;

}
