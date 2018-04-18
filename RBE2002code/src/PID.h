
#ifndef PID_H_
#define PID_H_


/************************************************
 * Class for implementing PID control algorithm *
 ************************************************/

class PID {
public:
  PID();
  float kp;
  float ki;
  float kd;
  float last_error = 0;
  float sum_error =0;
  void setpid(float P, float I, float D); //set PID constants
  float calc(int, int);  //calculate the output control signal
};

#endif
