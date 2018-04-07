
#ifndef PID_H_
#define PID_H_

class PID {

public:
PID();
float kp;
float ki;
float kd;
float last_error = 0;
float sum_error =0;
//set PID constants
void setpid(float P, float I, float D);
//calculate the output control signal
float calc(int, int);

private:

};



#endif
