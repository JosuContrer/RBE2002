#ifndef GLOBALPINS_H
#define GLOBALPINS_H

#include <Arduino.h>

//Drive Train
//Motor(digitalPin,analogPin,isReverse);
#define DLEFTMOTOR 27
#define ALEFTMOTOR 7
//boolean isMotorLeftReverse = true;
#define BUTTON 20
#define DRIGHTMOTOR 29
#define ARIGHTMOTOR 6
//boolean isMotorRightReverse = false;

#define BACKLEFTULTRATRIG 22
#define BACKLEFTULTRAECHO 23

#define FRONTLEFTULTRATRIG 24
#define FRONTLEFTULTRAECHO 25

#define FRONTULTRATRIG 10 //NEED TO CHANGE
#define FRONTULTRAECHO 9 //NEED TO CHANGE



//FOR LINE FOLLOWER
#define NUM_SENSORS             6 // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4 // average 4 analog samples per sensor reading
#define EMITTER_PIN             2 // emitter is controlled by digital pin 2
#define addrCalibratedMinimumOn 0
#define addrCalibratedMaximumOn 100


#endif
