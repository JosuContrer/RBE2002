#ifndef GLOBALPINS_H
#define GLOBALPINS_H

#include <Arduino.h>

//FOR THE DRIVETRAIN
//Motor(digitalPin,analogPin,isReverse);
#define DLEFTMOTOR 27
#define ALEFTMOTOR 7
//boolean isMotorLeftReverse = true;
#define DRIGHTMOTOR 29
#define ARIGHTMOTOR 6
//boolean isMotorRightReverse = false;

//FOR THE BUTTON
#define BUTTON 20

//FOR THE ULTRASONIC SENSORS
#define BACKLEFTULTRATRIG 22
#define BACKLEFTULTRAECHO 23

#define FRONTLEFTULTRATRIG 24
#define FRONTLEFTULTRAECHO 25

#define FRONTULTRATRIG 10
#define FRONTULTRAECHO 9

#define SIDEULTRATRIG 17
#define SIDEULTRAECHO 16

//FOR LINE FOLLOWER
#define NUM_SENSORS             6 // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4 // average 4 analog samples per sensor reading
#define EMITTER_PIN             2 // emitter is controlled by digital pin 2
#define addrCalibratedMinimumOn 0
#define addrCalibratedMaximumOn 100

//FOR FAN
#define FANPIN 28
#define FANSERVOPIN 8

#endif
